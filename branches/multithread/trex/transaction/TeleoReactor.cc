/** @file TeleoReactor.cc
 * @brief Provides implementation for TeleoReactor
 *
 * @author Conor McGann @& Frederic Py <fpy@mbari.org>
 * @ingroup transaction
 */
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, MBARI.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the TREX Project nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */
#include <utility>
#include <cmath>

// #include <boost/chrono/clock_string.hpp>

#include "TeleoReactor.hh"
#include "private/node_impl.hh"
#include "private/graph_impl.hh"


#include <trex/domain/FloatDomain.hh>
#include <trex/utils/chrono_helper.hh>

#include <boost/scope_exit.hpp>

#include <bitset>

using TREX::utils::Symbol;
namespace utils=TREX::utils;

namespace TREX {
  namespace transaction {
    
    class TeleoReactor::Logger {
    public:
      Logger(std::string const &dest, boost::asio::io_service &io);
      ~Logger();
      
      void provide(Symbol const &name, bool goals, bool plan);
      void unprovide(Symbol const &name);
      
      void use(Symbol const &name , bool goals, bool plan);
      void unuse(Symbol const &name);
      
      void init(TICK val);
      void newTick(TICK val);
      void synchronize();
      
      void failed();
      void has_work();
      void work(bool ret);
      void step();
      
      void observation(Observation const &obs);
      void request(goal_id const &goal);
      void recall(goal_id const &goal);
      
      void comment(std::string const &msg);
      void notifyPlan(goal_id const &tok);
      void cancelPlan(goal_id const &tok);
      
    private:
      boost::asio::strand           m_strand;
      boost::asio::io_service::work m_active;
      std::ofstream                 m_file;
      
      enum {
        header      = 0,
        tick        = 1,
        tick_opened = 2,
        in_phase    = 3,
        has_data    = 4
      };
      
      std::bitset<5> m_flags;
      
      enum tick_phase {
        in_init,
        in_new_tick,
        in_synchronize,
        in_work,
        in_step
      };
      
      tick_phase m_phase;
      TICK m_current;
      
      void obs(Observation o);
      void goal_event(std::string tag, goal_id g, bool full);
      
      void post_event(boost::function<void ()> fn);
      void phase_event(boost::function<void ()> fn);
      
      void open_tick();
      void open_phase();
      void close_phase();
      void close_tick();
      
      void set_tick(TICK val, tick_phase p);
      void set_phase(tick_phase p);
      void direct_write(std::string const &content, bool nl);
      
    };
    
  }
}



using namespace TREX::transaction;

/*
 * class TREX::transaction::ReactorException
 */
ReactorException::ReactorException(TeleoReactor const &r,
                                   std::string const &msg) throw()
  :GraphException(r.m_graph, r.getName().str(), msg) {}


/*
 * class TREX::transaction::DispatchError
 */
// statics
std::string DispatchError::build_msg(goal_id const &g, std::string const &msg) throw() {
  std::ostringstream oss;
  oss<<"While dispatching ";
  if( !!g )
    oss<<g->object()<<'.'<<g->predicate();
  oss<<'['<<g<<"]: "<<msg;
  return oss.str();
}



/*
 * class TREX::transaction::details::external
 */

bool TREX::transaction::details::external::cmp_goals(IntegerDomain const &a, IntegerDomain const &b) {
  // sorting order
  //   - based on upperBound
  //   - if same upperBound : sorted based on lower bound
  //
  // this way I can safely update lower bounds without impacting tokens order
  return a.upperBound()<b.upperBound() ||
    ( a.upperBound()==b.upperBound() && a.lowerBound()<b.lowerBound() );
}


// structors

TREX::transaction::details::external::external() {
  m_pos = m_last; // may not be necessary but I just want  ot be sure ...
}

TREX::transaction::details::external::external(details::external const &other)
  :m_pos(other.m_pos), m_last(other.m_last) {}

TREX::transaction::details::external::external(details::external_set::iterator const &pos,
                                               details::external_set::iterator const &last)
  :m_pos(pos), m_last(last) {}

// manipulators

details::goal_queue::iterator details::external::lower_bound(IntegerDomain const &dom) {
  details::goal_queue::iterator i = m_pos->second.begin();

  for(; m_pos->second.end()!=i && cmp_goals(i->first->getStart(), dom); ++i);
  return i;
}

// modifiers

bool details::external::post_goal(goal_id const &g) {
  IntegerDomain const &g_start(g->getStart());
  // locate goal position
  details::goal_queue::iterator i = lower_bound(g_start);

  // check that it is not already posted
  for( ; m_pos->second.end()!=i && !cmp_goals(g_start, i->first->getStart()); ++i)
    if( g==i->first )
      return false;
  // insert the new goal
  m_pos->second.insert(i, std::make_pair(g, true));
  if( m_pos->first.client().is_verbose() ) {
    syslog(info)<<m_pos->first.client().getName()
      <<" added "<<g->predicate()<<'['<<g<<"] to the pending queue of "
      <<m_pos->first.name();
  }
  return true;
}

void details::external::dispatch(TICK current, details::goal_queue &sent) {
  details::goal_queue::iterator i=m_pos->second.begin();
  IntegerDomain dispatch_w = m_pos->first.dispatch_window(current);

  for( ; m_pos->second.end()!=i && i->first->startsBefore(dispatch_w.upperBound());  ) {
    bool future = i->first->startsAfter(current);

    if( future || i->first->endsAfter(current+1) ) {
      // Need to check for dispatching
        if( i->second && m_pos->first.accept_goals() ) {
          if( m_pos->first.client().is_verbose() )
            syslog(info)<<"Dispatching "<<i->first->predicate()
                        <<'['<<(i->first)<<"] on \""
                        <<m_pos->first.name()<<"\".";
          bool posted = false;
          try {
            m_pos->first.request(i->first);
            posted = true;
            i = m_pos->second.erase(i);
          } catch(utils::Exception const &e) {
            syslog(warn)<<"Exception received while sending request: "<<e;
          } catch(std::exception const &se) {
            syslog(warn)<<"C++ exception received while sending request: "
                        <<se.what();
          } catch(...) {
            syslog(warn)<<"Unknown exception received while sending request.";
          }
          if( !posted ) {
            syslog(warn)<<"Marking goal as non-postable.";
            i->second = false;
          }
        } else
          ++i;
    } else if( !future ) {
      syslog(warn)<<"Goal "<<i->first->predicate()<<'['<<(i->first)
                  <<"] is in the past: removing it\n\t"<<(*(i->first));
      i = m_pos->second.erase(i);
    } else if( !m_pos->first.accept_goals() )
      break; // no need to  look further ... this guy do not accept goals
  }
}

void details::external::unblock() {
  // just mark all the 
  for(details::goal_queue::iterator i=m_pos->second.begin();
      m_pos->second.end()!=i; ++i) 
    i->second = true;
}

void details::external::recall(goal_id const &g) {
  IntegerDomain const &g_start(g->getStart());
  // locate goal position
  details::goal_queue::iterator i = lower_bound(g_start);
  for( ; m_pos->second.end()!=i && !cmp_goals(g_start, i->first->getStart()); 
       ++i) {
    if( g==i->first ) {
      // was still pending => just remove it
      m_pos->second.erase(i);
      return;
    }
  }
  // not found => send a recall
  m_pos->first.recall(g);
}

void details::external::increment() {
  if( valid() ) {
    ++m_pos;
  }
}

// observers

bool details::external::equal(details::external const &other) const {
  if ( valid() )
    return m_pos == other.m_pos;
  else
    return !other.valid();
}

TREX::utils::log::stream details::external::syslog(utils::Symbol const &kind) {
  return m_pos->first.client().syslog(m_pos->first.name(), kind);
}

Relation const &details::external::dereference() const {
  return *(m_pos->first);
}

/*
 * class TREX::transaction::TeleoReactor
 */

utils::Symbol const TeleoReactor::obs("ASSERT");
utils::Symbol const TeleoReactor::plan("PLAN");


// structors

TeleoReactor::TeleoReactor(TeleoReactor::xml_arg_type &arg, bool loadTL,
                           bool log_default)
:m_impl(arg.second->new_node(utils::parse_attr<Symbol>(xml_factory::node(arg), "name")))
,m_inited(false), m_firstTick(true), m_graph(*(arg.second)), m_have_goals(0)
,m_verbose(utils::parse_attr<bool>(arg.second->is_verbose(),
                                   xml_factory::node(arg), "verbose"))
,m_trLog(NULL), m_latency(utils::parse_attr<TICK>(xml_factory::node(arg), "latency"))
,m_maxDelay(0), m_lookahead(utils::parse_attr<TICK>(xml_factory::node(arg), "lookahead"))
,m_nSteps(0), m_past_deadline(false), m_validSteps(0) {
  boost::property_tree::ptree::value_type &node(xml_factory::node(arg));

  utils::LogManager::path_type fname = file_name("stat.csv");
  m_stat_log.open(fname.c_str());
  m_stat_log<<"tick, synch_ns, delib_ns, n_steps\n";
     
  if( utils::parse_attr<bool>(log_default, node, "log") ) {
    std::string base = getName().str()+".tr.log";
    fname = manager().file_name(base);
    m_trLog = new Logger(fname.string(), manager().service());
    utils::LogManager::path_type cfg = manager().file_name("cfg"), 
      pwd = boost::filesystem::current_path(), 
      short_name(base), location("../"+base);
    boost::filesystem::current_path(cfg);
    try {
      create_symlink(location, short_name);
    } catch(...) {}
    boost::filesystem::current_path(pwd);
    syslog(info)<<"Transactions logged to "<<fname;
  }

  if( loadTL ) {
    utils::Symbol tl_name;
    // Add external file content
    utils::ext_xml(node.second, "config");

    for(boost::property_tree::ptree::iterator i=node.second.begin();
        node.second.end()!=i; ++i) {
      if( utils::is_tag(*i, "External") ) {
        tl_name = utils::parse_attr<Symbol>(*i, "name");
        if( tl_name.empty() )
          throw utils::XmlError(*i, "Timelines cannot have an empty name");
        use(tl_name, utils::parse_attr<bool>(true, *i, "goals"),
            utils::parse_attr<bool>(false, *i, "listen"));
      } else if( utils::is_tag(*i, "Internal") ) {
        tl_name = utils::parse_attr<Symbol>(*i, "name");
        if( tl_name.empty() )
          throw utils::XmlError(*i, "Timelines cannot have an empty name");
        provide(tl_name);
      }
    }
  }
}

TeleoReactor::TeleoReactor(graph *owner, Symbol const &name,
                           TICK latency, TICK lookahead, bool log)
:m_impl(owner->new_node(name)), m_inited(false), m_firstTick(true)
,m_graph(*owner), m_have_goals(0), m_verbose(owner->is_verbose())
,m_trLog(NULL), m_latency(latency), m_maxDelay(0), m_lookahead(lookahead)
,m_nSteps(0) {
  utils::LogManager::path_type fname = file_name("stat.csv");
  m_stat_log.open(fname.c_str());
     
  if( log ) {
    fname = manager().file_name(getName().str()+".tr.log");
    m_trLog = new Logger(fname.string(), manager().service());
    syslog(info)<<"Transactions logged to "<<fname;

  }
}

TeleoReactor::~TeleoReactor() {
  isolate(false);
  if( NULL!=m_trLog ) {
    delete m_trLog;
  }
}

// observers

utils::Symbol TeleoReactor::getName() const {
  // TODO ensure that this call is robust
  return m_impl.lock()->name();
}


TeleoReactor::size_type TeleoReactor::count_internal_relations() const {
  size_type result(0);

  for(internal_set::const_iterator i=m_internals.begin(); m_internals.end()!=i; ++i)
    result += (*i)->size();
  return result;
}


bool TeleoReactor::internal_sync(TREX::utils::Symbol name) const {
  return m_internals.end()!=m_internals.find(name);
}

bool TeleoReactor::isInternal(TREX::utils::Symbol const &timeline) const {
  boost::function<bool ()> fn(boost::bind(&TeleoReactor::internal_sync, this, timeline));
  return utils::strand_run(m_graph.strand(), fn);
}

bool TeleoReactor::external_sync(TREX::utils::Symbol name) const {
  return m_externals.end()!=m_externals.find(name);
}


bool TeleoReactor::isExternal(TREX::utils::Symbol const &timeline) const {
  boost::function<bool ()> fn(boost::bind(&TeleoReactor::external_sync, this, timeline));
  return utils::strand_run(m_graph.strand(), fn);
}

details::external TeleoReactor::ext_begin() {
  return details::external(m_externals.begin(), m_externals.end());
}

details::external TeleoReactor::ext_end() {
  return details::external(m_externals.end(), m_externals.end());
}

details::external TeleoReactor::find_external(TREX::utils::Symbol const &name) {
  return details::external(m_externals.find(name), m_externals.end());
}

// modifers/callbacks

utils::log::stream
TeleoReactor::syslog(utils::log::id_type const &context, utils::log::id_type const &kind) const {
  return m_impl.lock()->syslog(context, kind);
}


void TeleoReactor::reset_deadline() {
  // initialize the deliberation parameters
  m_deadline = getCurrentTick()+1+getLatency();
  m_nSteps = 0;  
  m_past_deadline = false;
}

bool TeleoReactor::have_goals() {
  utils::SharedVar<size_t>::scoped_lock lock(m_have_goals);
  return 0 < *m_have_goals;
}

void TeleoReactor::goal_flush(std::list<goal_id> &a, std::list<goal_id> &dest) {
  {
    utils::SharedVar<size_t>::scoped_lock lock(m_have_goals);
    *m_have_goals -= a.size();
  }
  std::swap(a, dest);
}


void TeleoReactor::queue(std::list<goal_id> &l, goal_id g) {
  {
    utils::SharedVar<size_t>::scoped_lock lock(m_have_goals);
    *m_have_goals += 1;
  }
  l.push_back(g);
}


void TeleoReactor::queue_goal(goal_id g) {
  m_graph.strand().dispatch(boost::bind(&TeleoReactor::queue, this,
                                        boost::ref(m_sync_goals), g));
}

void TeleoReactor::queue_recall(goal_id g) {
  m_graph.strand().dispatch(boost::bind(&TeleoReactor::queue, this,
                                        boost::ref(m_sync_recalls), g));
}

void TeleoReactor::queue_token(goal_id g) {
  m_graph.strand().dispatch(boost::bind(&TeleoReactor::queue, this,
                                        boost::ref(m_sync_toks), g));
}

void TeleoReactor::queue_cancel(goal_id g) {
  m_graph.strand().dispatch(boost::bind(&TeleoReactor::queue, this,
                                        boost::ref(m_sync_cancels), g));
}


double TeleoReactor::workRatio() {
  std::list<goal_id> tmp;

  if( have_goals() ) {
    // Start to flush goals
    boost::function<void ()> fn(boost::bind(&TeleoReactor::goal_flush, this,
                                            boost::ref(m_sync_goals),
                                            boost::ref(tmp)));
    utils::strand_run(m_graph.strand(), fn);
    
    while( !tmp.empty() ) {
      handleRequest(tmp.front());
      tmp.pop_front();
    }
  }
  if( have_goals() ) {
    // Start to flush recalls
    boost::function<void ()> fn(boost::bind(&TeleoReactor::goal_flush, this,
                                            boost::ref(m_sync_recalls),
                                            boost::ref(tmp)));
    utils::strand_run(m_graph.strand(), fn);
    while( !tmp.empty() ) {
      handleRecall(tmp.front());
      tmp.pop_front();
    }
  }
  if( have_goals() ) {
    // Start to flush plan tokens
    boost::function<void ()> fn(boost::bind(&TeleoReactor::goal_flush, this,
                                            boost::ref(m_sync_toks),
                                            boost::ref(tmp)));
    utils::strand_run(m_graph.strand(), fn);
    while( !tmp.empty() ) {
      newPlanToken(tmp.front());
      tmp.pop_front();
    }
  }
  if( have_goals() ) {
    // Start to flush plan tokens
    boost::function<void ()> fn(boost::bind(&TeleoReactor::goal_flush, this,
                                            boost::ref(m_sync_cancels),
                                            boost::ref(tmp)));
    utils::strand_run(m_graph.strand(), fn);
    while( !tmp.empty() ) {
      cancelPlanToken(tmp.front());
      tmp.pop_front();
    }
  }
  
  bool ret = false;
  
  if( NULL!=m_trLog )
    m_trLog->has_work();
  try {
    ret = hasWork();
    if( NULL!=m_trLog )
      m_trLog->work(ret);

    if( ret ) {
      double ret = m_deadline;
      ret -= getCurrentTick();
      if( ret<0.0 && m_nSteps>0 ) {
        if( !m_past_deadline ) {
          m_past_deadline = true;
          m_validSteps = m_nSteps;
          syslog(warn)<<" Reactor is now exceeding its deliberation latency ("
          <<getLatency()<<")\n\tNumber of steps within its latency: "<<m_validSteps;
        }
        ret = m_nSteps+1;
      } else {
        ret += 1.0;
        ret *= m_nSteps+1;
      }
      return 1.0/ret;
    } else {
      // Dispatched goals management
      details::external i = ext_begin();
      details::goal_queue dispatched; // store the goals that got dispatched 
                                      // on this tick ...
                                      // I do nothing with it for now
      
      // Manage goal dispatching
      for( ; i.valid(); ++i )
        i.dispatch(getCurrentTick()+1, dispatched);
        
    }
  } catch(std::exception const &se) {
    syslog(warn)<<"Exception during hasWork question: "<<se.what();
  } catch(...) {
    syslog(warn)<<"Unknown Exception during hasWork question";
  }
  if( m_past_deadline ) {
    syslog(warn)<<"Reactor needed to deliberate "<<(m_nSteps-m_validSteps)
                      <<" extra steps spread other "
                      <<(getCurrentTick()-m_deadline)
                      <<" ticks after its latency."; 
  }
  reset_deadline();
  return NAN;
}

void TeleoReactor::observation_sync(Observation o, TICK date, bool verbose) {
  internal_set::iterator i = m_internals.find(o.object());
  
  if( m_internals.end()==i )
    throw boost::enable_current_exception(SynchronizationError(*this, "attempted to post observation on "+
                               o.object().str()+" which is not Internal."));
  
  (*i)->postObservation(date, o, verbose);
  m_updates.insert(*i);
}

void TeleoReactor::postObservation(Observation const &obs, bool verbose) {
  boost::function<void ()> fn(boost::bind(&TeleoReactor::observation_sync,
                                          this, obs, m_obsTick, verbose));
  utils::strand_run(m_graph.strand(), fn);
}

bool TeleoReactor::goal_sync(goal_id g) {
  details::external tl(m_externals.find(g->object()), m_externals.end());
  if( tl.valid() ) {
    if( NULL!=m_trLog )
      m_trLog->request(g);
    return tl.post_goal(g);
  } else
    throw boost::enable_current_exception(DispatchError(*this, g, "Goals can only be posted on External timelines"));
}


bool TeleoReactor::postGoal(goal_id const &g) {
  if( !g )
    throw DispatchError(*this, g, "Invalid goal Id");

  boost::function<bool ()> fn(boost::bind(&TeleoReactor::goal_sync,
                                          this, g));
  return utils::strand_run(m_graph.strand(), fn);
}

goal_id TeleoReactor::postGoal(Goal const &g) {
  goal_id tmp(new Goal(g));

  if( postGoal(tmp) )
    return tmp;
  else {
    // should never happen !?
    return goal_id();
  }
}

goal_id TeleoReactor::parse_goal(boost::property_tree::ptree::value_type const &g) {
  return getGraph().parse_goal(g);
}

bool TeleoReactor::recall_sync(goal_id g) {
  details::external tl(m_externals.find(g->object()), m_externals.end());
  
  if( tl.valid() ) {
    if( NULL!=m_trLog )
      m_trLog->recall(g);
    tl.recall(g);
    return true;
  }
  return false;
}


bool TeleoReactor::postRecall(goal_id const &g) {
  if( !g )
    return false;
  boost::function<bool ()> fn(boost::bind(&TeleoReactor::recall_sync,
                                          this, g));
  return utils::strand_run(m_graph.strand(), fn);
}

bool TeleoReactor::plan_sync(goal_id t) {
  
  // Look for the internal timeline
  internal_set::const_iterator tl = m_internals.find(t->object());
  if( m_internals.end()==tl )
    throw boost::enable_current_exception(DispatchError(*this, t, "plan tokens can only be posted on Internal timelines."));
  else if( t->getEnd().upperBound() > getCurrentTick() ) {
    if( NULL!=m_trLog )
      m_trLog->notifyPlan(t);
    return (*tl)->notifyPlan(t);
  }
  return false;
}

void TeleoReactor::cancel_sync(goal_id tok) {
  internal_set::const_iterator tl = m_internals.find(tok->object());
  if( m_internals.end()!=tl ) {
    // do something
    if( NULL!=m_trLog )
      m_trLog->cancelPlan(tok);
    
    (*tl)->cancelPlan(tok);
  } 
}

bool TeleoReactor::postPlanToken(goal_id const &t) {
  if( !t )
    throw DispatchError(*this, t, "Invalid token id");
  
  boost::function<bool ()> fn(boost::bind(&TeleoReactor::plan_sync,
                                          this, t));
  return utils::strand_run(m_graph.strand(), fn);
}

goal_id TeleoReactor::postPlanToken(Goal const &g) {
  goal_id tmp(new Goal(g));
  
  if( postPlanToken(tmp) )
    return tmp;
  else 
    return goal_id();
}

void TeleoReactor::cancelPlanToken(goal_id const &g) {
  if( g ) {
    boost::function<void ()> fn(boost::bind(&TeleoReactor::cancel_sync,
                                            this, g));
    utils::strand_run(m_graph.strand(), fn);
  }
}


TICK TeleoReactor::getFinalTick() const {
  TICK g_final = m_graph.finalTick();
  
  if( !m_finalTick || g_final<*m_finalTick )
    return g_final;
  else
    return *m_finalTick;
}


void TeleoReactor::setMaxTick(TICK max) {
  if( !m_finalTick || max<*m_finalTick ) {
    syslog(warn)<<"Restricted reactor final tick to "<<max;
    m_finalTick = max;
  }
}


bool TeleoReactor::initialize(TICK final) {
  if( m_inited ) {
    syslog(error)<< "Attempted to initalize this reactor twice.";
    return false;
  }
  m_initialTick = m_obsTick = getCurrentTick();
  
  if( !m_finalTick || final<*m_finalTick )
    m_finalTick   = final;
  syslog(info)<<"Creation tick is "<<getInitialTick();
  syslog(info)<<"Execution latency is "<<getExecLatency();
  // syslog()<<"Clock used for stats is "<<boost::chrono::clock_string<stat_clock, char>::name();
  try {
    if( NULL!=m_trLog )
      m_trLog->init(m_initialTick);
    handleInit();   // allow derived class initialization
    m_firstTick = true;
    m_inited = true;
    return true;
  } catch(TREX::utils::Exception const &e) {
    syslog(error)<<"Exception caught during init :\n"<<e;
  } catch( std::exception const &se) {
    syslog(error)<<"C++ exception caught during init :\n"<<se.what();
  } catch(...) {
    syslog(error)<<"Unknown exception caught during init";
  }
  return false;
}

bool TeleoReactor::newTick() {
  if( m_firstTick ) {
    m_obsTick = getCurrentTick();
    if( m_obsTick!=m_initialTick ) {
      syslog(warn)<<"Updating initial tick from "<<m_initialTick
                    <<" to "<<getCurrentTick();
      m_initialTick = m_obsTick;
    }
    reset_deadline();
    TICK final = getFinalTick();
    
    if( final < m_graph.finalTick() )
      syslog(warn)<<"Reactor final tick is before agent's one:\n\t"
        <<date_str(final)<<" ("<<final<<").";
    
    m_firstTick = false;
  } else 
    m_stat_log<<(getCurrentTick()-1)<<", "
              <<m_synch_usage.count()
              <<", "<<m_deliberation_usage.count()
              <<", "<<m_tick_steps<<std::endl;
  m_tick_steps = 0;
  
  if( getCurrentTick()>getFinalTick() ) {
    syslog(warn)<<"This reactor reached its final tick.";
    return false;
  }
  
//  if( m_deliberation_usage > stat_duration::zero() )
//    syslog("stats")<<" delib="<<boost::chrono::duration_short<<m_deliberation_usage;
  m_deliberation_usage = stat_duration::zero();
  
  if( NULL!=m_trLog )
    m_trLog->newTick(getCurrentTick());

  try {
    handleTickStart(); // allow derived class processing

    // Dispatched goals management
    details::external i = ext_begin();
    details::goal_queue dispatched; // store the goals that got dispatched on this tick ...
                                    // I do nothing with it for now
    
    // Manage goal dispatching
    for( ; i.valid(); ++i )
      i.dispatch(getCurrentTick(), dispatched);
    return true;
  } catch(TREX::utils::Exception const &e) {
    syslog(error)<<"Exception caught during new tick:\n"<<e;
  } catch(std::exception const &se) {
    syslog(error)<<"C++ exception caught during new tick:\n"<<se.what();
  } catch(...) {
    syslog(error)<<"Unknown exception caught during new tick";    
  }
  return false;
}

void TeleoReactor::collect_obs_sync(std::list<Observation> &l) {
  for(external_set::iterator i = m_externals.begin();
      m_externals.end()!=i; ++i) {
    // syslog(info)<<"Checking for new observation on "<<i->first.name();
    if( i->first.lastObsDate()==getCurrentTick() ) {
      // syslog(info)<<"Collecting new obs: "<<i->first.lastObservation();
      l.push_back( i->first.lastObservation() );
    } //else
     // syslog(info)<<"Last observation date ("<<i->first.lastObsDate()<<") is before current tick";
  }
  
}


void TeleoReactor::doNotify() {
  std::list<Observation> obs;
  boost::function<void ()> fn(boost::bind(&TeleoReactor::collect_obs_sync,
                                          this, boost::ref(obs)));
  utils::strand_run(m_graph.strand(), fn);
  for(std::list<Observation>::const_iterator i=obs.begin(); obs.end()!=i; ++i)
    notify(*i);
}


bool TeleoReactor::doSynchronize() {
  if( NULL!=m_trLog )
    m_trLog->synchronize();
  try {
    bool success;
    {
      // collect information from external timelines 
      doNotify();
      success = synchronize();      
    }
    if( success ) {
      for(internal_set::const_iterator i=m_updates.begin();
          m_updates.end()!=i; ++i) {
        bool echo;
        Observation const &observ = (*i)->lastObservation(echo);
        
        if( echo || is_verbose() || NULL==m_trLog )
          syslog(obs)<<observ;
        if( NULL!=m_trLog )
          m_trLog->observation(observ);
      }
      m_updates.clear();
    }
    m_obsTick = m_obsTick+1;
    return success;
  } catch(utils::Exception const &e) {
    syslog(error)<<"Exception caught: "<<e;
  } catch(std::exception const &se) {
    syslog(error)<<"C++ exception caught: "<<se.what();
  } catch(...) {
    syslog(error)<<"Unknown exception caught.";
  }
  syslog(error)<<"Failed to synchronize.";
  return false;
}

void TeleoReactor::step() {
  if( NULL!=m_trLog )
    m_trLog->step();
  stat_clock::time_point start = stat_clock::now();
  resume();
  stat_clock::duration delta = stat_clock::now()-start;
  m_deliberation_usage += delta;
  m_nSteps += 1;
  m_tick_steps +=1;
}

void TeleoReactor::use_sync(TREX::utils::Symbol name, details::transaction_flags f) {
  if( !m_graph.subscribe(this, name, f) ) {
    if( internal_sync(name) )
      syslog(warn)<<"External declaration of the Internal timeline \""<<name<<"\"";
    else
      syslog(warn)<<"Multiple External declarations of timeline \""<<name<<"\"";
  }
}


void TeleoReactor::use(TREX::utils::Symbol const &timeline, bool control, bool plan_listen) {
  details::transaction_flags flag; // initialize all the flags to 0
  flag.set(0,control);        // update the control flag
  flag.set(1,plan_listen);    // update the plan_listen flag
  
  boost::function<void ()> fn(boost::bind(&TeleoReactor::use_sync, this, timeline, flag));
  utils::strand_run(m_graph.strand(), fn);
}

void TeleoReactor::provide_sync(TREX::utils::Symbol name, details::transaction_flags f) {
  if( !m_graph.assign(this, name, f) )
    if( internal_sync(name) ) {
      syslog(warn)<<"Promoted \""<<name<<"\" from External to Internal with rights "
      <<details::access_str(f.test(0), f.test(1));
    }
}


void TeleoReactor::provide(TREX::utils::Symbol const &timeline,
                           bool controllable, bool publish) {
  details::transaction_flags flag;
  flag.set(0, controllable);
  flag.set(1, publish);
 
  boost::function<void ()> fn(boost::bind(&TeleoReactor::provide_sync, this, timeline, flag));
  utils::strand_run(m_graph.strand(), fn);
}

void TeleoReactor::tr_info(std::string const &msg) {
  if( NULL!=m_trLog ) {
    m_trLog->comment(msg);
  }
}

bool TeleoReactor::unuse_sync(TREX::utils::Symbol name) {
  external_set::iterator i = m_externals.find(name);
  if( m_externals.end()!=i ) {
    Relation r = i->first;
    r.unsubscribe();
    return true;
  }
  return false; 
}


bool TeleoReactor::unuse(TREX::utils::Symbol const &timeline) {
  boost::function<bool ()> fn(boost::bind(&TeleoReactor::unuse_sync, this, timeline));
  return utils::strand_run(m_graph.strand(), fn);
}

bool TeleoReactor::unprovide_sync(TREX::utils::Symbol name) {
  internal_set::iterator i = m_internals.find(name);
  if( m_internals.end()!=i ) {
    (*i)->unassign(getCurrentTick());
    return true;
  }
  return false;  
}

bool TeleoReactor::unprovide(TREX::utils::Symbol const &timeline) {
  boost::function<bool ()> fn(boost::bind(&TeleoReactor::unprovide_sync, this, timeline));
  return utils::strand_run(m_graph.strand(), fn);
}


void TeleoReactor::isolate(bool failed) {
  if( NULL!=m_trLog && failed ) {
    Logger *tmp = NULL;
    std::swap(tmp, m_trLog);
    tmp->failed();
    delete tmp;
  }
  
  clear_internals();
  clear_externals();

  // A bad cleanup -- just for now -- this should come from node_impl instead
  SHARED_PTR<details::node_impl> n = m_impl.lock();
  if( n ) {
    SHARED_PTR<details::graph_impl> g = n->graph();
    if( g )
      g->remove_node(m_impl);
  }
}


void TeleoReactor::clear_int_sync() {
  while( !m_internals.empty() ) {
    m_internals.front()->unassign(getCurrentTick());
  }
}

void TeleoReactor::clear_ext_sync() {
  while( !m_externals.empty() ) {
    Relation r = m_externals.begin()->first;
    r.unsubscribe();
  }  
}


void TeleoReactor::clear_internals() {
  boost::function<void ()> fn(boost::bind(&TeleoReactor::clear_int_sync, this));
  utils::strand_run(m_graph.strand(), fn);
}

void TeleoReactor::clear_externals() {
  boost::function<void ()> fn(boost::bind(&TeleoReactor::clear_ext_sync, this));
  utils::strand_run(m_graph.strand(), fn);
}

void TeleoReactor::assigned(details::timeline *tl) {
  m_internals.insert(tl);
  if( is_verbose() )
    syslog(null, info)<<"Declared \""<<tl->name()<<"\" with rights "<<tl->rights()<<".";
  if( NULL!=m_trLog ) {
    m_trLog->provide(tl->name(), tl->accept_goals(), tl->publish_plan());
  }
  for(graph::listen_set::const_iterator i=m_graph.m_listeners.begin();
      m_graph.m_listeners.end()!=i; ++i)
    (*i)->declared(*tl);
}

void TeleoReactor::unassigned(details::timeline *tl) {
  internal_set::iterator i = m_internals.find(tl);
  m_internals.erase(i);
  if( is_verbose() )
    syslog(null, info)<<"Undeclared \""<<tl->name()<<"\".";
  if( NULL!=m_trLog ) {
    m_trLog->unprovide(tl->name());
  }
  for(graph::listen_set::const_iterator i=m_graph.m_listeners.begin();
      m_graph.m_listeners.end()!=i; ++i)
    (*i)->undeclared(*tl);
}

void TeleoReactor::subscribed(Relation const &r) {
  external_set::value_type tmp;
  tmp.first = r;
  m_externals.insert(tmp);
  latency_updated(0, r.latency());
  if( is_verbose() )
    syslog(null, info)<<"Subscribed to \""<<r.name()<<"\" with rights "
      <<r.rights()<<'.';
  if( NULL!=m_trLog ) {
    m_trLog->use(r.name(), r.accept_goals(), r.accept_plan_tokens());
  }
  for(graph::listen_set::const_iterator i=m_graph.m_listeners.begin();
      m_graph.m_listeners.end()!=i; ++i) 
    (*i)->connected(r);
}

void TeleoReactor::unsubscribed(Relation const &r) {
  external_set::iterator i = m_externals.find(Relation::get_id(r));
  // No need to control that i is valid
  //    - this call comes from timeline::unsubscribe so
  //      which calls it only to the client of r -> me
  if( !i->second.empty() ) {
    latency_updated(r.latency(), 0);
  }
  // remove this relation
  m_externals.erase(i);
  if( is_verbose() ) 
    syslog(null, info)<<"Unsubscribed from \""<<r.name()<<"\".";
  if( NULL!=m_trLog ) {
    m_trLog->unuse(r.name());
  }
  for(graph::listen_set::const_iterator i=m_graph.m_listeners.begin();
      m_graph.m_listeners.end()!=i; ++i) 
    (*i)->disconnected(r);
}


void TeleoReactor::latency_updated(TICK old_l, TICK new_l) {
  TICK prev = m_maxDelay;

  if( new_l>m_maxDelay )
    m_maxDelay = new_l;
  else if( old_l==m_maxDelay ) {
    m_maxDelay = new_l;
    for(details::active_external i(ext_begin(), ext_end()), endi(ext_end());
        endi!=i; ++i) 
      m_maxDelay = std::max(m_maxDelay, i->latency());
  }
  if( m_maxDelay!=prev ) {
    // It may be anoying on the long run but for now I will log when this
    // exec latency changes
    syslog(info)<<" Execution latency updated from "<<prev<<" to "
                <<m_maxDelay;
    // Notify all the reactors that depend on me
    for(internal_set::iterator i=m_internals.begin(); m_internals.end()!=i; ++i)
      (*i)->latency_update(getLatency()+prev);
  }
}

void TeleoReactor::unblock(Symbol const &name) {
  details::external e=find_external(name);  
  if( e.active() )
    e.unblock();
}


/*
 * class TREX::transaction::TeleoReactor::Logger
 */

// structors

TeleoReactor::Logger::Logger(std::string const &dest, boost::asio::io_service &io)
:m_strand(io), m_active(io), m_file(dest.c_str()) {
  m_flags.set(header);
  m_strand.post(boost::bind(&Logger::direct_write, this, "<Log>\n <header>",
                            true));
}

TeleoReactor::Logger::~Logger() {
  //std::cerr<<"Destroy logger "<<std::endl;
  //std::cerr<<" - schedulle : close potential tick tag"<<std::endl;
  
  m_strand.post(boost::bind(&Logger::close_tick, this));
  //std::cerr<<" - build task : close the main tag"<<std::endl;
  boost::packaged_task<void> close_xml(boost::bind(&Logger::direct_write,
                                                   this, "</Log>",
                                                   true));
  //std::cerr<<" - get future of the task"<<std::endl;
  boost::unique_future<void> completed = close_xml.get_future();
  //std::cerr<<" - scedulled task execution"<<std::endl;
  m_strand.post(boost::bind(&boost::packaged_task<void>::operator(),
                            boost::ref(close_xml)));
  //std::cerr<<" - wait for task completion"<<std::endl;
  completed.wait();
  //std::cerr<<" - close the file"<<std::endl;
  m_file.close();
}

// interface

void TeleoReactor::Logger::comment(std::string const &msg) {
  m_strand.post(boost::bind(&Logger::direct_write, this, "<!-- "+msg+" -->",
                            true));
}

void TeleoReactor::Logger::init(TICK val) {
  m_strand.post(boost::bind(&Logger::set_tick, this, val, in_init));
}

void TeleoReactor::Logger::newTick(TICK val) {
  m_strand.post(boost::bind(&Logger::set_tick, this, val, in_new_tick));
}

void TeleoReactor::Logger::synchronize() {
  m_strand.post(boost::bind(&Logger::set_phase, this, in_synchronize));
}

void TeleoReactor::Logger::failed() {
  post_event(boost::bind(&Logger::direct_write,
                         this, "   <failed/>", true));
}

void TeleoReactor::Logger::has_work() {
  m_strand.post(boost::bind(&Logger::set_phase, this, in_work));
}

void TeleoReactor::Logger::step() {
  m_strand.post(boost::bind(&Logger::set_phase, this, in_step));
}

void TeleoReactor::Logger::work(bool ret) {
  std::ostringstream oss;
  oss<<"   <work value=\""<<ret<<"\" />";
  post_event(boost::bind(&Logger::direct_write,
                         this, oss.str(), true));
}

void TeleoReactor::Logger::provide(Symbol const &name, bool goals, bool plan) {
  std::ostringstream oss;
  oss<<"   <provide name=\""<<name
  <<"\" goals=\""<<goals
  <<"\" plan=\""<<plan<<"\" />";
  post_event(boost::bind(&Logger::direct_write,
                         this, oss.str(), true));
}

void TeleoReactor::Logger::unprovide(Symbol const &name) {
  std::ostringstream oss;
  oss<<"   <unprovide name=\""<<name<<"\" />";
  post_event(boost::bind(&Logger::direct_write,
                         this, oss.str(), true));  
}

void TeleoReactor::Logger::use(Symbol const &name, bool goals, bool plan) {
  std::ostringstream oss;
  oss<<"   <use name=\""<<name
  <<"\" goals=\""<<goals
  <<"\" plan=\""<<plan<<"\" />";
  post_event(boost::bind(&Logger::direct_write,
                         this, oss.str(), true));
}

void TeleoReactor::Logger::unuse(Symbol const &name) {
  std::ostringstream oss;
  oss<<"   <unuse name=\""<<name<<"\" />";
  post_event(boost::bind(&Logger::direct_write,
                         this, oss.str(), true));
}

void TeleoReactor::Logger::observation(Observation const &o) {
  post_event(boost::bind(&Logger::obs, this, o));
}

void TeleoReactor::Logger::request(goal_id const &goal) {
  post_event(boost::bind(&Logger::goal_event, this, "request", goal, true));
}

void TeleoReactor::Logger::recall(goal_id const &goal) {
  post_event(boost::bind(&Logger::goal_event, this, "recall", goal, false));
}

void TeleoReactor::Logger::notifyPlan(goal_id const &tok) {
  post_event(boost::bind(&Logger::goal_event, this, "token", tok, true));
}

void TeleoReactor::Logger::cancelPlan(goal_id const &tok) {
  post_event(boost::bind(&Logger::goal_event, this, "cancel", tok, false));
}


// asio methods

void TeleoReactor::Logger::obs(Observation o) {
  o.to_xml(m_file)<<std::endl;
}


void TeleoReactor::Logger::goal_event(std::string tag, goal_id g, bool full) {
  m_file<<"   <"<<tag<<" id=\""<<g<<"\" ";
  if( full )
    g->to_xml(m_file<<">\n")<<"\n   </"<<tag<<">"<<std::endl;
  else
    direct_write("/>", true);
}

void TeleoReactor::Logger::post_event(boost::function<void ()> fn) {
  m_strand.post(boost::bind(&Logger::phase_event, this, fn));
}

void TeleoReactor::Logger::phase_event(boost::function<void ()> fn) {
  open_phase();
  fn();
}

void TeleoReactor::Logger::set_tick(TICK val, TeleoReactor::Logger::tick_phase p) {
  close_tick();
  m_current = val;
  m_phase = p;
  m_flags.set(tick);
  m_flags.set(in_phase);
}

void TeleoReactor::Logger::set_phase(TeleoReactor::Logger::tick_phase p) {
  close_phase();
  m_phase = p;
  m_flags.set(in_phase);
}

void TeleoReactor::Logger::open_phase() {
  if( m_flags.test(in_phase) && !m_flags.test(has_data) ) {
    open_tick();
    switch( m_phase ) {
      case in_init:
        direct_write("  <init>", true);
        break;
      case in_new_tick:
        direct_write("  <start>", true);
        break;
      case in_synchronize:
        direct_write("  <synchronize>", true);
        break;
      case in_work:
        direct_write("  <has_work>", true);
        break;
      case in_step:
        direct_write("  <step>", true);
        break;
      default:
        direct_write("  <unknown>", true);
    }
    m_flags.set(has_data);
  }
}

void TeleoReactor::Logger::close_phase() {
  if( m_flags.test(in_phase) ) {
    if( m_flags.test(has_data) ) {
      switch( m_phase ) {
        case in_init:
          direct_write("  </init>", true);
          break;
        case in_new_tick:
          direct_write("  </start>", true);
          break;
        case in_synchronize:
          direct_write("  </synchronize>", true);
          break;
        case in_work:
          direct_write("  </has_work>", true);
          break;
        case in_step:
          direct_write("  </step>", true);
          break;
        default:
          direct_write("  </unknown>", true);
      }
      m_flags.reset(has_data);
    }
    m_flags.reset(in_phase);
  }
}

void TeleoReactor::Logger::open_tick() {
  if( m_flags.test(tick) && !m_flags.test(tick_opened) ) {
    m_file<<" <tick value=\""<<m_current<<"\">\n";
    m_flags.set(tick_opened);
  }
}

void TeleoReactor::Logger::close_tick() {
  if( m_flags.test(tick) ) {
    if( m_flags.test(tick_opened) ) {
      close_phase();
      direct_write(" </tick>", true);
    }
  } else if( m_flags.test(header) ) {
    direct_write(" </header>", true);
    m_flags.reset(header);
    m_flags.reset(in_phase);
  }
  m_flags.reset(tick);
  m_flags.reset(tick_opened);
}

void TeleoReactor::Logger::direct_write(std::string const &content, bool nl) {
  m_file.write(content.c_str(), content.length());
  if( nl )
    std::endl(m_file);
}
