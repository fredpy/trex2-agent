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

#include "reactor.hh"
#include <trex/domain/float_domain.hh>

#include <boost/scope_exit.hpp>

#include "private/node_impl.hh"
#include "private/graph_impl.hh"

#include <bitset>

using TREX::utils::symbol;
namespace utils=TREX::utils;

namespace TREX {
  namespace transaction {
    
    class reactor::logger {
    public:
      logger(std::string const &dest, boost::asio::io_service &io);
      ~logger();
      
      void provide(symbol const &name, bool goals, bool plan);
      void unprovide(symbol const &name);
      
      void use(symbol const &name , bool goals, bool plan);
      void unuse(symbol const &name);
      
      void init(TICK val);
      void new_tick(TICK val);
      void synchronize();
      
      void failed();
      void has_work();
      void work(bool ret);
      void step();
      
      void observation(token &obs);
      void request(token_id const &goal);
      void recall(token_id const &goal);
      
      void comment(std::string const &msg);
      void notify_plan(token_id const &tok);
      void cancel_plan(token_id const &tok);
      
    private:
      boost::asio::strand           m_strand;
      boost::asio::io_service::work m_active;
      utils::async_ofstream         m_file;
      
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
      
      void obs(token o);
      void goal_event(std::string tag, token_id g, bool full);
      
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
 * TREX::transaction::instance_scope_exec
 */

// statics
instance_scope_exec::exec_ref instance_scope_exec::init_exec(boost::asio::io_service &io) {
  return MAKE_SHARED<utils::priority_strand>(boost::ref(io));
}


/*
 * class TREX::transaction::ReactorException
 */
ReactorException::ReactorException(reactor const &r,
                                   std::string const &msg) throw()
  :GraphException(r.m_graph, r.name().str(), msg) {}


/*
 * class TREX::transaction::DispatchError
 */
// statics
std::string DispatchError::build_msg(token_id const &g, std::string const &msg) throw() {
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

bool TREX::transaction::details::external::cmp_goals(int_domain const &a,
                                                     int_domain const &b) {
  // sorting order
  //   - based on upperBound
  //   - if same upperBound : sorted based on lower bound
  //
  // this way I can safely update lower bounds without impacting tokens order
  return a.upper_bound()<b.upper_bound() ||
    ( a.upper_bound()==b.upper_bound() && a.lower_bound()<b.lower_bound() );
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

details::goal_queue::iterator details::external::lower_bound(int_domain const &dom) {
  details::goal_queue::iterator i = m_pos->second.begin();

  for(; m_pos->second.end()!=i && cmp_goals(i->first->start(), dom); ++i);
  return i;
}

// modifiers

bool details::external::post_goal(token_id const &g) {
  int_domain const &g_start(g->start());
  // locate goal position
  details::goal_queue::iterator i = lower_bound(g_start);

  // check that it is not already posted
  for( ; m_pos->second.end()!=i && !cmp_goals(g_start, i->first->start()); ++i)
    if( g==i->first )
      return false;
  // insert the new goal
  m_pos->second.insert(i, std::make_pair(g, true));
  if( m_pos->first.client().is_verbose() ) {
    syslog(info)<<m_pos->first.client().name()
      <<" added "<<g->predicate()<<'['<<g<<"] to the pending queue of "
      <<m_pos->first.name();
  }
  return true;
}

void details::external::dispatch(TICK current, details::goal_queue &sent) {
  details::goal_queue::iterator i=m_pos->second.begin();
  int_domain dispatch_w = m_pos->first.dispatch_window(current);

  for( ; m_pos->second.end()!=i && i->first->starts_before(dispatch_w.upper_bound());  ) {
    bool future = i->first->starts_after(current);

    if( future || i->first->ends_after(current+1) ) {
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

void details::external::recall(token_id const &g) {
  int_domain const &g_start(g->start());
  // locate goal position
  details::goal_queue::iterator i = lower_bound(g_start);
  for( ; m_pos->second.end()!=i && !cmp_goals(g_start, i->first->start());
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

TREX::utils::log::stream details::external::syslog(utils::symbol const &kind) {
  return m_pos->first.client().syslog(m_pos->first.name(), kind);
}

Relation const &details::external::dereference() const {
  return *(m_pos->first);
}

/*
 * class TREX::transaction::TeleoReactor
 */

utils::symbol const reactor::obs("ASSERT");
utils::symbol const reactor::plan("PLAN");

// structors

reactor::reactor(reactor::xml_arg_type &arg, bool loadTL, bool log_default)
:m_exec(arg.get<2>()), m_inited(false), m_firstTick(true),
m_graph(*reactor::arg_graph(arg)), m_have_goals(0),
m_verbose(utils::parse_attr<bool>(reactor::arg_graph(arg)->is_verbose(),
                                  reactor::xml(arg), "verbose")),
m_trLog(NULL),
m_latency(utils::parse_attr<TICK>(reactor::xml(arg), "latency")),
m_maxDelay(0),
m_lookahead(utils::parse_attr<TICK>(reactor::xml(arg), "lookahead")),
m_nSteps(0), m_past_deadline(false), m_validSteps(0),
m_stat_log(m_log->service()) {
  boost::property_tree::ptree::value_type &node(reactor::xml(arg));
  utils::symbol name_str = utils::parse_attr<utils::symbol>(node, "name");
     
  m_impl = get_graph().m_impl->add_node(name_str).lock();
  
  utils::log_manager::path_type fname = file_name("stat.csv");
  
  m_stat_log.open(fname.c_str());
  m_stat_log<<"tick, tick_ns, tick_rt_ns, synch_ns, synch_rt_ns, delib_ns, delib_rt_ns, n_steps\n";
  
  if( utils::parse_attr<bool>(log_default, node, "log") ) {
    std::string log_base = name().str()+".tr.log";
    
    fname = manager().log_file(log_base);
    m_trLog = new logger(fname.string(), manager().service());
    
    utils::log_manager::path_type cfg = manager().log_file("cfg"),
    pwd = boost::filesystem::current_path(),
    short_name(log_base), location("../"+log_base);
    
    // This change of directory is bad ...
    boost::filesystem::current_path(cfg);
    try {
      create_symlink(location, short_name);
    } catch(...) {}
    boost::filesystem::current_path(pwd);
    syslog(info)<<"Transactions logged to "<<fname;
  }
  
  if( loadTL ) {
    utils::symbol tl_name;
    // Add external file content
    utils::ext_xml(node.second, "config");
    for(boost::property_tree::ptree::iterator i=node.second.begin();
        node.second.end()!=i; ++i) {
      if( utils::is_tag(*i, "External") ) {
        tl_name = utils::parse_attr<symbol>(*i, "name");
        if( tl_name.empty() )
          boost::property_tree::ptree_bad_data("Timelines cannot have an empty name", *i);
        use(tl_name, utils::parse_attr<bool>(true, *i, "goals"),
            utils::parse_attr<bool>(false, *i, "listen"));
      } else if( utils::is_tag(*i, "Internal") ) {
        tl_name = utils::parse_attr<symbol>(*i, "name");
        if( tl_name.empty() )
          throw boost::property_tree::ptree_bad_data("Timelines cannot have an empty name", *i);
        provide(tl_name);
      }
    }
  }
}

reactor::reactor(graph *owner, exec_type exec, symbol const &name_str,
                 TICK lat, TICK lookahead, bool log)
  :m_exec(exec), m_inited(false), m_firstTick(true), m_graph(*owner),
   m_have_goals(0),
   m_verbose(owner->is_verbose()), m_trLog(NULL),
   m_latency(lat), m_maxDelay(0), m_lookahead(lookahead),
   m_nSteps(0), m_stat_log(m_log->service()) {
  m_impl = get_graph().m_impl->add_node(name_str).lock();

     
  utils::log_manager::path_type fname = file_name("stat.csv");
  m_stat_log.open(fname.string());
  
     
  if( log ) {
    fname = manager().log_file(name().str()+".tr.log");
    m_trLog = new logger(fname.string(), manager().service());
    syslog(info)<<"Transactions logged to "<<fname;

  }
}

reactor::~reactor() {
  isolate(false);
  if( !m_firstTick ) {
    m_stat_log<<", "<<m_deliberation_usage.count()
              <<", "<<m_delib_rt.count()
              <<", "<<m_tick_steps<<std::endl;
    m_stat_log.close();
  }

  if( NULL!=m_trLog ) {
    delete m_trLog;
  }
}

// observers

utils::symbol const &reactor::name() const {
  return m_impl->name();
}

utils::log_manager &reactor::manager() const {
  return m_impl->manager();
}

utils::log::stream reactor::syslog(utils::log::id_type const &ctx,
                                   utils::log::id_type const &kind) const {
  return m_impl->syslog(ctx, kind);
}


reactor::size_type reactor::count_internal_relations() const {
  size_type result(0);

  for(internal_set::const_iterator i=m_internals.begin(); m_internals.end()!=i; ++i)
    result += (*i)->size();
  return result;
}


bool reactor::internal_sync(TREX::utils::symbol name) const {
  return m_internals.end()!=m_internals.find(name);
}

bool reactor::is_internal(TREX::utils::symbol const &timeline) const {
  boost::function<bool ()> fn(boost::bind(&reactor::internal_sync, this, timeline));
  return utils::strand_run(m_graph.strand(), fn);
}

bool reactor::external_sync(TREX::utils::symbol name) const {
  return m_externals.end()!=m_externals.find(name);
}


bool reactor::is_external(TREX::utils::symbol const &timeline) const {
  boost::function<bool ()> fn(boost::bind(&reactor::external_sync, this, timeline));
  return utils::strand_run(m_graph.strand(), fn);
}

details::external reactor::ext_begin() {
  return details::external(m_externals.begin(), m_externals.end());
}

details::external reactor::ext_end() {
  return details::external(m_externals.end(), m_externals.end());
}

details::external reactor::find_external(TREX::utils::symbol const &name) {
  return details::external(m_externals.find(name), m_externals.end());
}

// modifers/callbacks

void reactor::reset_deadline() {
  // initialize the deliberation parameters
  m_deadline = current_tick()+1+latency();
  m_nSteps = 0;  
  m_past_deadline = false;
}

bool reactor::have_goals() {
  utils::shared_var<size_t>::scoped_lock lock(m_have_goals);
  return 0 < *m_have_goals;
}

void reactor::goal_flush(std::list<token_id> &a, std::list<token_id> &dest) {
  {
    utils::shared_var<size_t>::scoped_lock lock(m_have_goals);
    *m_have_goals -= a.size();
  }
  std::swap(a, dest);
}


void reactor::queue(std::list<token_id> &l, token_id g) {
  {
    utils::shared_var<size_t>::scoped_lock lock(m_have_goals);
    *m_have_goals += 1;
  }
  l.push_back(g);
}


void reactor::queue_goal(token_id g) {
  m_graph.strand().dispatch(boost::bind(&reactor::queue, this,
                                        boost::ref(m_sync_goals), g));
}

void reactor::queue_recall(token_id g) {
  m_graph.strand().dispatch(boost::bind(&reactor::queue, this,
                                        boost::ref(m_sync_recalls), g));
}

void reactor::queue_token(token_id g) {
  m_graph.strand().dispatch(boost::bind(&reactor::queue, this,
                                        boost::ref(m_sync_toks), g));
}

void reactor::queue_cancel(token_id g) {
  m_graph.strand().dispatch(boost::bind(&reactor::queue, this,
                                        boost::ref(m_sync_cancels), g));
}


double reactor::work_ratio() {
  std::list<token_id> tmp;

  if( have_goals() ) {
    // Start to flush goals
    boost::function<void ()> fn(boost::bind(&reactor::goal_flush, this,
                                            boost::ref(m_sync_goals),
                                            boost::ref(tmp)));
    utils::strand_run(m_graph.strand(), fn);
    
    while( !tmp.empty() ) {
      handle_request(tmp.front());
      tmp.pop_front();
    }
  }
  if( have_goals() ) {
    // Start to flush recalls
    boost::function<void ()> fn(boost::bind(&reactor::goal_flush, this,
                                            boost::ref(m_sync_recalls),
                                            boost::ref(tmp)));
    utils::strand_run(m_graph.strand(), fn);
    while( !tmp.empty() ) {
      handle_recall(tmp.front());
      tmp.pop_front();
    }
  }
  if( have_goals() ) {
    // Start to flush plan tokens
    boost::function<void ()> fn(boost::bind(&reactor::goal_flush, this,
                                            boost::ref(m_sync_toks),
                                            boost::ref(tmp)));
    utils::strand_run(m_graph.strand(), fn);
    while( !tmp.empty() ) {
      new_plan_token(tmp.front());
      tmp.pop_front();
    }
  }
  if( have_goals() ) {
    // Start to flush plan tokens
    boost::function<void ()> fn(boost::bind(&reactor::goal_flush, this,
                                            boost::ref(m_sync_cancels),
                                            boost::ref(tmp)));
    utils::strand_run(m_graph.strand(), fn);
    while( !tmp.empty() ) {
      cancel_plan_token(tmp.front());
      tmp.pop_front();
    }
  }
  
  bool ret = false;
  
  if( NULL!=m_trLog )
    m_trLog->has_work();
  try {
    ret = has_work();
    if( NULL!=m_trLog )
      m_trLog->work(ret);

    if( ret ) {
      double ret = m_deadline;
      ret -= current_tick();
      if( ret<0.0 && m_nSteps>0 ) {
        if( !m_past_deadline ) {
          m_past_deadline = true;
          m_validSteps = m_nSteps;
          syslog(warn)<<" Reactor is now exceeding its deliberation latency ("
          <<latency()<<")\n\tNumber of steps within its latency: "<<m_validSteps;
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
        i.dispatch(current_tick()+1, dispatched);
        
    }
  } catch(std::exception const &se) {
    syslog(warn)<<"Exception during hasWork question: "<<se.what();
  } catch(...) {
    syslog(warn)<<"Unknown Exception during hasWork question";
  }
  if( m_past_deadline ) {
    syslog(warn)<<"Reactor needed to deliberate "<<(m_nSteps-m_validSteps)
                      <<" extra steps spread other "
                      <<(current_tick()-m_deadline)
                      <<" ticks after its latency."; 
  }
  reset_deadline();
  return NAN;
}

void reactor::observation_sync(token_id o, bool verbose) {
  internal_set::iterator i = m_internals.find(o->object());
  
  if( m_internals.end()==i )
    throw boost::enable_current_exception(SynchronizationError(*this, "attempted to post observation on "+
                               o->object().str()+" which is not Internal."));
  
  (*i)->postObservation(o, verbose);
  m_updates.insert(*i);
}

void reactor::post_observation(token const &obs, bool verbose) {
  token_id tmp = MAKE_SHARED<token>(boost::ref(obs));
  tmp->pred_tag(token::obs_tag);
  
  boost::function<void ()> fn(boost::bind(&reactor::observation_sync,
                                          this, tmp, verbose));
  // m_graph.strand().dispatch(fn);
  utils::strand_run(m_graph.strand(), fn);
}

bool reactor::goal_sync(token_id g) {
  details::external tl(m_externals.find(g->object()), m_externals.end());
  if( tl.valid() ) {
    if( NULL!=m_trLog )
      m_trLog->request(g);
    return tl.post_goal(g);
  } else
    throw boost::enable_current_exception(DispatchError(*this, g, "Goals can only be posted on External timelines"));
}


bool reactor::post_goal(token_id const &g) {
  if( !g )
    throw DispatchError(*this, g, "Invalid goal Id");

  g->pred_tag(token::goal_tag);
  boost::function<bool ()> fn(boost::bind(&reactor::goal_sync,
                                          this, g));
  return utils::strand_run(m_graph.strand(), fn);
}

token_id reactor::post_goal(token const &g) {
  token_id tmp = MAKE_SHARED<token>(g);

  if( post_goal(tmp) )
    return tmp;
  else {
    // should never happen !?
    return token_id();
  }
}

token_id reactor::parse_goal(boost::property_tree::ptree::value_type const &g) {
  token_id ret = get_graph().parse_goal(g);
  ret->pred_tag(token::goal_tag);
  return ret;
}

bool reactor::recall_sync(token_id g) {
  details::external tl(m_externals.find(g->object()), m_externals.end());
  
  if( tl.valid() ) {
    if( NULL!=m_trLog )
      m_trLog->recall(g);
    tl.recall(g);
    return true;
  }
  return false;
}


bool reactor::post_recall(token_id const &g) {
  if( !g )
    return false;
  boost::function<bool ()> fn(boost::bind(&reactor::recall_sync,
                                          this, g));
  return utils::strand_run(m_graph.strand(), fn);
}

bool reactor::plan_sync(token_id t) {
  
  // Look for the internal timeline
  internal_set::const_iterator tl = m_internals.find(t->object());
  if( m_internals.end()==tl )
    throw boost::enable_current_exception(DispatchError(*this, t, "plan tokens can only be posted on Internal timelines."));
  else if( t->end().upper_bound() > current_tick() ) {
    if( NULL!=m_trLog )
      m_trLog->notify_plan(t);
    return (*tl)->notifyPlan(t);
  }
  return false;
}

void reactor::cancel_sync(token_id tok) {
  internal_set::const_iterator tl = m_internals.find(tok->object());
  if( m_internals.end()!=tl ) {
    // do something
    if( NULL!=m_trLog )
      m_trLog->cancel_plan(tok);
    
    (*tl)->cancelPlan(tok);
  } 
}

bool reactor::post_plan_token(token_id const &t) {
  if( !t )
    throw DispatchError(*this, t, "Invalid token id");
  
  boost::function<bool ()> fn(boost::bind(&reactor::plan_sync,
                                          this, t));
  return utils::strand_run(m_graph.strand(), fn);
}

token_id reactor::post_plan_token(token const &g) {
  token_id tmp = MAKE_SHARED<token>(g);
  
  if( post_plan_token(tmp) )
    return tmp;
  else 
    return token_id();
}

void reactor::cancel_plan_token(token_id const &g) {
  if( g ) {
    boost::function<void ()> fn(boost::bind(&reactor::cancel_sync,
                                            this, g));
    utils::strand_run(m_graph.strand(), fn);
  }
}


TICK reactor::final_tick() const {
  TICK g_final = m_graph.final_tick();
  
  if( !m_finalTick || g_final<*m_finalTick )
    return g_final;
  else
    return *m_finalTick;
}


void reactor::set_max_tick(TICK max) {
  if( !m_finalTick || max<*m_finalTick ) {
    syslog(warn)<<"Restricted reactor final tick to "<<max;
    m_finalTick = max;
  }
}


bool reactor::initialize(TICK final) {
  if( m_inited ) {
    syslog(error)<< "Attempted to initalize this reactor twice.";
    return false;
  }
  m_initialTick = m_obsTick = current_tick();
  
  if( !m_finalTick || final<*m_finalTick )
    m_finalTick   = final;
  syslog(info)<<"Creation tick is "<<initial_tick();
  syslog(info)<<"Execution latency is "<<exec_latency();
  // syslog()<<"Clock used for stats is "<<boost::chrono::clock_string<stat_clock, char>::name();
  try {
    if( NULL!=m_trLog )
      m_trLog->init(m_initialTick);
    handle_init();   // allow derived class initialization
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

bool reactor::new_tick() {
  if( m_firstTick ) {
    m_obsTick = current_tick();
    if( m_obsTick!=m_initialTick ) {
      syslog(warn)<<"Updating initial tick from "<<m_initialTick
                    <<" to "<<current_tick();
      m_initialTick = m_obsTick;
    }
    reset_deadline();
    TICK final = final_tick();
    
    if( final < m_graph.final_tick() )
      syslog(warn)<<"Reactor final tick is before agent's one:\n\t"
        <<date_str(final)<<" ("<<final<<").";
    
    m_firstTick = false;
  } else
    m_stat_log<<", "<<m_deliberation_usage.count()
              <<", "<<m_delib_rt.count()
              <<", "<<m_tick_steps<<std::endl;
  m_tick_steps = 0;
  
  if( current_tick()>final_tick() ) {
    syslog(warn)<<"This reactor reached its final tick.";
    return false;
  }
  
//  if( m_deliberation_usage > stat_duration::zero() )
//    syslog("stats")<<" delib="<<boost::chrono::duration_short<<m_deliberation_usage;
  m_deliberation_usage = stat_duration::zero();
  m_delib_rt = rt_clock::duration::zero();
  
  if( NULL!=m_trLog )
    m_trLog->new_tick(current_tick());

  try {
    {
      utils::chronograph<stat_clock> stat_chron(m_start_usage);
      utils::chronograph<rt_clock> rt_chron(m_start_rt);
    
      handle_tick_start(); // allow derived class processing
    }

    // Dispatched goals management
    details::external i = ext_begin();
    details::goal_queue dispatched; // store the goals that got dispatched on this tick ...
                                    // I do nothing with it for now
    
    // Manage goal dispatching
    for( ; i.valid(); ++i )
      i.dispatch(current_tick(), dispatched);
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

void reactor::collect_obs_sync(std::list<token_id> &l) {
  for(external_set::iterator i = m_externals.begin();
      m_externals.end()!=i; ++i) {
    // syslog(info)<<"Checking for new observation on "<<i->first.name();
    if( i->first.lastObsDate()==current_tick() ) {
      // syslog(info)<<"Collecting new obs: "<<i->first.lastObservation();
      l.push_back( i->first.lastObservation() );
    } //else
     // syslog(info)<<"Last observation date ("<<i->first.lastObsDate()<<") is before current tick";
  }
  
}


void reactor::do_notify() {
  std::list<token_id> obs;
  boost::function<void ()> fn(boost::bind(&reactor::collect_obs_sync,
                                          this, boost::ref(obs)));
  utils::strand_run(m_graph.strand(), fn);
  for(std::list<token_id>::const_iterator i=obs.begin(); obs.end()!=i; ++i) {
    // syslog("NOTIFY")<<(*i);
    notify(**i);
  }
}


bool reactor::do_synchronize() {
  if( NULL!=m_trLog )
    m_trLog->synchronize();
  bool stat_logged = false;
  
  try {
    TICK now = current_tick();
    
    bool success;
    {
      // collect information from external timelines 
      do_notify();
      {
        // measure timing only for synchronization call
        utils::chronograph<rt_clock> real_time(m_synch_rt);
        utils::chronograph<stat_clock> usage(m_synch_usage);
        success = synchronize();
      }
      m_stat_log<<now<<", "<<m_start_usage.count()
      <<", "<<m_start_rt.count()
      <<", "<<m_synch_usage.count()
      <<", "<<m_synch_rt.count();
      stat_logged = true;
    }
    if( success ) {
      for(internal_set::iterator i=m_updates.begin();
          m_updates.end()!=i; ++i) {
        bool echo;
        
        boost::function<void ()> fn(boost::bind(&details::timeline::synchronize,
                                                *i, now));
        utils::strand_run(m_graph.strand(), fn);

        token &observ = *((*i)->lastObservation(echo));
        
        // if( echo || is_verbose() || NULL==m_trLog )
        //   syslog(obs)<<observ;
        if( NULL!=m_trLog )
          m_trLog->observation(observ);
      }
      m_updates.clear();
    }
    m_obsTick = m_obsTick+1;
    
    if( !stat_logged ) {
      m_stat_log<<current_tick()<<", "<<m_synch_usage.count()
      <<", "<<m_synch_rt.count();
    }
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

void reactor::step() {
  if( NULL!=m_trLog )
    m_trLog->step();
  stat_clock::duration delta;
  rt_clock::duration delta_rt;
  
  {
    utils::chronograph<rt_clock> rt_chron(delta_rt);
    utils::chronograph<stat_clock> stat_chron(delta);
    resume();
  }
  m_deliberation_usage += delta;
  m_delib_rt += delta_rt;
  m_nSteps += 1;
  m_tick_steps +=1;
}

void reactor::use_sync(TREX::utils::symbol name, details::transaction_flags f) {
  if( !m_graph.subscribe(this, name, f) ) {
    if( internal_sync(name) )
      syslog(warn)<<"External declaration of the Internal timeline \""<<name<<"\"";
    else
      syslog(warn)<<"Multiple External declarations of timeline \""<<name<<"\"";
  }
}


void reactor::use(TREX::utils::symbol const &timeline, bool control, bool plan_listen) {
  details::transaction_flags flag; // initialize all the flags to 0
  flag.set(0,control);        // update the control flag
  flag.set(1,plan_listen);    // update the plan_listen flag
  
  boost::function<void ()> fn(boost::bind(&reactor::use_sync, this, timeline, flag));
  utils::strand_run(m_graph.strand(), fn);
}

void reactor::provide_sync(TREX::utils::symbol name, details::transaction_flags f) {
  if( !m_graph.assign(this, name, f) )
    if( internal_sync(name) ) {
      syslog(warn)<<"Promoted \""<<name<<"\" from External to Internal with rights "
      <<details::access_str(f.test(0), f.test(1));
    }
}


void reactor::provide(TREX::utils::symbol const &timeline,
                           bool controllable, bool publish) {
  details::transaction_flags flag;
  flag.set(0, controllable);
  flag.set(1, publish);
 
  boost::function<void ()> fn(boost::bind(&reactor::provide_sync, this, timeline, flag));
  utils::strand_run(m_graph.strand(), fn);
}

void reactor::tr_info(std::string const &msg) {
  if( NULL!=m_trLog ) {
    m_trLog->comment(msg);
  }
}

bool reactor::unuse_sync(TREX::utils::symbol name) {
  external_set::iterator i = m_externals.find(name);
  if( m_externals.end()!=i ) {
    Relation r = i->first;
    r.unsubscribe();
    return true;
  }
  return false; 
}


bool reactor::unuse(TREX::utils::symbol const &timeline) {
  boost::function<bool ()> fn(boost::bind(&reactor::unuse_sync, this, timeline));
  return utils::strand_run(m_graph.strand(), fn);
}

bool reactor::unprovide_sync(TREX::utils::symbol name) {
  internal_set::iterator i = m_internals.find(name);
  if( m_internals.end()!=i ) {
    (*i)->unassign(current_tick());
    return true;
  }
  return false;  
}

bool reactor::unprovide(TREX::utils::symbol const &timeline) {
  boost::function<bool ()> fn(boost::bind(&reactor::unprovide_sync, this, timeline));
  return utils::strand_run(m_graph.strand(), fn);
}


void reactor::isolate(bool failed) {
  if( NULL!=m_trLog && failed ) {
    logger *tmp = NULL;
    std::swap(tmp, m_trLog);
    tmp->failed();
    delete tmp;
  }
  clear_internals();
  clear_externals();
}


void reactor::clear_int_sync() {
  while( !m_internals.empty() ) {
    m_internals.front()->unassign(current_tick());
  }
}

void reactor::clear_ext_sync() {
  while( !m_externals.empty() ) {
    Relation r = m_externals.begin()->first;
    r.unsubscribe();
  }  
}


void reactor::clear_internals() {
  boost::function<void ()> fn(boost::bind(&reactor::clear_int_sync, this));
  utils::strand_run(m_graph.strand(), fn);
}

void reactor::clear_externals() {
  boost::function<void ()> fn(boost::bind(&reactor::clear_ext_sync, this));
  utils::strand_run(m_graph.strand(), fn);
}

void reactor::assigned(details::timeline *tl) {
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

void reactor::unassigned(details::timeline *tl) {
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

void reactor::subscribed(Relation const &r) {
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

void reactor::unsubscribed(Relation const &r) {
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


void reactor::latency_updated(TICK old_l, TICK new_l) {
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
      (*i)->latency_update(latency()+prev);
  }
}

void reactor::unblock(symbol const &name) {
  details::external e=find_external(name);  
  if( e.active() )
    e.unblock();
}


/*
 * class TREX::transaction::TeleoReactor::Logger
 */

// structors

reactor::logger::logger(std::string const &dest, boost::asio::io_service &io)
:m_strand(io), m_active(io), m_file(io, dest) {
  m_flags.set(header);
  m_strand.post(boost::bind(&logger::direct_write, this, "<Log>\n <header>",
                            true));
}

reactor::logger::~logger() {
  //std::cerr<<"Destroy logger "<<std::endl;
  //std::cerr<<" - schedulle : close potential tick tag"<<std::endl;
  
  m_strand.post(boost::bind(&logger::close_tick, this));
  //std::cerr<<" - build task : close the main tag"<<std::endl;
  boost::packaged_task<void> close_xml(boost::bind(&logger::direct_write,
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

void reactor::logger::comment(std::string const &msg) {
  m_strand.post(boost::bind(&logger::direct_write, this, "<!-- "+msg+" -->",
                            true));
}

void reactor::logger::init(TICK val) {
  m_strand.post(boost::bind(&logger::set_tick, this, val, in_init));
}

void reactor::logger::new_tick(TICK val) {
  m_strand.post(boost::bind(&logger::set_tick, this, val, in_new_tick));
}

void reactor::logger::synchronize() {
  m_strand.post(boost::bind(&logger::set_phase, this, in_synchronize));
}

void reactor::logger::failed() {
  post_event(boost::bind(&logger::direct_write,
                         this, "   <failed/>", true));
}

void reactor::logger::has_work() {
  m_strand.post(boost::bind(&logger::set_phase, this, in_work));
}

void reactor::logger::step() {
  m_strand.post(boost::bind(&logger::set_phase, this, in_step));
}

void reactor::logger::work(bool ret) {
  std::ostringstream oss;
  oss<<"   <work value=\""<<ret<<"\" />";
  post_event(boost::bind(&logger::direct_write,
                         this, oss.str(), true));
}

void reactor::logger::provide(symbol const &name, bool goals, bool plan) {
  std::ostringstream oss;
  oss<<"   <provide name=\""<<name
  <<"\" goals=\""<<goals
  <<"\" plan=\""<<plan<<"\" />";
  post_event(boost::bind(&logger::direct_write,
                         this, oss.str(), true));
}

void reactor::logger::unprovide(symbol const &name) {
  std::ostringstream oss;
  oss<<"   <unprovide name=\""<<name<<"\" />";
  post_event(boost::bind(&logger::direct_write,
                         this, oss.str(), true));  
}

void reactor::logger::use(symbol const &name, bool goals, bool plan) {
  std::ostringstream oss;
  oss<<"   <use name=\""<<name
  <<"\" goals=\""<<goals
  <<"\" plan=\""<<plan<<"\" />";
  post_event(boost::bind(&logger::direct_write,
                         this, oss.str(), true));
}

void reactor::logger::unuse(symbol const &name) {
  std::ostringstream oss;
  oss<<"   <unuse name=\""<<name<<"\" />";
  post_event(boost::bind(&logger::direct_write,
                         this, oss.str(), true));
}

void reactor::logger::observation(token &o) {
  o.pred_tag(token::obs_tag);
  post_event(boost::bind(&logger::obs, this, o));
}

void reactor::logger::request(token_id const &goal) {
  goal->pred_tag(token::goal_tag);
  post_event(boost::bind(&logger::goal_event, this, "request", goal, true));
}

void reactor::logger::recall(token_id const &goal) {
  goal->pred_tag(token::goal_tag);
  post_event(boost::bind(&logger::goal_event, this, "recall", goal, false));
}

void reactor::logger::notify_plan(token_id const &tok) {
  tok->pred_tag(token::goal_tag);
  post_event(boost::bind(&logger::goal_event, this, "token", tok, true));
}

void reactor::logger::cancel_plan(token_id const &tok) {
  tok->pred_tag(token::goal_tag);
  post_event(boost::bind(&logger::goal_event, this, "cancel", tok, false));
}


// asio methods

void reactor::logger::obs(token o) {
  utils::async_ofstream::entry e = m_file.new_entry();
  o.to_xml(e.stream())<<'\n';
}


void reactor::logger::goal_event(std::string tag, token_id g, bool full) {
  m_file<<"   <"<<tag<<" id=\""<<g<<"\" ";
  if( full )
    g->to_xml(m_file<<">\n")<<"\n   </"<<tag<<">\n";
  else
    direct_write("/>", true);
}

void reactor::logger::post_event(boost::function<void ()> fn) {
  m_strand.post(boost::bind(&logger::phase_event, this, fn));
}

void reactor::logger::phase_event(boost::function<void ()> fn) {
  open_phase();
  fn();
}

void reactor::logger::set_tick(TICK val, reactor::logger::tick_phase p) {
  close_tick();
  m_current = val;
  m_phase = p;
  m_flags.set(tick);
  m_flags.set(in_phase);
}

void reactor::logger::set_phase(reactor::logger::tick_phase p) {
  close_phase();
  m_phase = p;
  m_flags.set(in_phase);
}

void reactor::logger::open_phase() {
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

void reactor::logger::close_phase() {
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

void reactor::logger::open_tick() {
  if( m_flags.test(tick) && !m_flags.test(tick_opened) ) {
    m_file<<" <tick value=\""<<m_current<<"\">";
    m_flags.set(tick_opened);
  }
}

void reactor::logger::close_tick() {
  if( m_flags.test(tick) ) {
    if( m_flags.test(tick_opened) ) {
      close_phase();
      direct_write(" </tick>", true);
      // std::flush(m_file); // Flush the buffer at every tick
    }
  } else if( m_flags.test(header) ) {
    direct_write(" </header>", true);
    m_flags.reset(header);
    m_flags.reset(in_phase);
  }
  m_flags.reset(tick);
  m_flags.reset(tick_opened);
}

void reactor::logger::direct_write(std::string const &content, bool nl) {
  utils::async_ofstream::entry e = m_file.new_entry();
  
  e.stream().write(content.c_str(), content.length());
  if( nl )
    e.stream().put('\n');
}
