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
#include <trex/domain/FloatDomain.hh>
#include <trex/utils/chrono_helper.hh>

#include <boost/scope_exit.hpp>

using namespace TREX::transaction;
using namespace TREX::utils;

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

  for(; m_pos->second.end()!=i && cmp_goals((*i)->getStart(), dom); ++i);
  return i;
}

// modifiers

bool details::external::post_goal(goal_id const &g) {
  IntegerDomain const &g_start(g->getStart());
  // locate goal position
  details::goal_queue::iterator i = lower_bound(g_start);

  // check that it is not already posted
  for( ; m_pos->second.end()!=i && !cmp_goals(g_start, (*i)->getStart()); ++i)
    if( g==(*i) )
      return false;
  // insert the new goal
  m_pos->second.insert(i, g);
  return true;
}

void details::external::dispatch(TICK current, details::goal_queue &sent) {
  details::goal_queue::iterator i=m_pos->second.begin();
  IntegerDomain dispatch_w = m_pos->first.dispatch_window(current);

  for( ; m_pos->second.end()!=i && (*i)->startsBefore(dispatch_w.upperBound());  ) {
    if( (*i)->startsAfter(current) || (*i)->endsAfter(current+1) ) {
      // Need to check for dispatching
      if( m_pos->first.accept_goals() ) {
        if( m_pos->first.client().is_verbose() )
          syslog()<<"Dispatching "<<(*i)->predicate()<<'['<<(*i)<<"] on \""
                  <<m_pos->first.name()<<"\".";
	m_pos->first.request(*i);
	i = m_pos->second.erase(i);
      } else
	++i;
    } else {
      syslog()<<"Goal "<<(*i)->predicate()<<'['<<(*i)<<"] is in the past !\n\t"<<(**i);
      i = m_pos->second.erase(i);
    }
  }
}

void details::external::recall(goal_id const &g) {
  IntegerDomain const &g_start(g->getStart());
  // locate goal position
  details::goal_queue::iterator i = lower_bound(g_start);
  for( ; m_pos->second.end()!=i && !cmp_goals(g_start, (*i)->getStart()); ++i)
    if( g==(*i) ) {
      // was still pending => just remove it
      m_pos->second.erase(i);
      return;
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

TREX::utils::internals::LogEntry details::external::syslog() {
  return m_pos->first.client().syslog(m_pos->first.name().str());
}

Relation const &details::external::dereference() const {
  return *(m_pos->first);
}

/*
 * class TREX::transaction::TeleoReactor
 */

// structors

TeleoReactor::TeleoReactor(TeleoReactor::xml_arg_type &arg, bool loadTL,
			   bool log_default)
  :m_inited(false), m_firstTick(true), m_graph(*(arg.second)),
   m_verbose(parse_attr<bool>(arg.second->is_verbose(), xml_factory::node(arg), "verbose")), 
   m_trLog(NULL),
   m_name(parse_attr<Symbol>(xml_factory::node(arg), "name")),
   m_latency(parse_attr<TICK>(xml_factory::node(arg), "latency")),
   m_maxDelay(0),
   m_lookahead(parse_attr<TICK>(xml_factory::node(arg), "lookahead")),
   m_nSteps(0), m_past_deadline(false), m_validSteps(0) {
  boost::property_tree::ptree::value_type &node(xml_factory::node(arg));

  LogManager::path_type fname = file_name("stat.csv");
  m_stat_log.open(fname.c_str());
  m_stat_log<<"tick, synch_ns, delib_ns, n_steps\n";
     
  if( parse_attr<bool>(log_default, node, "log") ) {
    std::string base = getName().str()+".tr.log";
    fname = manager().file_name(base);
    m_trLog = new Logger(fname.string());
    LogManager::path_type cfg = manager().file_name("cfg"), 
      pwd = boost::filesystem::current_path(), 
      short_name(base), location("../"+base);
    boost::filesystem::current_path(cfg);
    try {
      create_symlink(location, short_name);
    } catch(...) {}
    boost::filesystem::current_path(pwd);
    syslog()<<"Transactions logged to "<<fname;
  }

  if( loadTL ) {
    Symbol tl_name;
    // Add external file content
    ext_xml(node.second, "config");

    for(boost::property_tree::ptree::iterator i=node.second.begin();
	node.second.end()!=i; ++i) {
      if( is_tag(*i, "External") ) {
	tl_name = parse_attr<Symbol>(*i, "name");
	if( tl_name.empty() )
	  throw XmlError(*i, "Timelines cannot have an empty name");
	use(tl_name, parse_attr<bool>(true, *i, "goals"),
            parse_attr<bool>(false, *i, "listen"));
      } else if( is_tag(*i, "Internal") ) {
	tl_name = parse_attr<Symbol>(*i, "name");
	if( tl_name.empty() )
	  throw XmlError(*i, "Timelines cannot have an empty name");
	provide(tl_name);
      }
    }
  }
}

TeleoReactor::TeleoReactor(graph *owner, Symbol const &name,
			   TICK latency, TICK lookahead, bool log)
  :m_inited(false), m_firstTick(true), m_graph(*owner), 
   m_verbose(owner->is_verbose()), m_trLog(NULL), m_name(name),
   m_latency(latency), m_maxDelay(0), m_lookahead(lookahead),
   m_nSteps(0) {
  LogManager::path_type fname = file_name("stat.csv");
  m_stat_log.open(fname.c_str());
     
  if( log ) {
    fname = manager().file_name(getName().str()+".tr.log");
    m_trLog = new Logger(fname.string());
    syslog()<<"Transactions logged to "<<fname;

  }
}

TeleoReactor::~TeleoReactor() {
  isolate(false);
  if( NULL!=m_trLog )
    delete m_trLog;
}

// observers

TeleoReactor::size_type TeleoReactor::count_internal_relations() const {
  size_type result(0);

  for(internal_set::const_iterator i=m_internals.begin(); m_internals.end()!=i; ++i)
    result += (*i)->size();
  return result;
}


bool TeleoReactor::isInternal(TREX::utils::Symbol const &timeline) const {
  return m_internals.end()!=m_internals.find(timeline);
}

bool TeleoReactor::isExternal(TREX::utils::Symbol const &timeline) const {
  return m_externals.end()!=m_externals.find(timeline);
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

void TeleoReactor::reset_deadline() {
  // initialize the deliberation parameters
  m_deadline = getCurrentTick()+1+getLatency();
  m_nSteps = 0;  
  m_past_deadline = false;
}



double TeleoReactor::workRatio() {
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
          syslog("WARN")<<" Reactor is now exceeding its deliberation latency ("
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
      details::goal_queue dispatched; // store the goals that got dispatched on this tick ...
                                      // I do nothing with it for now
      
      // Manage goal dispatching
      for( ; i.valid(); ++i )
	i.dispatch(getCurrentTick()+1, dispatched);
	
    }
  } catch(std::exception const &se) {
    syslog("WARN")<<"Exception during hasWork question: "<<se.what();
  } catch(...) {
    syslog("WARN")<<"Unknown Exception during hasWork question";
  }
  if( m_past_deadline ) {
    syslog("WARN")<<"Reactor needed to deliberate "<<(m_nSteps-m_validSteps)
                  <<" extra steps spread other "<<(getCurrentTick()-m_deadline)
                  <<" ticks after its latency."; 
  }
  reset_deadline();
  return NAN;
}

void TeleoReactor::postObservation(Observation const &obs) {
  internal_set::iterator i = m_internals.find(obs.object());

  if( m_internals.end()==i )
    throw SynchronizationError(*this, "attempted to post observation on "+
			       obs.object().str()+" which is not Internal.");

  (*i)->postObservation(getCurrentTick(), obs);
  m_updates.insert(*i);
}

bool TeleoReactor::postGoal(goal_id const &g) {
  if( !g )
    throw DispatchError(*this, g, "Invalid goal Id");

  details::external tl(m_externals.find(g->object()), m_externals.end());

  if( tl.valid() ) {
    bool ret = tl.post_goal(g);
    if( ret && NULL!=m_trLog )
      m_trLog->request(g);
    return ret;
  } else
    throw DispatchError(*this, g, "Goals can only be posted on External timelines");
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


bool TeleoReactor::postRecall(goal_id const &g) {
  if( !g )
    return false;
  details::external tl(m_externals.find(g->object()), m_externals.end());

  if( tl.valid() ) {
    tl.recall(g);
    if( NULL!=m_trLog )
      m_trLog->recall(g);
    return true;
  }
  return false;
}

bool TeleoReactor::postPlanToken(goal_id const &t) {
  if( !t )
    throw DispatchError(*this, t, "Invalid token id");
  
  // Look for the internal timeline
  internal_set::const_iterator tl = m_internals.find(t->object());
  if( m_internals.end()==tl )
    throw DispatchError(*this, t, "plan tokens can only be posted on Internal timelines.");
  else if( t->getEnd().upperBound() > getCurrentTick() ) {
    bool ret = (*tl)->notifyPlan(t);
    if( ret && NULL!=m_trLog )
      m_trLog->notifyPlan(t);
    return ret;
  }
  return false;
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
    internal_set::const_iterator tl = m_internals.find(g->object());
    if( m_internals.end()!=tl ) {
      // do something 
      if( (*tl)->cancelPlan(g) && NULL!=m_trLog ) 
        m_trLog->cancelPlan(g);
    }
  }
}


bool TeleoReactor::initialize(TICK final) {
  if( m_inited ) {
    syslog("ERROR")<< "Attempted to initalize this reactor twice.";
    return false;
  }
  m_initialTick = getCurrentTick();
  m_finalTick   = final;
  syslog()<<"Creation tick is "<<getInitialTick();
  syslog()<<"Execution latency is "<<getExecLatency();
  // syslog()<<"Clock used for stats is "<<boost::chrono::clock_string<stat_clock, char>::name();
  try {
    if( NULL!=m_trLog )
      m_trLog->init(m_initialTick);
    handleInit();   // allow derived class initialization
    m_firstTick = true;
    m_inited = true;
    return true;
  } catch(TREX::utils::Exception const &e) {
    syslog("ERROR")<<"Exception caught during init :\n"<<e;
  } catch( std::exception const &se) {
    syslog("ERROR")<<"C++ exception caught during init :\n"<<se.what();
  } catch(...) {
    syslog("ERROR")<<"Unknown exception caught during init";
  }
  return false;
}

bool TeleoReactor::newTick() {
  if( m_firstTick ) {
    if( getCurrentTick()!=m_initialTick ) {
      syslog("WARN")<<"Updating initial tick from "<<m_initialTick
		    <<" to "<<getCurrentTick();
      m_initialTick = getCurrentTick();
    }
    reset_deadline();
    m_firstTick = false;
  } else 
    m_stat_log<<(getCurrentTick()-1)<<", "
              <<m_synch_usage.count()
              <<", "<<m_deliberation_usage.count()
              <<", "<<m_tick_steps<<std::endl;
  m_tick_steps = 0;
  
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
    syslog("ERROR")<<"Exception caught during new tick:\n"<<e;
  } catch(std::exception const &se) {
    syslog("ERROR")<<"C++ exception caught during new tick:\n"<<se.what();    
  } catch(...) {
    syslog("ERROR")<<"Unknown exception caught during new tick";    
  }
  return false;
}

void TeleoReactor::doNotify() {
  for(external_set::iterator i = m_externals.begin();
      m_externals.end()!=i; ++i)
    if( i->first.lastObsDate()==getCurrentTick() )
      notify( i->first.lastObservation() );
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
        if( is_verbose() || NULL==m_trLog )
          syslog("ASSERT")<<(*i)->lastObservation();
        if( NULL!=m_trLog )
          m_trLog->observation((*i)->lastObservation());
      }
      m_updates.clear();
    }
    return success;
  } catch(Exception const &e) {
    syslog("SYNCH")<<"Exception caught: "<<e;
  } catch(std::exception const &se) {
    syslog("SYNCH")<<"C++ exception caught: "<<se.what();
  } catch(...) {
    syslog("SYNCH")<<"Unknown exception caught.";
  }
  syslog("ERROR")<<"Failed to synchronize.";
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

void TeleoReactor::use(TREX::utils::Symbol const &timeline, bool control, bool plan_listen) {
  details::transaction_flags flag; // initialize all the flags to 0
  flag.set(0,control);        // update the control flag
  flag.set(1,plan_listen);    // update the plan_listen flag 
  
  if( !m_graph.subscribe(this, timeline, flag) ) {
    if( isInternal(timeline) ) 
      syslog("WARN")<<"External declaration of the Internal timeline \""
	      <<timeline.str()<<"\"";
    else
      syslog("WARN")<<"Multiple External declarations of timeline \""
	      <<timeline.str()<<"\"";
  }
}


void TeleoReactor::provide(TREX::utils::Symbol const &timeline, bool controllable, bool publish) {
  details::transaction_flags flag;
  flag.set(0, controllable);
  flag.set(1, publish);
  if( !m_graph.assign(this, timeline, flag) )
    if( isInternal(timeline) ) {
      syslog("WARN")<<"Promoted \""<<timeline.str()<<"\" from External to Internal.";
    }
}

void TeleoReactor::tr_info(std::string const &msg) {
  if( NULL!=m_trLog ) {
    m_trLog->comment(msg);
  }
}


bool TeleoReactor::unuse(TREX::utils::Symbol const &timeline) {
  external_set::iterator i = m_externals.find(timeline);
  if( m_externals.end()!=i ) {
    Relation r = i->first;
    r.unsubscribe();
    return true;
  }
  return false;
}

bool TeleoReactor::unprovide(TREX::utils::Symbol const &timeline) {
  internal_set::iterator i = m_internals.find(timeline);
  if( m_internals.end()!=i ) {
    (*i)->unassign(getCurrentTick());
    return true;
  }
  return false;
}


void TeleoReactor::clear_internals() {
  while( !m_internals.empty() ) {
    m_internals.front()->unassign(getCurrentTick());
  }
}

void TeleoReactor::clear_externals() {
  while( !m_externals.empty() ) {
    Relation r = m_externals.begin()->first;
    r.unsubscribe();
  }
}

void TeleoReactor::assigned(details::timeline *tl) {
  m_internals.insert(tl);
  if( is_verbose() )
    syslog()<<"Declared \""<<tl->name()<<"\".";
  if( NULL!=m_trLog ) {
    m_trLog->provide(tl->name());
  }
  for(graph::listen_set::const_iterator i=m_graph.m_listeners.begin();
      m_graph.m_listeners.end()!=i; ++i)
    (*i)->declared(*tl);
}

void TeleoReactor::unassigned(details::timeline *tl) {
  internal_set::iterator i = m_internals.find(tl);
  m_internals.erase(i);
  if( is_verbose() )
    syslog()<<"Undeclared \""<<tl->name()<<"\".";
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
    syslog()<<"Subscribed to \""<<r.name()<<'\"'
            <<(r.accept_plan_tokens()?" with plan listening":"")<<'.';
  if( NULL!=m_trLog ) {
    m_trLog->use(r.name());
  }
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
    syslog()<<"Unsubscribed from \""<<r.name()<<"\".";
  if( NULL!=m_trLog ) {
    m_trLog->unuse(r.name());
  }
}


void TeleoReactor::latency_updated(TICK old_l, TICK new_l) {
  TICK prev = m_maxDelay;

  if( new_l>m_maxDelay )
    m_maxDelay = new_l;
  else if( old_l==m_maxDelay ) {
    // special case : the updated value is smaller and the old value is
    //                equal to my former exec delay
    //                this means that on of the timelines that were
    //                constraining my maxDelay has just reduced its latency
    m_maxDelay = new_l;
    for(details::active_external i(ext_begin(), ext_end()), endi(ext_end()); endi!=i; ++i) 
      m_maxDelay = std::max(m_maxDelay, i->latency());
  }
  if( m_maxDelay!=prev ) {
    // It may be anoying on the long run but for now I will log when this
    // exec latency changes
    syslog()<<" Execution latency updated from "<<prev<<" to "<<m_maxDelay;
    // Notify all the reactors that depend on me
    for(internal_set::iterator i=m_internals.begin(); m_internals.end()!=i; ++i)
      (*i)->latency_update(getLatency()+prev);
  }
}

/*
 * class TREX::transaction::TeleoReactor::Logger
 */

TeleoReactor::Logger::Logger(std::string const &file_name)
  :m_file(file_name.c_str()), m_header(true), m_tick(false), 
   m_tick_opened(false), m_in_phase(false), m_hasData(false) {
  m_file<<"<Log>\n  <header>"<<std::endl;
}

TeleoReactor::Logger::~Logger() {
  close_tick();
  m_file<<"</Log>\n";
  m_file.close();
}

void TeleoReactor::Logger::provide(TREX::utils::Symbol const &name) {
  open_phase();
  m_file<<"      <provide name=\""<<name<<"\"/>"<<std::endl;
}

void TeleoReactor::Logger::use(TREX::utils::Symbol const &name) {
  open_phase();
  m_file<<"      <use name=\""<<name<<"\"/>"<<std::endl;
}

void TeleoReactor::Logger::unprovide(TREX::utils::Symbol const &name) {
  open_phase();
  m_file<<"      <unprovide name=\""<<name<<"\"/>"<<std::endl;
}

void TeleoReactor::Logger::unuse(TREX::utils::Symbol const &name) {
  open_phase();
  m_file<<"      <unuse name=\""<<name<<"\"/>"<<std::endl;
}

void TeleoReactor::Logger::comment(std::string const &msg) {
  open_phase();
  m_file<<"    <!-- "<<msg<<" -->"<<std::endl;
}

void TeleoReactor::Logger::init(TICK val) {
  close_tick();
  m_tick = true;
  m_current = val;
  m_in_phase = true;
  m_phase = in_init;
}

void TeleoReactor::Logger::newTick(TICK val) {
  close_tick();
  m_tick = true;
  m_current = val;
  m_in_phase = true;
  m_phase = in_new_tick;
}

void TeleoReactor::Logger::synchronize() {
  close_phase();
  m_in_phase = true;
  m_phase = in_synchronize;
}

void TeleoReactor::Logger::failed() {
  open_phase();
  m_file<<"      <failed/>"<<std::endl;
}


void TeleoReactor::Logger::has_work() {
  close_phase();
  m_in_phase = true;
  m_phase = in_work;
}

void TeleoReactor::Logger::work(bool ret) {
  open_phase();
  m_file<<"      <work value=\""<<ret<<"\"/>"<<std::endl;
}


void TeleoReactor::Logger::step() {
  close_phase();
  m_in_phase = true;
  m_phase = in_step;
}

void TeleoReactor::Logger::open_tick() {
  if( m_tick ) {
    if( !m_tick_opened ) {
      m_file<<"  <tick value=\""<<m_current<<"\">\n";
      m_tick_opened = true;
    } 
  }
}

void TeleoReactor::Logger::open_phase() {
  if( m_in_phase && !m_hasData ) {
    open_tick(); 
  
    switch(m_phase) {
      case in_init:
	m_file<<"    <init>"<<std::endl;
	break;
      case in_new_tick:
	m_file<<"    <start>"<<std::endl;
	break;
      case in_synchronize:
	m_file<<"    <synchronize>"<<std::endl;
	break;
      case in_work:
	m_file<<"    <has_work>"<<std::endl;
	break;
      case in_step:
	m_file<<"    <step>"<<std::endl;
	break;
      default:
	m_file<<"    <unknown>"<<std::endl;      
    }
    m_hasData = true;
  }
}

void TeleoReactor::Logger::close_phase() {
  if( m_in_phase ) {
    if( m_hasData ) {
      switch(m_phase) {
      case in_init:
	m_file<<"    </init>"<<std::endl;
	break;
      case in_new_tick:
	m_file<<"    </start>"<<std::endl;
	break;
      case in_synchronize:
	m_file<<"    </synchronize>"<<std::endl;
	break;
      case in_work:
	m_file<<"    </has_work>"<<std::endl;
	break;
      case in_step:
	m_file<<"    </step>"<<std::endl;
	break;
      default:
	m_file<<"    </unknown>"<<std::endl;
      }
      m_hasData = false;
    } 
    m_in_phase = false;
  }
}

void TeleoReactor::Logger::close_tick() {
  if( m_tick ) {
    if( m_tick_opened ) {
      close_phase();
      m_file<<"  </tick>"<<std::endl;
    }
  } else if( m_header ) {
    m_file<<"  </header>"<<std::endl;
    m_header = false;
    m_in_phase = false;
  }
  m_tick = false;
  m_tick_opened = false;
}

void TeleoReactor::Logger::observation(Observation const &obs) {
  open_phase();
  obs.toXml(m_file, 6)<<std::endl;
}

void TeleoReactor::Logger::request(goal_id const &goal) {
  open_phase();
  m_file<<"      <request id=\""<<goal<<"\">\n";
  goal->toXml(m_file, 8)<<"\n      </request>"<<std::endl;
}

void TeleoReactor::Logger::recall(goal_id const &goal) {
  open_phase();
  m_file<<"      <recall id=\""<<goal<<"\"/>"<<std::endl;
}

void TeleoReactor::Logger::notifyPlan(goal_id const &t) {
  open_phase();
  m_file<<"      <token id=\""<<t<<"\">\n";
  t->toXml(m_file, 8)<<"\n      </token>"<<std::endl;  
}

void TeleoReactor::Logger::cancelPlan(goal_id const &t) {
  open_phase();
  m_file<<"      <cancel id=\""<<t<<"\"/>"<<std::endl;  
}

