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

#include "TeleoReactor.hh"

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
std::string DispatchError::buil_msg(goal_id const &g, std::string const &msg) throw() {
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

bool details::external::cmp_goals(IntegerDomain const &a, IntegerDomain const &b) {
  // sorting order
  //   - based on upperBound
  //   - if same upperBound : sorted base on lower bound
  //
  // this way I can safely update lower bounds without impacting tokens order
  return a.upperBound()<b.upperBound() ||
    ( a.upperBound()==b.upperBound() && a.lowerBound()<b.lowerBound() );
}


// structors

details::external::external() {
  m_pos = m_last; // may not be necessary but I just want  ot be sure ...
}

details::external::external(details::external const &other) 
  :m_pos(other.m_pos), m_last(other.m_last) {}

details::external::external(details::external_set::iterator const &pos,
			    details::external_set::iterator const &last, 
			    bool only_active)
  :m_pos(pos), m_last(last) {
  if( only_active )
    next_active();
}

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
    if( (*i)->startsAfter(current) ) {
      // Need to check for dispatching
      if( m_pos->first.accept_goals() ) {
	syslog()<<"Dispatching "<<(*i)->predicate()<<'['<<(*i)<<"] on \""
	 	<<m_pos->first.name()<<"\".";
	m_pos->first.request(*i);
	i = m_pos->second.erase(i);
      } else 
	++i;
    } else {
      syslog()<<"Goal "<<(*i)->predicate()<<'['<<(*i)<<"] is in the past !";
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
 
details::external &details::external::operator++() {
  if( valid() ) {
    ++m_pos;
    next_active();
  }
  return *this;
}

void details::external::next_active() {
  while( valid() && !m_pos->first.active() )
    ++m_pos;
}

// observers 

bool details::external::operator==(details::external const &other) const {
  if ( valid() )
    return m_pos == other.m_pos;
  else 
    return !other.valid();
}

TREX::utils::internals::LogEntry details::external::syslog() {
  return m_pos->first.client().syslog(m_pos->first.name().str());
}

/*
 * class TREX::transaction::TeleoReactor
 */

// structors


TeleoReactor::TeleoReactor(TeleoReactor::xml_arg_type &arg, bool loadTL)
  :m_inited(false), m_firstTick(true), m_graph(*(arg.second)),
   m_trLog(NULL),
   m_name(parse_attr<Symbol>(xml_factory::node(arg), "name")),
   m_latency(parse_attr<TICK>(xml_factory::node(arg), "latency")),
   m_maxDelay(0),
   m_lookahead(parse_attr<TICK>(xml_factory::node(arg), "lookahead")),
   m_nSteps(0) {
  rapidxml::xml_node<> &node(xml_factory::node(arg));
  
  if( parse_attr<bool>(true, node, "log") ) {
    std::string log = manager().file_name(getName().str()+".tr.log");
    m_trLog = new Logger(log);
    syslog()<<"Transactions logged to "<<log;
  }

  if( loadTL ) {
    Symbol tl_name;
    

    for(ext_iterator iter(node, "config"); iter.valid(); ++iter) {
      if( is_tag(*iter, "External") ) {
	tl_name = parse_attr<Symbol>(*iter, "name");
	if( tl_name.empty() )
	  throw XmlError(*iter, "Timelines cannot have an empty name");
	use(tl_name, parse_attr<bool>(true, *iter, "goals"));
      } else if( is_tag(*iter, "Internal") ) {
	tl_name = parse_attr<Symbol>(*iter, "name");
	if( tl_name.empty() )
	  throw XmlError(*iter, "Timelines cannot have an empty name");
	provide(tl_name);
      }
    }
  }
}
   
TeleoReactor::TeleoReactor(graph *owner, Symbol const &name, 
			   TICK latency, TICK lookahead, bool log)
  :m_inited(false), m_firstTick(true), m_graph(*owner), m_trLog(NULL),
   m_name(name),
   m_latency(latency), m_maxDelay(0), m_lookahead(lookahead),
   m_nSteps(0) {
  if( log ) {
    std::string log = manager().file_name(getName().str()+".tr.log");
    m_trLog = new Logger(log);
    syslog()<<"Transactions logged to "<<log;
  }
}

TeleoReactor::~TeleoReactor() {
  isolate();
  if( NULL!=m_trLog )
    delete m_trLog;
}

// observers 

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

double TeleoReactor::workRatio() {
  if( hasWork() ) {
    double ret = m_deadline;
    ret -= getCurrentTick();
    if( ret<=0.0 ) 
      ret = m_nSteps+1;
    else {
      ret += 1.0;
      ret *= m_nSteps+1;
    }
    return 1.0/ret;
  } else {
    m_nSteps = 0;
    m_deadline = getCurrentTick()+1+getLatency();
    return NAN;
  }
}

void TeleoReactor::postObservation(Observation const &obs) {
  internal_set::iterator i = m_internals.find(obs.object());
  
  if( m_internals.end()==i )
    throw SynchronizationError(*this, "attempted to post observation on "+
			       obs.object().str()+" which is not Internal.");
  
  (*i)->postObservation(getCurrentTick(), obs);
  m_updates.insert(*i);
  // syslog("ASSERT")<<obs;
}

bool TeleoReactor::postGoal(goal_id const &g) {
  if( !g ) 
    throw DispatchError(*this, g, "Invalid goal Id");

  details::external tl(m_externals.find(g->object()), m_externals.end(), false);
  
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

bool TeleoReactor::initialize(TICK final) {
  if( m_inited ) {
    syslog("ERROR")<< "Attempted to initalize this reactor twice.";
    return false;
  }
  m_initialTick = getCurrentTick(); 
  m_finalTick   = final;
  syslog()<<"Creation tick is "<<getInitialTick();
  syslog()<<"Execution latency is "<<getExecLatency();
  try {
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

void TeleoReactor::newTick() {
  if( m_firstTick ) {
    if( getCurrentTick()!=m_initialTick ) {
      syslog("WARN")<<"Updating initial tick from "<<m_initialTick
		    <<" to "<<getCurrentTick();
      m_initialTick = getCurrentTick();
    }
    m_firstTick = false;
  }
  if( NULL!=m_trLog )
    m_trLog->newTick(getCurrentTick());

  handleTickStart(); // allow derived class processing

  // Dispatched goals management
  details::external i = ext_begin();
  details::goal_queue dispatched; // store the goals that got dispatched on this tick ...
				  // I do nothing with it for now  
  // Manage goal dispatching
  for( ; i.valid(); ++i )
    i.dispatch(getCurrentTick(), dispatched);  
}

void TeleoReactor::doNotify() {
  for(external_set::iterator i = m_externals.begin();
      m_externals.end()!=i; ++i) 
    if( i->first.lastObsDate()==getCurrentTick() )
      notify( i->first.lastObservation() );
}
  

bool TeleoReactor::doSynchronize() {
  try {
    // Update the timelines here !
    if( synchronize() ) {
      for(internal_set::const_iterator i=m_updates.begin();
	  m_updates.end()!=i; ++i) {
	syslog("ASSERT")<<(*i)->lastObservation();
	if( NULL!=m_trLog )
	  m_trLog->observation((*i)->lastObservation());
      }
      m_updates.clear();
      return true;
    }
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
  resume();
  m_nSteps += 1;
}

void TeleoReactor::use(TREX::utils::Symbol const &timeline, bool control) {
  if( !m_graph.subscribe(this, timeline, control) )
    if( isInternal(timeline) )
      syslog("WARN")<<"External declaration of the Internal timeline \""
	      <<timeline.str()<<"\"";
    else
      syslog("WARN")<<"Multiple External declarations of timeline \""
	      <<timeline.str()<<"\"";
}

void TeleoReactor::provide(TREX::utils::Symbol const &timeline) {
  if( !m_graph.assign(this, timeline) )
    syslog("WARN")<<"Promoted \""<<timeline.str()<<"\" from External to Internal.";
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
  syslog()<<"Declared \""<<tl->name()<<"\".";  
  if( NULL!=m_trLog ) {
    m_trLog->provide(tl->name());
  }
}

void TeleoReactor::unassigned(details::timeline *tl) {
  internal_set::iterator i = m_internals.find(tl);
  m_internals.erase(i);
  syslog()<<"Undeclared \""<<tl->name()<<"\".";
  if( NULL!=m_trLog ) {
    m_trLog->unprovide(tl->name());
  }
}

void TeleoReactor::subscribed(Relation const &r) {
  external_set::value_type tmp;
  tmp.first = r;
  m_externals.insert(tmp);
  latency_updated(0, r.latency());
  syslog()<<"Subscribed to \""<<r.name()<<"\".";
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
    // I need to check what is the new maximum from there
    for(details::external i = ext_begin(); i.valid(); ++i)
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
  :m_file(file_name.c_str()), m_tick(false) {
  m_file<<"<Log>\n  <header>"<<std::endl;
}

TeleoReactor::Logger::~Logger() {
  if( m_tick ) 
    m_file<<"  </tick";
  else 
    m_file<<"  </header";
  m_file<<">\n</log>";
}

void TeleoReactor::Logger::provide(TREX::utils::Symbol const &name) {
  m_file<<"    <provide name=\""<<name<<"\"/>"<<std::endl;
}

void TeleoReactor::Logger::use(TREX::utils::Symbol const &name) {
  m_file<<"    <unprovide name=\""<<name<<"\"/>"<<std::endl;
}

void TeleoReactor::Logger::unprovide(TREX::utils::Symbol const &name) {
  m_file<<"    <use name=\""<<name<<"\"/>"<<std::endl;
}

void TeleoReactor::Logger::unuse(TREX::utils::Symbol const &name) {
  m_file<<"    <unuse name=\""<<name<<"\"/>"<<std::endl;
}

void TeleoReactor::Logger::newTick(TICK val) {
  if( m_tick ) 
    m_file<<"  </tick";
  else {
    m_file<<"  </header";
    m_tick = true;
  }
  m_file<<">\n  <tick value=\""<<val<<"\">"<<std::endl;
}

void TeleoReactor::Logger::observation(Observation const &obs) {
  obs.toXml(m_file, 4)<<std::endl;
}

void TeleoReactor::Logger::request(goal_id const &goal) {
  m_file<<"    <request id=\""<<goal<<"\">\n";
  goal->toXml(m_file, 6)<<"\n    </request>"<<std::endl;
}

void TeleoReactor::Logger::recall(goal_id const &goal) {
  m_file<<"    <recall id=\""<<goal<<"\"/>"<<std::endl;
}
