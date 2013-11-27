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
#include "bits/timeline.hh"
#include "TeleoReactor.hh"

using namespace TREX;
using namespace TREX::transaction;
using namespace TREX::transaction::details;

/*
 * class TREX::transaction::MultipleInternals
 */

MultipleInternals::MultipleInternals(TeleoReactor const &faulty, utils::Symbol const &timeline,
				     TeleoReactor const &owner) throw() 
  :ReactorException(faulty, "timeline "+timeline.str()+" already Internal to "
		    +owner.getName().str()) {}

/*
 * class TREX::transaction::details::timeline
 */

utils::Symbol const timeline::s_failed(Predicate::failed_pred());

// structors :

timeline::timeline(TICK date, utils::Symbol const &name)
  :m_name(name), m_owner(NULL), m_plan_listeners(0),
   m_last_obs(Observation(name, Predicate::failed_pred())), m_obs_date(date), m_shouldPrint(false) {}

timeline::timeline(TICK date, utils::Symbol const &name, TeleoReactor &serv, transaction_flags const &flags)
  :m_name(name), m_owner(&serv), m_transactions(flags), m_plan_listeners(0), 
   m_last_obs(Observation(name, Predicate::failed_pred())), m_obs_date(date), m_shouldPrint(false)  {}

timeline::~timeline() {
  // maybe some clean-up to do (?)
}

// observers :

bool timeline::should_publish() const {
  return publish_plan() && (0 < m_plan_listeners);
}


TICK timeline::look_ahead() const {
  return (accept_goals())?owner().getLookAhead():0;
}

TICK timeline::latency() const {
  return accept_goals()?owner().getExecLatency():0;
}

std::string TREX::transaction::details::access_str(bool g, bool p) {
  std::ostringstream oss;
  oss<<(g?'g':'-')<<(p?'p':'-');
  return oss.str();
}


// modifiers :

bool timeline::assign(TeleoReactor &r, transaction_flags const &flags) {
  bool ret = true;
  
  if( NULL==m_owner ) {
    client_set::iterator pos = m_clients.find(r.getName());
    
    if( m_clients.end()!=pos ) {
      unsubscribe(Relation(this, pos));
      ret = false;
    }
    m_owner = &r;
    m_transactions = flags;
    r.assigned(this);
    latency_update(0);    
  } else if( owned_by(r) ) {
    TICK update = 0;
    if( !flags.test(0) )
      update = r.getExecLatency();
    r.syslog(name(), warn)<<"Updated transaction rights to "<<rights();
    m_transactions = flags;
    r.assigned(this);
    latency_update(update);
  } else {
    throw MultipleInternals(r, name(), owner());
  }
  return ret;
}

TeleoReactor *timeline::unassign(TICK date) {
  TeleoReactor *ret = NULL;
  if( owned() ) {
    ret = m_owner;
    
    m_owner->unassigned(this);
    m_owner = NULL;
    m_transactions.reset();
    postObservation(Observation(name(), Predicate::failed_pred()));
    synchronize(date);
    latency_update(ret->getExecLatency());
  }
  return ret;
}

void timeline::demote(TICK date, transaction_flags const &flags) {
  TeleoReactor *r = unassign(date);
  if( NULL!=r )
    subscribe(*r, flags);
}

bool timeline::subscribe(TeleoReactor &r, transaction_flags const &flags) {
  if( owned_by(r) )
    return false;
  else {
    bool inserted;
    client_set::iterator pos;
    
    boost::tie(pos, inserted) = m_clients.insert(std::make_pair(&r, flags));
    
    if( inserted ) {
      if( flags.test(1) )
        m_plan_listeners += 1;
      r.subscribed(Relation(this, pos));
    } else if( pos->second!=flags ) {
      std::string prev = rights();
      
      if( flags.test(1) ) {
        if( !pos->second.test(1) )
          m_plan_listeners += 1;
      } else if( pos->second.test(1) )
        m_plan_listeners -= 1;
      pos->second = flags;
      r.syslog(name(), warn)<<"Updated timeline transaction rights from "
        <<prev<<" to "<<rights();
    }
    return inserted;
  }
}

void timeline::unsubscribe(Relation const &rel) {
  if( rel.accept_plan_tokens() )
    m_plan_listeners -= 1;
  rel.client().unsubscribed(rel);
  m_clients.erase(rel.m_pos);
}

void timeline::postObservation(Observation const &obs,
			       bool verbose) {
  verbose = verbose || ( owned() && owner().is_verbose() );

#if 0
  if( m_next_obs && owned() )
    owner().syslog(warn)<<"New observation overwrite formerly posted one:\n\t"
			<<(*m_next_obs);
#endif
  m_next_obs = obs;
  m_shouldPrint = verbose;
}

void timeline::synchronize(TICK date) {
  if( m_next_obs ) {
    m_last_obs = m_next_obs;
    m_obs_date = date;
    m_next_obs.reset();
    if( owned() )
      owner().syslog(name(), TeleoReactor::obs)<<(*m_last_obs);
    else {
      static utils::SingletonUse<utils::LogManager> s_log;
      s_log->syslog(date, name(), utils::log::error)<<(*m_last_obs);
    }
  }
}

void timeline::request(goal_id const &g) {
  if( owned() ) {
    owner().syslog(info)<<"Request received ["<<g<<"] "
				<<*g;
    owner().queue_goal(g);
  }
}

void timeline::recall(goal_id const &g) {
  if( owned() ) {
    owner().syslog(info)<<"Recall received ["<<g<<"]";
    owner().queue_recall(g);
  }
}

bool timeline::notifyPlan(goal_id const &t) {
  if( m_transactions.test(1) && owned() ) {
    owner().syslog(TeleoReactor::plan)<<"Added ["<<t<<"] "<<*t;
    if( m_plan_listeners>0 ) {
      for(client_set::const_iterator i=m_clients.begin(); m_clients.end()!=i; ++i)
        if( i->second.test(1) )
          i->first->queue_token(t);
    }
    return true; 
  } 
  return false;
}

bool timeline::cancelPlan(goal_id const &t) {
  if( m_transactions.test(1) && owned() ) {
    owner().syslog(TeleoReactor::plan)<<"Removed ["<<t<<"] from "
				      <<t->object()<<" to the "
				      <<m_plan_listeners<<" plan clients.";
    if( m_plan_listeners>0 ) {
      for(client_set::const_iterator i=m_clients.begin(); m_clients.end()!=i; ++i)
        if( i->second.test(1) )
          i->first->queue_cancel(t);
    }
  } 
  return true;
}


void timeline::latency_update(TICK prev) {
  if( look_ahead()>0 ) 
    for(client_set::const_iterator i=m_clients.begin(); m_clients.end()!=i; 
	++i) {
      if( i->second.test(0) ) {
	i->first->latency_updated(prev, latency());
	i->first->unblock(name());
      }
    }
}

/*
 * class TREX::transaction::details::relation_iter
 */

relation_iter::relation_iter(timeline_set::const_iterator const &it, 
			     timeline_set::const_iterator const &last)
  :m_pos(it), m_last(last) {
  if( m_pos!=m_last ) {
    m_rel = (*m_pos)->begin();
    next_valid();
  }
}

void relation_iter::increment() {
  if( m_pos!=m_last ) {
    ++m_rel;
    next_valid();
  }
}


void relation_iter::next_valid() {
  while( (*m_pos)->end()==m_rel ) {
    ++m_pos;
    if( m_pos==m_last )
      return;
    m_rel = (*m_pos)->begin();
  }
}

bool relation_iter::equal(relation_iter const &other) const {
  return m_pos==other.m_pos && ( m_pos==m_last || m_rel==other.m_rel );
}

/*
 * class TREX::transaction::Relation
 */

void Relation::unsubscribe() const {
  if( valid() )
    m_timeline->unsubscribe(*this);
}

// Observers 
bool Relation::accept_goals() const {
  return active() && m_pos->second.test(0) && m_timeline->look_ahead()>0;
}

bool Relation::accept_plan_tokens() const {
  return m_pos->second.test(1);
}

std::string Relation::rights() const {
  return details::access_str(accept_goals(), accept_plan_tokens());
}



TICK Relation::latency() const {
  return valid()?m_timeline->latency():0;
}

TICK Relation::look_ahead() const {
  return accept_goals()?m_timeline->look_ahead():0;
}

bool Relation::valid() const {
  return NULL!=m_timeline && m_timeline->m_clients.end()!=m_pos;
}

bool Relation::active() const {
  return valid() && m_timeline->owned();
}

TICK Relation::lastObsDate() const {
  return m_timeline->lastObsDate();
}



Observation const &Relation::lastObservation() const {
  return m_timeline->lastObservation();
}

utils::Symbol const &Relation::name() const {
  return m_timeline->name();
}

TeleoReactor &Relation::server() const {
  return m_timeline->owner();
}


void Relation::recall(goal_id const &g) {
  return m_timeline->recall(g);
}

void Relation::request(goal_id const &g) {
  return m_timeline->request(g);
}
