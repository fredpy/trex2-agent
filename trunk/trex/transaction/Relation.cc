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

utils::Symbol const timeline::s_failed("Failed");

// structors :

timeline::timeline(TICK date, utils::Symbol const &name)
  :m_name(name), m_owner(NULL), m_plan_listeners(0),
   m_lastObs(name, s_failed), m_obsDate(date) {}

timeline::timeline(TICK date, utils::Symbol const &name, TeleoReactor &serv, transaction_flags const &flags)
  :m_name(name), m_owner(&serv), m_transactions(flags), m_plan_listeners(0), 
   m_lastObs(name, s_failed), m_obsDate(date)  {}

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
    postObservation(date, Observation(name(), s_failed));
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
      r.syslog("WARN")<<"Updated transaction flags for external timeline "<<name();
      if( flags.test(1) ) {
        if( !pos->second.test(1) )
          m_plan_listeners += 1;
      } else if( pos->second.test(1) )
        m_plan_listeners -= 1;
      pos->second = flags; 
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

void timeline::postObservation(TICK date, Observation const &obs) {
  m_obsDate = date;
  m_lastObs = obs;
}

void timeline::request(goal_id const &g) {
  if( owned() ) {
    owner().syslog(name().str())<<"Request received ["<<g<<"] "
				<<*g;
    owner().handleRequest(g);
  }
}

void timeline::recall(goal_id const &g) {
  if( owned() )
    owner().handleRecall(g);
}

bool timeline::notifyPlan(goal_id const &t) {
  if( m_transactions.test(1) && owned() ) {
    owner().syslog("plan.INFO")<<"added ["<<t<<"] "<<*t;
    if( m_plan_listeners>0 ) {
      for(client_set::const_iterator i=m_clients.begin(); m_clients.end()!=i; ++i)
        if( i->second.test(1) )
          i->first->newPlanToken(t);
    }
    return true; 
  } 
  return false;
}

bool timeline::cancelPlan(goal_id const &t) {
  if( m_transactions.test(1) && owned() ) {
    owner().syslog("plan.CANCEL")<<"Removed ["<<t<<"] from "<<t->object()<<" to the "<<m_plan_listeners<<" plan clients.";
    if( m_plan_listeners>0 ) {
      for(client_set::const_iterator i=m_clients.begin(); m_clients.end()!=i; ++i)
        if( i->second.test(1) )
          i->first->cancelledPlanToken(t);
    }
  } 
  return true;
}


void timeline::latency_update(TICK prev) {
  if( look_ahead()>0 ) 
    for(client_set::const_iterator i=m_clients.begin(); m_clients.end()!=i; ++i) 
      if( i->second.test(0) )
	i->first->latency_updated(prev, latency());
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
