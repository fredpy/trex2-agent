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
  :m_name(name), m_owner(NULL), m_lastObs(name, s_failed),
   m_obsDate(date) {}

timeline::timeline(TICK date, utils::Symbol const &name, TeleoReactor &serv)
  :m_name(name), m_owner(&serv), m_lastObs(name, s_failed),
   m_obsDate(date) {}

timeline::~timeline() {
  // maybe some clean-up to do (?)
}

// observers :

TICK timeline::look_ahead() const {
  return owned()?owner().getLookAhead():0;
}

TICK timeline::latency() const {
  return owned()?owner().getExecLatency():0;
}

// modifiers :

bool timeline::assign(TeleoReactor &r) {
  bool ret = true;
  
  if( NULL==m_owner ) {
    client_set::iterator pos = m_clients.find(r.getName());
    
    if( m_clients.end()!=pos ) {
      unsubscribe(Relation(this, pos));
      ret = false;
    }
    m_owner = &r;
    r.assigned(this);
    latency_update(0);
  } else if( !owned_by(r) ) 
    throw MultipleInternals(r, name(), owner());
  return ret;
}

void timeline::unassign(TICK date, bool demotion, bool control) {
  if( owned() ) {
    TeleoReactor *tmp = m_owner;
    
    m_owner->unassigned(this);
    m_owner = NULL;
    postObservation(date, Observation(name(), s_failed));
    latency_update(tmp->getExecLatency());
    if( demotion ) 
      subscribe(*tmp, control);
  }
}

bool timeline::subscribe(TeleoReactor &r, bool control) {
  if( owned_by(r) )
    return false;
  else {
    std::pair<client_set::iterator, bool>
      ins = m_clients.insert(std::make_pair(&r, control));
    
    if( ins.second ) 
      r.subscribed(Relation(this, ins.first));
    else if( control )
      ins.first->second = true;
    return ins.second;
  }
}

void timeline::unsubscribe(Relation const &rel) {
  rel.client().unsubscribed(rel);
  m_clients.erase(rel.m_pos);
}

void timeline::postObservation(TICK date, Observation const &obs) {
  m_obsDate = date;
  m_lastObs = obs;
}

void timeline::request(goal_id const &g) {
  if( owned() ) {
    owner().syslog(name().str())<<"Request received "<<g->predicate()<<'['<<g<<"].";
    owner().handleRequest(g);
  }
}

void timeline::recall(goal_id const &g) {
  if( owned() )
    owner().handleRecall(g);
}

void timeline::latency_update(TICK prev) {
  if( look_ahead()>0 ) 
    for(client_set::const_iterator i=m_clients.begin(); m_clients.end()!=i; ++i) 
      if( i->second )
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
  return active() && m_pos->second && m_timeline->look_ahead()>0;
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
