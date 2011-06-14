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
#include "EuropaReactor.hh"
#include "DbSolver.hh"

#include <PLASMA/TokenVariable.hh>
#include <PLASMA/Timeline.hh>
#include <PLASMA/Debug.hh>

#include <boost/tuple/tuple.hpp>

using namespace TREX::europa;
using namespace TREX::transaction;

/*
 * class TREX::europa::DbCore
 */
// statics 

std::pair<DbCore::seq_iter, DbCore::seq_iter> 
DbCore::find_after(EUROPA::TimelineId const &tl, TICK now) {
  sequence_type const &seq = tl->getTokenSequence();
  seq_iter i = seq.begin(), last = seq.end();

  for(; last!=i && (*i)->end()->lastDomain().getUpperBound()<=now; ++i);
  return  std::make_pair(i, last);
}


// structors 

DbCore::DbCore(EuropaReactor &owner)
  :EUROPA::PlanDatabaseListener(owner.assembly().plan_db()),
   m_reactor(owner) {}

DbCore::~DbCore() {}

// 

std::string DbCore::dbg(std::string const &base) const {
  return base+":_"+m_reactor.getName().str();
}

// modifiers 

void DbCore::initialize(EUROPA::ConstrainedVariableId const &clk) {
  TICK initial = m_reactor.getInitialTick();
  m_clk = clk;

  EUROPA::TokenSet const &all = m_reactor.assembly().plan_db()->getTokens();  
  EUROPA::IntervalIntDomain mission_scope(initial, m_reactor.getFinalTick()),
    end_scope(initial+1, PLUS_INFINITY);

  for(EUROPA::TokenSet::const_iterator i=all.begin(); all.end()!=i; ++i) {
    if( (*i)->isFact() ) {
      // First check that the fact applies to a single object
      if( !(*i)->getObject()->baseDomain().isSingleton() ) 
	throw TokenError(**i, "Facts should be associated to a single object"); 

      // extract the object
      EUROPA::ObjectDomain const &domain=(*i)->getObject()->baseDomain();
      EUROPA::ObjectId object = domain.getObject(domain.getSingletonValue());

      // check that he object is not en external timeline
      if( m_reactor.assembly().isExternal(object) ) 
	throw TokenError(**i, "Facts cannot be applied to external timelines");

      // OK now should I ignore this fact 
      if( m_reactor.assembly().isIgnored(object) ) {
	m_reactor.log("WARN")<<"Ignoring the fact "<<object->toString()
			     <<'.'<<(*i)->getUnqualifiedPredicateName().toString()
			     <<" as its object is on the Ignore list.";
	m_reactor.assembly().ignore(*i);
      } else {
	if( (*i)->start()->baseDomain().getLowerBound() > mission_scope.getUpperBound() 
	    || (*i)->end()->baseDomain().getUpperBound() < end_scope.getLowerBound() ) {
	  m_reactor.log("WARN")<<"Ignoring the fact "<<object->toString()
			       <<'.'<<(*i)->getUnqualifiedPredicateName().toString()
			       <<" as it is outside of mission scope";
	  m_reactor.assembly().ignore(*i);
	} else {
	  EUROPA::IntervalIntDomain start_bounds(mission_scope), 
	    end_bounds(end_scope);
	  start_bounds.intersect((*i)->start()->baseDomain());
	  end_bounds.intersect((*i)->end()->baseDomain());
	  if( !start_bounds.isEmpty() )
	    (*i)->start()->restrictBaseDomain(start_bounds);
	  if( !end_bounds.isEmpty() )
	    (*i)->end()->restrictBaseDomain(end_bounds);
	  (*i)->activate();
	  object->constrain(*i, *i);
	}
      }
    }
  }
}

void DbCore::set_internal(EUROPA::ObjectId const &obj) {
  m_internals.insert(std::make_pair(obj, EUROPA::TokenId::noId()));
}

void DbCore::set_external(EUROPA::ObjectId const &obj) {
  m_externals.insert(std::make_pair(obj, EUROPA::TokenId::noId()));
}

void DbCore::notify(state_map::value_type const &obs) {
  std::pair<state_map::iterator, bool> ret = m_new_obs.insert(obs);
  if( !ret.second ) {
    if( ret.first->second.isId() ) {
      m_reactor.log("WARN")<<"Multiple observations received on "
			   <<obs.first->toString()
			   <<"\n discard "<<ret.first->second->toString()
			   <<"\n replace by "<<obs.second->toString();
      // Only the last observation count
      ret.first->second->discard();
    }
    ret.first->second = obs.second;
  }
  debugMsg(dbg("trex:token"), '['<<m_reactor.getCurrentTick()
	   <<"] OBSERVATION "<<obs.first->toString()
	   <<'.'<<obs.second->getUnqualifiedPredicateName().toString());
  m_observations.insert(obs.second);
}

bool DbCore::update_externals() {
  TICK cur = m_reactor.getCurrentTick();
  EUROPA::IntervalIntDomain now(cur, cur), future(cur+1, PLUS_INFINITY);
  state_map::iterator i = m_externals.begin();
  Assembly &assembly = m_reactor.assembly();

  // Integrate new observations and apply inertial value assumption
  for( ; m_externals.end()!=i; ++i) {
    state_map::iterator ni = m_new_obs.find(i->first);
    debugMsg(dbg("trex:sync"), " Update external "<<i->first->toString());
    if( m_new_obs.end()!=ni ) {
      EUROPA::TokenId obs = ni->second,
	previous = i->second;
      debugMsg(dbg("trex:sync"), "New observation : "
	       <<obs->toString());
      
      // terminate previous observation
      if( previous.isId() ) {
	previous->end()->restrictBaseDomain(now);	
	if( !assembly.propagate() ) {
	  debugMsg(dbg("trex:sync"), "Unable to terminate past observation "
		   <<obs->toString());
	  m_reactor.log("ERROR")<<"Failed to terminate "
				<<i->first->getName().toString()<<'.'
				<<i->second->getUnqualifiedPredicateName().toString();
	  return false;
	}
	m_terminated.insert(previous);
	debugMsg(dbg("trex:sync"), "Terminating external obs "
		 <<previous->toString());
	if( previous->isMerged() ) {
	  EUROPA::TokenId active = previous->getActiveToken();
	  previous->cancel();
	  previous->merge(active);
	  previous = active;
	} else if( !previous->isActive() )
	  previous = EUROPA::TokenId::noId();
      }
      
      // Find where the new observations belong
      seq_iter pos, last;
      boost::tie(pos, last) = find_after(i->first, cur);
      
      if( last!=pos && previous==*pos )
	++pos;

      if( last!=pos ) {
	// Now lets see if I can merge it with the successor of prev
	std::vector<EUROPA::TokenId> cands;
	assembly.plan_db()->getCompatibleTokens(obs, cands,
						UINT_MAX,
						true);
	if( cands.size()==1 ) {
	  debugMsg(dbg("trex:sync"), "Merge observation "
		   <<obs->toString()<<" with "<<cands[0]->toString());
	  obs->merge(cands[0]);
	  if( !assembly.propagate() ) {
	    debugMsg(dbg("trex:sync"), "Unable to merge new observation "
		     <<obs->toString()<<" with "<<cands[0]->toString());
	    m_reactor.log("ERROR")<<"Failed to merge new observation "
				  <<i->first->getName().toString()<<'.'
				  <<i->second->getUnqualifiedPredicateName().toString();
	    return false;
	  }
	} else {
	  for(std::vector<EUROPA::TokenId>::const_iterator 
		j=cands.begin(); cands.end()!=j; ++j) {
	    if( *j==*pos ) {
	      debugMsg(dbg("trex:sync"), " Merge observation "
		       <<obs->toString()<<" with "
		       <<(*pos)->toString());
	      obs->merge(*pos);
	      if( !assembly.propagate() ) {
		debugMsg(dbg("trex:sync"), "Unable to merge new observation "
			 <<obs->toString()<<" with "<<(*pos)->toString());
		m_reactor.log("ERROR")<<"Failed to merge new observation "
				      <<i->first->getName().toString()<<'.'
				      <<i->second->getUnqualifiedPredicateName().toString();
		return false;
	      }
	      break;
	    }
	  }
	}
      }      

      // if I could not merge it just insert it right after last obs
      if( !obs->isMerged() ) {
	obs->activate();
	bool inserted = false;
	if( previous.isId() ) {
	  debugMsg(dbg("trex:sync"), "Insert observation "
		   <<obs->toString()<<" after "<<previous->toString());
	  i->first->constrain(previous, obs);
	  inserted = true;
	} 
	if( last!=pos ) {
	  debugMsg(dbg("trex:sync"), "Insert observation "
		   <<obs->toString()<<" before "<<(*pos)->toString());
	  i->first->constrain(obs, *pos);
	  inserted = true;
	}
	if( !inserted ) {
	  debugMsg(dbg("trex:sync"), "Insert observation "
		   <<obs->toString()<<" in "<<i->first->toString());
	  i->first->constrain(obs, obs);
	}
	if( !assembly.propagate() ) {
	  debugMsg(dbg("trex:sync"), "Unable to insert new observation "
		   <<obs->toString());
	  m_reactor.log("ERROR")<<"Failed to insert new observation "
				<<i->first->getName().toString()<<'.'
				<<i->second->getUnqualifiedPredicateName().toString();
	  return false;
	}
      }
      i->second = obs;
      // succeeded on integrating the new observation
      m_new_obs.erase(ni); 
    } else {
      debugMsg(dbg("trex:sync"), " extend duration of "
	       <<i->first->toString());
      i->second->end()->restrictBaseDomain(future);
      if( i->second->isMerged() ) {
	  EUROPA::TokenId active = i->second->getActiveToken();
	  i->second->cancel();
	  i->second->merge(active);
      }
	
      
      if( !assembly.propagate() ) {
	debugMsg(dbg("trex:sync"), "Unable to apply inertial value "
		 "assumption to "<<i->second->toString());
	m_reactor.log("ERROR")<<"Failed to extend dutration of "
			      <<i->first->getName().toString()<<'.'
			      <<i->second->getUnqualifiedPredicateName().toString();
	return false;
      }
    }
  }
  
  for(i=m_new_obs.begin(); m_new_obs.end()!=i; ++i) {
    if( i->second.isId() ) {
      m_reactor.log("WARN")<<i->first->toString()<<" is not external :"
			   <<"\n\tdiscarding observation "
			   <<i->second->toString();
      i->second->discard();
    }
  }
  m_new_obs.clear();
  return propagate();
}


bool DbCore::synchronize() {
  size_t steps = 0;
  TICK cur = m_reactor.getCurrentTick();

  // Update the clock for the planner
  if( m_clk.isId() ) {
    m_clk->restrictBaseDomain(EUROPA::IntervalIntDomain(cur, PLUS_INFINITY));
  }

  if( resolve(steps) ) {
    if( update_internals(steps) ) {
      if( resolve(steps) ) {
	m_reactor.logPlan("sync");
	debugMsg(dbg("trex:sync"), "SUCESS after "<<steps<<" steps.");
	return true;
      } else 
	debugMsg(dbg("trex:sync"), "FAILED to resolve internal timelines updates");
    } else
      debugMsg(dbg("trex:sync"), "FAILED to identify internal timelines state");
  } else
    debugMsg(dbg("trex:sync"), "FAILED to resolve current state");
  m_reactor.logPlan("failed");
  return false;
}

bool DbCore::process_agenda(EUROPA::TokenSet agenda, size_t &steps) {
  TICK cur = m_reactor.getCurrentTick();
  Assembly &assembly = m_reactor.assembly();

  for(EUROPA::TokenSet::const_iterator i=agenda.begin(); agenda.end()!=i; ++i) {
    if( (*i)->getObject()->lastDomain().isSingleton() && (*i)->isInactive() ) {
      EUROPA::TokenId cand;
      
      if( assembly.in_synch_scope(*i, cand) ) {
	debugMsg(dbg("trex:agenda"), "resolving "<<(*i)->toString());
	++steps;
	if( !( assembly.resolve(*i, cand, steps) && propagate() ) ) {
	  debugMsg(dbg("trex:agenda"), "FAILED to resolve "<<(*i)->toString());
	  m_reactor.logPlan("failure");
	  return false;
	}
      }
    }
  }
  return true;
}

bool DbCore::resolve(size_t &steps) {
  size_t last_count;
  // Loop until quiescence
  do {
    last_count = steps;
    
    if( !propagate() )
      return false;

    // first process the facts
    condDebugMsg(!m_facts_agenda.empty(), dbg("trex:sync"), 
		 m_facts_agenda.size()<<" facts in the agenda.");
    if( !process_agenda(m_facts_agenda, steps) ) {
      debugMsg(dbg("trex:sync"), "Failed to resolve facts");
      return false;
    }
    // then process everything else
    condDebugMsg(!m_agenda.empty(), dbg("trex:sync"), 
		 m_agenda.size()<<" tokens in the agenda.");
    if( !process_agenda(m_agenda, steps) ) {
      debugMsg(dbg("trex:sync"), "Failed to resolve tokens");
      return false;
    }
  } while( last_count!=steps );
  return true;
}

bool DbCore::update_internals(size_t &steps) {
  Assembly &assembly = m_reactor.assembly();
  TICK cur = m_reactor.getCurrentTick();
  EUROPA::IntervalIntDomain future(cur, PLUS_INFINITY);
  std::list<EUROPA::TokenId> new_tokens;
  
  if( assembly.invalid() )
    return false;
  for(state_map::const_iterator i=m_internals.begin(); m_internals.end()!=i; ++i) {
    EUROPA::TokenId active = i->second, last_obs;
    seq_iter p_cur, last;

    if( active.isId() ) {
      active->end()->restrictBaseDomain(future);
      if( active->isMerged() )
	active = active->getActiveToken();
    }

    boost::tie(p_cur, last) = find_after(i->first, cur);
    if( p_cur!=last && (*p_cur)->start()->lastDomain().getUpperBound()<=cur ) 
      last_obs = *p_cur;
    else {
      // did not find an observation for current tick
      debugMsg(dbg("trex:sync"), '['<<cur<<"] setting "<<i->first->toString()
	       <<" state to its default.");
      if( !assembly.insert_default(i->first, last_obs, steps) ) {
	debugMsg(dbg("trex:sync"), "FAILED to insert default value on "
		 <<i->first->toString());
	return false;
      }
    } 
   
    if( last_obs->start()->lastDomain().getUpperBound()>=cur ) {
      last_obs->start()->specify(cur);
      if( last_obs!=active && active.isId() )
	active->end()->specify(cur);    
    }
  }
  if( !propagate() )
    return false;
  return true;
}

bool DbCore::remove_goal(EUROPA::TokenId const &tok, bool restrict) {
  if( m_goals.erase(tok) ) {
    if( restrict )
      tok->restrictBaseDomains();
    m_reactor.removed(tok);
    return true;
  }
  return false;
}


bool DbCore::reset_goal(EUROPA::TokenId const &tok, bool aggressive) {
  TICK cur = m_reactor.getCurrentTick();
  EUROPA::IntervalIntDomain future(cur, PLUS_INFINITY);
  EUROPA::TokenId active = (tok->isMerged()?tok->getActiveToken():tok);
  
  if( active->end()->baseDomain().getUpperBound()<cur ) {
    debugMsg(dbg("trex:relax"), "DISCARD completed goal "<<tok->toString()
	     <<": "<<tok->getPredicateName().toString());
    return true;
  }
  if( tok->start()->baseDomain().getUpperBound()<=cur ) {
    if( aggressive ) {
      debugMsg(dbg("trex:relax"), "DISCARD goal started  in the past "<<tok->toString()
	       <<": "<<tok->getPredicateName().toString());
      return true;
    }
  }
  
  return false;
}

bool DbCore::relax(bool aggressive) {
  Assembly &assembly = m_reactor.assembly();  

  m_reactor.log("core")<<"Beginning database relax"
     		       <<(aggressive?" and forget about the past":"");
  debugMsg(dbg("trex:relax"), "START "<<(aggressive?"aggressive ":"")<<"relax");
  // recall all of my former objectives
  m_reactor.relax();
  // Put the solver out of the way 
  assembly.solver().clear();
  assembly.mark_inactive();

  TICK cur = m_reactor.getCurrentTick();
  
  // First pass : deal with facts produced by TREX

  EUROPA::TokenSet relax_list = m_observations;
  for(EUROPA::TokenSet::const_iterator i=relax_list.begin(); 
      relax_list.end()!=i; ++i) {
    if( aggressive && (*i)->end()->baseDomain().getUpperBound()<=cur ) {
  	debugMsg(dbg("trex:relax"), "DISCARD past fact "<<(*i)->toString()
  		 <<(*i)->getPredicateName().toString());
	(*i)->discard();	
    } else 
      (*i)->cancel();
  }

  // Second pass : deal with the goals 
  relax_list = m_goals;
  for(EUROPA::TokenSet::const_iterator i=relax_list.begin(); 
      relax_list.end()!=i; ++i) 
    if( reset_goal(*i, aggressive) ) 
      (*i)->discard();
    else {
      // Make sure that the domain is fully relaxed 
      std::vector<EUROPA::ConstrainedVariableId> const &vars = (*i)->getVariables();
      if( !(*i)->isInactive() )
	(*i)->cancel();

      for(std::vector<EUROPA::ConstrainedVariableId>::const_iterator v=vars.begin();
	  vars.end()!=v; ++v) {
	if( (*v)->canBeSpecified() && (*v)->isSpecified() )
	  (*v)->reset();
	(*v)->relax();
      }
    }
  // third pass get rid of recalled goals
  relax_list = m_recalled;
  for(EUROPA::TokenSet::const_iterator i=relax_list.begin(); 
      relax_list.end()!=i; ++i) {
    debugMsg(dbg("trex:relax"), "DISCARD recalled goal "<<(*i)->toString());
    (*i)->discard();
  }
      
  debugMsg(dbg("trex:relax"), "SUCESS");
  m_reactor.logPlan();
  return true;
}

void DbCore::step() {
  Assembly &assembly = m_reactor.assembly();

  debugMsg(dbg("trex:step"), '['<<m_reactor.getCurrentTick()<<"] START deliberation step");
  
  if( propagate() ) {
    DbSolver &solver = assembly.solver();
    
    if( assembly.inactive() ) 
      assembly.mark_active();
    solver.step();
    process_pending();
    if( solver.noMoreFlaws() ) {      
      m_reactor.end_deliberation();
    }
  }
}

void DbCore::doDispatch() {
  EUROPA::DbClientId cli = m_reactor.assembly().plan_db()->getClient();

  // todo add a notion of guards to avoid to dispatch too early
  for( state_map::const_iterator i=m_externals.begin();
       m_externals.end()!=i; ++i) {
    TICK from, to;
    if( m_reactor.dispatch_window(i->first, from, to) ) {
      seq_iter it, last;
      boost::tie(it, last) = find_after(i->first, from);

      // skipp the tokens that are necessarily before from 
      for( ;last!=it && (*it)->start()->lastDomain().getUpperBound()<from; ++it);

      // now dispatch tokens that can start before to 
      // and are not part of deliberation
      for( ;last!=it && (*it)->start()->lastDomain().getLowerBound()<to; ++it)
	if( !m_reactor.assembly().in_deliberation(*it) ) {
	  // // to be safe just put it in a new token 
	  // EUROPA::TokenId 
	  //   requested = cli->createToken((*it)->getPredicateName().c_str(),
	  // 				 NULL, false, false);
	  // cli->merge(requested, (*it));
	  m_reactor.request(i->first, *it);
	  debugMsg(dbg("trex:token:state"), '['<<m_reactor.getCurrentTick()
		   <<"] REQUESTED "<<(*it)->toString()<<": "
		   <<i->first->toString()<<'.'
		   <<(*it)->getUnqualifiedPredicateName().toString()
		   <<"\n\tstart = "<<(*it)->start()->lastDomain().toString()
		   <<"\n\tduration = "<<(*it)->duration()->lastDomain().toString()
		   <<"\n\tend = "<<(*it)->end()->lastDomain().toString());
	}
    }
  }
}

void DbCore::doNotify() {
  TICK cur = m_reactor.getCurrentTick();
  EUROPA::DbClientId cli = m_reactor.assembly().plan_db()->getClient();
  EUROPA::IntervalIntDomain now(cur, cur), future(cur+1, PLUS_INFINITY);

  // Update my internal state
  for(state_map::iterator i=m_internals.begin(); m_internals.end()!=i; ++i) {
    seq_iter p, p_cur, last;
    
    boost::tie(p, last) = find_after(i->first, cur-1);
    
    p_cur = p;
    if( last!=p_cur && (*p_cur)->end()->lastDomain().getUpperBound()<=cur ) 
      ++p_cur;
    if( last==p_cur || (*p_cur)->start()->lastDomain().getLowerBound()>cur )
      throw EuropaException("Unexpected missing state for internal timeline "+
			    i->first->getName().toString());
    else if( (*p_cur)->start()->lastDomain().getUpperBound()>=cur ) {
      EUROPA::TokenId obs;
      
      if( (*p_cur)->isFact() ) 
	obs = *p_cur;
      else {
	// From here  I know that I have a new observation 
	obs = cli->createToken((*p_cur)->getPredicateName().c_str(),
			       NULL, false, true);
      }
      obs->start()->restrictBaseDomain(now);
      obs->end()->restrictBaseDomain(future);
      obs->getObject()->specify(i->first->getKey());
      if( *p_cur!=obs ) {
	cli->merge(obs, *p_cur);	
	m_reactor.assembly().propagate();
	// restrict the fact attributes to what we are going to produce
	obs->getObject()->restrictBaseDomain(obs->getObject()->getLastDomain());
	for(std::vector<EUROPA::ConstrainedVariableId>::const_iterator 
	      v=obs->parameters().begin(); obs->parameters().end()!=v; ++v) 
	  (*v)->restrictBaseDomain((*v)->lastDomain());	            
	m_reactor.assembly().propagate();
      }
      m_observations.insert(obs);
	
      // check if any requested goal was completed
      EUROPA::TokenSet cands = (*p)->getMergedTokens();
      if( (*p)!=i->second ) {
	if( i->second.isId() ) 
	  cands.erase(i->second);
	cands.insert(*p);
      }
      for(EUROPA::TokenSet::const_iterator it=cands.begin(); cands.end()!=it; ++it) {
	// We assume that if a goal token is completed then
	// the goal itself is completed
	// - note formally it is not necessarily true as this
	//   goal may depend on future tokens. To check that
	//   I need to se iff this token is the master of a token
	//   that is yet to be completed... this could be checked
	//   in the future but for now lets try to be efficient 
	//   instead
	remove_goal(*it);	
      }
      // finally update the current state
      if( i->second.isId() ) {
	i->second->end()->restrictBaseDomain(now);
	m_terminated.insert(i->second);
      }
      i->second = obs;
      m_reactor.notify(i->first, obs);
    }
    i->second->end()->restrictBaseDomain(future);
    if( i->second->master().isId() && i->second->master()->isFact() ) {
      i->second->master()->end()->restrictBaseDomain(future);
      i->second->master()->start()->restrictBaseDomain(i->second->start()->baseDomain());
    }
  }
}

void DbCore::recall(EUROPA::eint const &key) {
  EUROPA::EntityId entity = EUROPA::Entity::getEntity(key);
  
  if( entity.isId() && EUROPA::TokenId::convertable(entity) ) {
    EUROPA::TokenId goal(entity);
    
    if( m_goals.erase(goal) ) 
      m_recalled.insert(goal);
  }
}

bool DbCore::propagate() {
  if( m_reactor.assembly().propagate() ) {
    process_pending();
    return true;
  }
  debugMsg(dbg("trex:core"), "Failed to propagate");
  return false;
}

void DbCore::process_pending() {
  EUROPA::TokenSet pending;
  EUROPA::IntervalIntDomain scope(m_reactor.getInitialTick(),
				  m_reactor.getFinalTick());
  TICK cur = m_reactor.getCurrentTick();
  
  std::swap(pending, m_pending);
  condDebugMsg(!pending.empty(), dbg("trex:pending"), '['<<cur<<"] "
	       <<pending.size()<<" pending tokens");
  for(EUROPA::TokenSet::const_iterator i=pending.begin(); pending.end()!=i; ++i) {
    if( m_reactor.assembly().ignored(*i) ) {
      debugMsg(dbg("trex:token"), '['<<cur<<"] IGNORE "<<(*i)->toString()
	       <<": "<<(*i)->getPredicateName().toString());
    } else {
      EUROPA::TokenId master = (*i)->master();
      
      if( master.isNoId() ) {
	if( (*i)->getState()->baseDomain().isMember(EUROPA::Token::REJECTED) ) {
	  m_goals.insert(*i);
	  // A new goal necessirly start during the mission 
	  (*i)->start()->restrictBaseDomain(scope);
	  propagate();
	  debugMsg(dbg("trex:token"), '['<<cur<<"] GOAL "<<(*i)->toString()
		   <<": "<<(*i)->getPredicateName().toString());
	}
      }
      if( (*i)->isFact() ) {
	debugMsg(dbg("trex:token"), '['<<cur<<"] FACT "<<(*i)->toString()
		 <<": "<<(*i)->getPredicateName().toString());
      }
      if( (*i)->isInactive() )
	add_to_agenda(*i);
    }
  }
}

void DbCore::add_to_agenda(EUROPA::TokenId const &token) {
  m_pending.erase(token);
  if( token->isFact() ) 
    m_facts_agenda.insert(token);
  else 
    m_agenda.insert(token);
}

void DbCore::remove_from_agenda(EUROPA::TokenId const &token) {
  if( token->isFact() ) 
    m_facts_agenda.erase(token);
  else 
    m_agenda.erase(token);
}

void DbCore::remove_observation(EUROPA::TokenId const &tok) {
  TICK cur = m_reactor.getCurrentTick();

  if( m_observations.erase(tok) &&
      tok->end()->getUpperBound()>=cur ) {
    EUROPA::ObjectDomain const &domain=tok->getObject()->baseDomain();
    EUROPA::ObjectId object = domain.getObject(domain.getSingletonValue());
    state_map::iterator i = m_internals.find(object);
    if( m_internals.end()!=i ) {
      if( tok==i->second )
	i->second = EUROPA::TokenId::noId();
    } else {
      i = m_externals.find(tok);
      if( m_externals.end()!=i &&
	  tok==i->second ) 
	i->second = EUROPA::TokenId::noId();
    }
  }
}

void DbCore::archive() {
  TICK cur = m_reactor.getCurrentTick();

  if( m_reactor.assembly().inactive() ) {
    EUROPA::TokenSet terminated = m_terminated, removed;

    condDebugMsg(!terminated.empty(), dbg("trex:archive"), 
		 "Evaluating "<<terminated.size()<<" facts for archiving.");
    for(EUROPA::TokenSet::const_iterator i=terminated.begin(); 
	terminated.end()!=i; ++i) {
      EUROPA::TokenId token = *i;
      // First case the token is inactive 
      if( token->isInactive() ) {
	debugMsg(dbg("trex:archive"), "DISCARD inactive fact "<<token->toString());
	removed.insert(token);
      } else {
	if( token->isMerged() ) {
 	  token = token->getActiveToken();
	  debugMsg(dbg("trex:archive"), "REPLACE fact "<<(*i)->toString()
		   <<" by "<<token->toString());
	  token->restrictBaseDomains();
	  token->makeFact();
	  m_terminated.insert(token);
	  removed.insert(token);
	} else 
	  token->restrictBaseDomains();
	// Look for merged goals or tokens with no master
	EUROPA::TokenSet tokens = token->getMergedTokens();
	for(EUROPA::TokenSet::const_iterator j=tokens.begin(); tokens.end()!=j; ++j) {
	  if( remove_goal(*j, false) || (*j)->master().isNoId() ) {
	    debugMsg(dbg("trex:archive"), 
		     "DISCARD merged token "<<(*j)->toString());
	    removed.insert(token);
	  }
	}
	// Look for inactive slaves 
	tokens = token->slaves();
	for(EUROPA::TokenSet::const_iterator j=tokens.begin(); tokens.end()!=j; ++j) {
	  if( !(*j)->isActive() 
	      && (*j)->end()->lastDomain().getUpperBound()<=cur ) {
	    if( !( (*j)->isMerged() && (*j)->isFact() ) )
		removed.insert(*j);
	  }
	}
	// if( token->slaves().empty() ) {
	//   EUROPA::TokenId master = token->master();
	//   if( master.isNoId() )
	//     removed.insert(token);
	// }
      }
    }    
    terminated = m_recalled;
    for(EUROPA::TokenSet::const_iterator i=terminated.begin(); 
	terminated.end()!=i; ++i) {
      // for now play it safe and just remove the one that are not active
      // in the future we may be more aggressive on this one
      if( !(*i)->isActive() ) {
	debugMsg(dbg("trex:archive"), "DISCARD recalled goal "<<(*i)->toString());
	removed.insert(*i);
      }
    }
    condDebugMsg(!removed.empty(), dbg("trex:archive"), 
		 "Archiving "<<removed.size()<<" tokens");
    for(EUROPA::TokenSet::const_iterator i=removed.begin(); 
	removed.end()!=i; ++i)
      (*i)->discard();
  }
}


// europa callbacks

void DbCore::notifyAdded(EUROPA::TokenId const &token) {
  debugMsg(dbg("trex:token:state"), "ADD "<<token->toString()
	   <<": "<<token->getPredicateName().toString());
  m_pending.insert(token);
}

void DbCore::notifyRemoved(EUROPA::TokenId const &token) {
  debugMsg(dbg("trex:token:state"), "REMOVE "<<token->toString()
	   <<": "<<token->getPredicateName().toString());
  m_pending.erase(token);
  remove_from_agenda(token);
  remove_goal(token, false);
  remove_observation(token);
  m_terminated.erase(token);
  m_recalled.erase(token);
}

void DbCore::notifyActivated(EUROPA::TokenId const &token) {
  debugMsg(dbg("trex:token:state"), "ACTIVE "<<token->toString()
	   <<": "<<token->getPredicateName().toString());
  remove_from_agenda(token);
}

void DbCore::notifyDeactivated(EUROPA::TokenId const &token) {
  debugMsg(dbg("trex:token:state"), "INACTIVE "<<token->toString()
	   <<": "<<token->getPredicateName().toString());
  add_to_agenda(token);
}

void DbCore::notifyMerged(EUROPA::TokenId const &token) {
  debugMsg(dbg("trex:token:state"), "MERGE "<<token->toString()
	   <<": "<<token->getPredicateName().toString());
  remove_from_agenda(token);
}

void DbCore::notifySplit(EUROPA::TokenId const &token) {
  debugMsg(dbg("trex:token:state"), "SPLIT "<<token->toString()
	   <<": "<<token->getPredicateName().toString());
  add_to_agenda(token);
}

void DbCore::notifyCommitted(EUROPA::TokenId const &token) {
  debugMsg(dbg("trex:token:state"), "COMMIT "<<token->toString()
	   <<": "<<token->getPredicateName().toString());
}

void DbCore::notifyRejected(EUROPA::TokenId const &token) {
  debugMsg(dbg("trex:token:state"), "REJECT "<<token->toString()
	   <<": "<<token->getPredicateName().toString());
  remove_from_agenda(token);
}

void DbCore::notifyTerminated(EUROPA::TokenId const &token) {
  debugMsg(dbg("trex:token:state"), "TERMINATE "<<token->toString()
	   <<": "<<token->getPredicateName().toString());
  remove_from_agenda(token);
  m_terminated.erase(token);
}

