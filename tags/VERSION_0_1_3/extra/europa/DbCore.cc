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

EUROPA::TokenId DbCore::find_current(EUROPA::TimelineId const &tl, TICK now) {
  seq_iter i, last;
  boost::tie(i, last) = find_after(tl, now);
  
  if( last!=i ) {
    if( (*i)->start()->lastDomain().getUpperBound()<=now )
      return *i;
  }
  return EUROPA::TokenId::noId();
}


// structors 

DbCore::DbCore(EuropaReactor &owner)
  :EUROPA::PlanDatabaseListener(owner.assembly().plan_db()),
   m_reactor(owner) {}

DbCore::~DbCore() {}

// modifiers 

void DbCore::initialize() {
  TICK initial = m_reactor.getInitialTick();
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

void DbCore::notify(state_map::value_type const &obs) {
  std::pair<state_map::iterator, bool> ret = m_externals.insert(obs);
  if( !ret.second ) {
    EUROPA::TokenId pred = ret.first->second;
    m_completed_obs.insert(pred);
    ret.first->second = obs.second;
  }
  debugMsg("trex:token", '['<<m_reactor.getCurrentTick()<<"] OBSERVATION "
	   <<obs.first->toString()
	   <<'.'<<obs.second->getUnqualifiedPredicateName().toString());
}

bool DbCore::update_externals() {
  TICK cur = m_reactor.getCurrentTick();
  EUROPA::IntervalIntDomain now(cur, cur), future(cur+1, PLUS_INFINITY);
 
  // First terminate former observations
  for(EUROPA::TokenSet::const_iterator i=m_completed_obs.begin();
      m_completed_obs.end()!=i; ++i) {
    debugMsg("trex:sync", '['<<cur<<"] closing old observation "
	     <<(*i)->toString()<<": "<<(*i)->getPredicateName().toString());
    (*i)->end()->restrictBaseDomain(now);    
    debugMsg("trex:sync", "\tend="
	     <<(*i)->end()->baseDomain().toString());
    // if( (*i)->canBeTerminated(cur+1) ) {
    //   debugMsg("trex:sync", '['<<cur<<"] terminating old observation "
    // 	       <<(*i)->toString());
    //   (*i)->terminate();
    // }
  }
  m_completed_obs.clear();
  
  // Now apply inertial value asumption on cuureent observations
  for(state_map::const_iterator i=m_externals.begin(); m_externals.end()!=i; ++i) {
    debugMsg("trex:sync", '['<<cur<<"] applying inertial value assumption to "
	     <<i->first->toString()
	     <<'.'<<i->second->getUnqualifiedPredicateName().toString());
    i->second->end()->restrictBaseDomain(future);    
  }
  debugMsg("trex:sync", '['<<cur<<"] Propagating external state updates");
  return propagate();
}

bool DbCore::synchronize() {
  size_t steps = 0;
  
  if( resolve(steps) ) {
    if( update_internals(steps) ) {
      if( resolve(steps) ) {
	debugMsg("trex:sync", "SUCESS after "<<steps<<" steps.");
	return true;
      } else 
	debugMsg("trex:sync", "FAILED to resolve internal timelines updates");
    } else
      debugMsg("trex:sync", "FAILED to identify internal timelines state");
  } else
    debugMsg("trex:sync", "FAILED to resolve current state");
  return false;
}

bool DbCore::process_agenda(EUROPA::TokenSet agenda, size_t &steps) {
  TICK cur = m_reactor.getCurrentTick();
  Assembly &assembly = m_reactor.assembly();

  for(EUROPA::TokenSet::const_iterator i=agenda.begin(); agenda.end()!=i; ++i) {
    if( (*i)->getObject()->lastDomain().isSingleton() && (*i)->isInactive() ) {
      EUROPA::TokenId cand;
      
      if( assembly.in_synch_scope(*i, cand) ) {
	debugMsg("trex:agenda", "resolving "<<(*i)->toString());
	++steps;
	if( !( assembly.resolve(*i, cand, steps) && propagate() ) ) {
	  debugMsg("trex:agenda", "FAILED to resolve "<<(*i)->toString());
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
    condDebugMsg(!m_facts_agenda.empty(), "trex:sync", 
		 m_facts_agenda.size()<<" facts in the agenda.");
    if( !process_agenda(m_facts_agenda, steps) ) {
      debugMsg("trex:sync", "Failed to resolve facts");
      return false;
    }
    // then process everything else
    condDebugMsg(!m_agenda.empty(), "trex:sync", 
		 m_agenda.size()<<" tokens in the agenda.");
    if( !process_agenda(m_agenda, steps) ) {
      debugMsg("trex:sync", "Failed to resolve tokens");
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
    EUROPA::TokenId active = i->second,
      last_obs = find_current(i->first, cur);

    if( active.isId() ) {
      active->end()->restrictBaseDomain(future);
      if( active->isMerged() )
      active = active->getActiveToken();
    }

    if( last_obs.isNoId() ) {
      debugMsg("trex:sync", '['<<cur<<"] setting "<<i->first->toString()
	       <<" state to its default.");
      if( !assembly.insert_default(i->first, last_obs, steps) ) {
	debugMsg("trex:sync", "FAILED to insert default value on "
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

bool DbCore::reset_goal(EUROPA::TokenId const &tok, bool aggressive) {
  TICK cur = m_reactor.getCurrentTick();
  EUROPA::IntervalIntDomain future(cur, PLUS_INFINITY);
  EUROPA::TokenId active = (tok->isMerged()?tok->getActiveToken():tok);
  
  if( active->end()->baseDomain().getUpperBound()<cur ) {
    debugMsg("trex:relax", "DISCARD completed goal "<<tok->toString()
	     <<": "<<tok->getPredicateName().toString());
    return true;
  }
  if( tok->start()->baseDomain().getUpperBound()<=cur ) {
    if( aggressive ) {
      debugMsg("trex:relax", "DISCARD goal started  in the past "<<tok->toString()
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
  debugMsg("trex:relax", "START "<<(aggressive?"aggressive ":"")<<"relax");
  // Put the solver out of the way 
  assembly.solver().clear();
  assembly.mark_inactive();

  EUROPA::TokenSet all = assembly.plan_db()->getTokens();
  TICK cur = m_reactor.getCurrentTick();

  for(EUROPA::TokenSet::const_iterator i=all.begin(); all.end()!=i; ++i) {
    if( (*i)->isFact() ) {
      if( aggressive && (*i)->end()->baseDomain().getUpperBound()<=cur ) {
	// In aggressive mode we get rid of the past 
	debugMsg("trex:relax", "DISCARD past fact "<<(*i)->toString()
		 <<(*i)->getPredicateName().toString());
	(*i)->discard();
      } else 
	(*i)->cancel();
    } else if( is_goal(*i) ) {
      if( reset_goal(*i, aggressive) )
	(*i)->discard();
      else {
	std::vector<EUROPA::ConstrainedVariableId> const &vars = (*i)->getVariables();
	(*i)->cancel();

	for(std::vector<EUROPA::ConstrainedVariableId>::const_iterator v=vars.begin();
	    vars.end()!=v; ++v) {
	  if( (*v)->canBeSpecified() && (*v)->isSpecified() )
	    (*v)->reset();
	  (*v)->relax();
	}
      }
    } else if( assembly.internal(*i) ) {
      // internal objects are either merged with a fact (or a goal) or 
      // from the future
      (*i)->discard();
    } else if( assembly.overlaps_now(*i) ) {
      if( aggressive ) {
	condDebugMsg(!(*i)->isMerged(), "trex:relax", "DISCARD token "
		     <<(*i)->toString()<<": "<<(*i)->getPredicateName().toString());
	(*i)->discard();
      } else if( !(*i)->isInactive() )
	(*i)->cancel();
    }
  }
  
  debugMsg("trex:relax", "SUCESS");
  m_reactor.logPlan();
  return true;
}

void DbCore::step() {
  Assembly &assembly = m_reactor.assembly();

  debugMsg("trex:step", '['<<m_reactor.getCurrentTick()<<"] START deliberation step");
  
  if( propagate() ) {
    DbSolver &solver = assembly.solver();
    
    if( assembly.inactive() ) 
      assembly.mark_active();
    solver.step();
    process_pending();
    if( solver.noMoreFlaws() ) 
      m_reactor.end_deliberation();
  }
}

void DbCore::doDispatch() {
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
	if( !m_reactor.assembly().in_deliberation(*it) )
	  m_reactor.request(i->first, *it);
    }
  }
}

void DbCore::doNotify() {
  TICK cur = m_reactor.getCurrentTick();
  EUROPA::DbClientId cli = m_reactor.assembly().plan_db()->getClient();
  EUROPA::IntervalIntDomain now(cur, cur), future(cur+1, PLUS_INFINITY);

  for(state_map::iterator i=m_internals.begin(); m_internals.end()!=i; ++i) {
    // Look for current state in the plan
    EUROPA::TokenId active = i->second,
      last_obs = find_current(i->first, cur);

    last_obs->end()->restrictBaseDomain(future);
    if( active.isId() ) {
      if( active->isMerged() )
	active = active->getActiveToken();
    }
    if( active!=last_obs ) {
      // create the fact
      EUROPA::TokenId obs = cli->createToken(last_obs->getPredicateName().c_str(),
					     NULL, false, true);
      obs->start()->restrictBaseDomain(now);
      obs->getObject()->specify(i->first->getKey());
      cli->merge(obs, last_obs);
      propagate();
      // restrict all the attribute and the object
      obs->getObject()->restrictBaseDomain(obs->getObject()->getLastDomain());

      for(std::vector<EUROPA::ConstrainedVariableId>::const_iterator 
	    p=obs->parameters().begin(); obs->parameters().end()!=p; ++p) 
	(*p)->restrictBaseDomain((*p)->lastDomain());
      if( i->second.isId() ) {
	i->second->end()->restrictBaseDomain(now);
	// if( i->second->canBeTerminated(cur+1) ) {
	// 	i->second->terminate();
	// }
      }
      i->second = obs;
      m_reactor.notify(i->first, i->second);
    }
    i->second->end()->restrictBaseDomain(future);
  }
}

bool DbCore::propagate() {
  if( m_reactor.assembly().propagate() ) {
    process_pending();
    return true;
  }
  debugMsg("trex:core", "Failed to propagate");
  return false;
}

void DbCore::process_pending() {
  EUROPA::TokenSet pending;
  EUROPA::IntervalIntDomain scope(m_reactor.getInitialTick(),
				  m_reactor.getFinalTick());
  TICK cur = m_reactor.getCurrentTick();
  
  std::swap(pending, m_pending);
  condDebugMsg(!pending.empty(), "trex:pending", '['<<cur<<"] "
	       <<pending.size()<<" pending tokens");
  for(EUROPA::TokenSet::const_iterator i=pending.begin(); pending.end()!=i; ++i) {
    if( m_reactor.assembly().ignored(*i) ) {
      debugMsg("trex:token", '['<<cur<<"] IGNORE "<<(*i)->toString()
	       <<": "<<(*i)->getPredicateName().toString());
    } else {
      EUROPA::TokenId master = (*i)->master();
      
      if( master.isNoId() ) {
	if( (*i)->getState()->baseDomain().isMember(EUROPA::Token::REJECTED) ) {
	  m_goals.insert(*i);
	  // A new goal necessirly start during the mission 
	  (*i)->start()->restrictBaseDomain(scope);
	  propagate();
	  debugMsg("trex:token", '['<<cur<<"] GOAL "<<(*i)->toString()
		   <<": "<<(*i)->getPredicateName().toString());
	}
      }
      if( (*i)->isFact() ) {
	debugMsg("trex:token", '['<<cur<<"] FACT "<<(*i)->toString()
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

// europa callbacks

void DbCore::notifyAdded(EUROPA::TokenId const &token) {
  debugMsg("trex:token:state", "ADD "<<token->toString()
	   <<": "<<token->getPredicateName().toString());
  m_pending.insert(token);
}

void DbCore::notifyRemoved(EUROPA::TokenId const &token) {
  debugMsg("trex:token:state", "REMOVE "<<token->toString()
	   <<": "<<token->getPredicateName().toString());
  m_pending.erase(token);
  remove_from_agenda(token);
  m_goals.erase(token);
  m_completed_obs.erase(token);
}

void DbCore::notifyActivated(EUROPA::TokenId const &token) {
  debugMsg("trex:token:state", "ACTIVE "<<token->toString()
	   <<": "<<token->getPredicateName().toString());
  remove_from_agenda(token);
}

void DbCore::notifyDeactivated(EUROPA::TokenId const &token) {
  debugMsg("trex:token:state", "INACTIVE "<<token->toString()
	   <<": "<<token->getPredicateName().toString());
  add_to_agenda(token);
}

void DbCore::notifyMerged(EUROPA::TokenId const &token) {
  debugMsg("trex:token:state", "MERGE "<<token->toString()
	   <<": "<<token->getPredicateName().toString());
  remove_from_agenda(token);
}

void DbCore::notifySplit(EUROPA::TokenId const &token) {
  debugMsg("trex:token:state", "SPLIT "<<token->toString()
	   <<": "<<token->getPredicateName().toString());
  add_to_agenda(token);
}

void DbCore::notifyCommitted(EUROPA::TokenId const &token) {
  debugMsg("trex:token:state", "COMMIT "<<token->toString()
	   <<": "<<token->getPredicateName().toString());
}

void DbCore::notifyRejected(EUROPA::TokenId const &token) {
  debugMsg("trex:token:state", "REJECT "<<token->toString()
	   <<": "<<token->getPredicateName().toString());
  remove_from_agenda(token);
}

void DbCore::notifyTerminated(EUROPA::TokenId const &token) {
  debugMsg("trex:token:state", "TERMINATE "<<token->toString()
	   <<": "<<token->getPredicateName().toString());
  remove_from_agenda(token);
}

