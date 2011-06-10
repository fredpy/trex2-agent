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
  debugMsg(dbg("trex:token"), '['<<m_reactor.getCurrentTick()<<"] OBSERVATION "
	   <<obs.first->toString()
	   <<'.'<<obs.second->getUnqualifiedPredicateName().toString());
  m_observations.insert(obs.second);
}

bool DbCore::update_externals() {
  TICK cur = m_reactor.getCurrentTick();
  EUROPA::IntervalIntDomain now(cur, cur), future(cur+1, PLUS_INFINITY);
 
  // First terminate former observations
  for(EUROPA::TokenSet::const_iterator i=m_completed_obs.begin();
      m_completed_obs.end()!=i; ++i) {
    debugMsg(dbg("trex:sync"), '['<<cur<<"] closing old observation "
	     <<(*i)->toString()<<": "<<(*i)->getPredicateName().toString());
    (*i)->end()->restrictBaseDomain(now);    
    debugMsg(dbg("trex:sync"), '['<<cur<<"] end time closed :"
	     <<"\n\tbase "<<(*i)->end()->baseDomain().toString()
	     <<"\n\tlast "<<(*i)->end()->lastDomain().toString());
    m_reactor.removed(*i);
  }
  m_completed_obs.clear();
  
  // Now apply inertial value asumption on cuureent observations
  for(state_map::const_iterator i=m_externals.begin(); m_externals.end()!=i; ++i) {
    debugMsg(dbg("trex:sync"), '['<<cur<<"] applying inertial value assumption to "
	     <<i->first->toString()
	     <<'.'<<i->second->getUnqualifiedPredicateName().toString());
    i->second->end()->restrictBaseDomain(future);    
  }
  debugMsg(dbg("trex:sync"), '['<<cur<<"] Propagating external state updates");
  return propagate();
}

bool DbCore::synchronize() {
  size_t steps = 0;
  
  if( resolve(steps) ) {
    if( update_internals(steps) ) {
      if( resolve(steps) ) {
	debugMsg(dbg("trex:sync"), "SUCESS after "<<steps<<" steps.");
	return true;
      } else 
	debugMsg(dbg("trex:sync"), "FAILED to resolve internal timelines updates");
    } else
      debugMsg(dbg("trex:sync"), "FAILED to identify internal timelines state");
  } else
    debugMsg(dbg("trex:sync"), "FAILED to resolve current state");
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

void DbCore::remove_goal(EUROPA::TokenId const &tok) {
  if( m_goals.erase(tok) )
    m_reactor.removed(tok);
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
  // finally I probably should  get rid of other tokens ... 
  
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
      // From here  I know that I have a new observation 
      EUROPA::TokenId obs = cli->createToken((*p_cur)->getPredicateName().c_str(),
					     NULL, false, true);
      obs->start()->restrictBaseDomain(now);
      obs->end()->restrictBaseDomain(future);
      obs->getObject()->specify(i->first->getKey());
      cli->merge(obs, *p_cur);
      propagate();
      // restrict the fact attributes to what we are going to produce
      obs->getObject()->restrictBaseDomain(obs->getObject()->getLastDomain());
      for(std::vector<EUROPA::ConstrainedVariableId>::const_iterator 
	    v=obs->parameters().begin(); obs->parameters().end()!=v; ++v) 
	(*v)->restrictBaseDomain((*v)->lastDomain());	            
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
      }
      i->second = obs;
      m_reactor.notify(i->first, obs);
    }
    i->second->end()->restrictBaseDomain(future);
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
  remove_goal(token);
  m_completed_obs.erase(token);
  remove_observation(token);
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
}

