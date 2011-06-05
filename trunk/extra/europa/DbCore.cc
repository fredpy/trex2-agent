#include "EuropaReactor.hh"

# include <PLASMA/Token.hh>
# include <PLASMA/TokenVariable.hh>
# include <PLASMA/Timeline.hh>

# include <PLASMA/Debug.hh>

using namespace TREX::europa;

/*
 * class TREX::europa::DbCore 
 */

// structors 

DbCore::DbCore(EuropaReactor &owner) 
  :EUROPA::PlanDatabaseListener(owner.assembly().plan_db()), 
   m_reactor(owner) {}

DbCore::~DbCore() {}

inline bool DbCore::is_current(EUROPA::TokenId const &tok) const {
  return m_reactor.assembly().overlaps_now(tok);
}


// - modifiers

void DbCore::notify(DbCore::state_map::value_type const &obs) {
  std::pair<state_map::iterator, bool>
    ret = m_externals.insert(obs);
  obs.second->activate();
  if( !ret.second ) {
    EUROPA::TokenId pred = ret.first->second;
    m_completed_obs.insert(pred);
    ret.first->second = obs.second;
    obs.first->constrain(pred, obs.second);
  }
  debugMsg("trex:token", "["<<m_reactor.getCurrentTick()<<"] OBSERVATION "
	   <<obs.second->toString());
  m_observations.insert(obs.second);
}

bool DbCore::update_externals() {
  TREX::transaction::TICK cur = m_reactor.getCurrentTick();
  EUROPA::IntervalIntDomain now(cur, cur), future(cur+1, PLUS_INFINITY);
  
  // enforce that previous observations terminate now
  for(EUROPA::TokenSet::const_iterator i=this->m_completed_obs.begin();
      m_completed_obs.end()!=i; ++i)
    (*i)->end()->restrictBaseDomain(now);
  m_completed_obs.clear();
  // now extend the duration of current observation
  for(state_map::const_iterator i=m_externals.begin(); 
      m_externals.end()!=i; ++i) {
    i->second->end()->restrictBaseDomain(future);
  }
  return propagate();
}

bool DbCore::propagate() {
  if( m_reactor.assembly().propagate() ) {
    process_pendings();
    return true;
  }
  return false;  
}


void DbCore::process_pendings() {
  EUROPA::IntervalIntDomain scope(m_reactor.getInitialTick(),
                                  m_reactor.getFinalTick());
  EUROPA::TokenSet pendings;
  std::swap(pendings, m_pending);
  
  for(EUROPA::TokenSet::const_iterator i=pendings.begin(); 
      pendings.end()!=i; ++i) {
    if( m_reactor.assembly().ignored(*i) ) {
      // ignore_token(*i);
    } else {
      if( (*i)->master().isNoId() &&
         (*i)->getState()->baseDomain().isMember(EUROPA::Token::REJECTED) ) {
        m_goals.insert(*i);
        // A goal necessarily starts during the mission 
        (*i)->start()->restrictBaseDomain(scope);
      }
      add_to_agenda(*i);
    }
  }
}

void DbCore::add_to_agenda(EUROPA::TokenId const &tok) {
  m_agenda.insert(tok);
}

bool DbCore::resolve(size_t &steps) {
  size_t last_count;
  do {
    EUROPA::TokenSet agenda = m_agenda;
    last_count = steps;
    
    if( !propagate() )
      return false;
    for(EUROPA::TokenSet::const_iterator i=agenda.begin();
        agenda.end()!=i; ++i) {
      if( (*i)->getObject()->lastDomain().isSingleton() && (*i)->isInactive() ) {
        EUROPA::TokenId candidate;
        if( m_reactor.assembly().in_synch_scope(*i, candidate) ) {
          ++steps;
          if( !( m_reactor.assembly().resolve(*i, candidate, steps) && propagate() ) )
            return false;
        }
      }
    }
  } while( last_count!=steps );
  return true;
}

void DbCore::set_internal(EUROPA::ObjectId const &obj) {
  m_internals.insert(std::make_pair(obj, EUROPA::TokenId::noId()));
}


bool DbCore::update_internals(size_t &steps) {
  if( m_reactor.assembly().invalid() )
    return false;
  TREX::transaction::TICK cur=m_reactor.getCurrentTick();
  EUROPA::IntervalIntDomain future(cur+1, PLUS_INFINITY);
  
  for(state_map::iterator i=m_internals.begin();
      m_internals.end()!=i; ++i) {
    EUROPA::TimelineId timeline(i->first);
    std::list<EUROPA::TokenId> const &tokens=timeline->getTokenSequence();
    std::list<EUROPA::TokenId>::const_iterator it=tokens.begin();
    i->second = EUROPA::TokenId::noId();
    
    while( tokens.end()!=it ) {
      if( (*it)->start()->lastDomain().getLowerBound() > cur )
        break;
      else if( (*it)->end()->lastDomain().getUpperBound()>cur ) {
        i->second = *it;
        break;
      }
      ++it;
    }
    if( i->second.isNoId() ) {
      if( !m_reactor.assembly().insert_default(i->first, i->second, steps) )
        return false;
    } else {
      if( i->second->start()->lastDomain().getUpperBound()<cur )
        i->second->end()->restrictBaseDomain(future);
      else
        i->second->start()->specify(cur);
    }
    if( !propagate() )
      return false;
  }
  
  return true;
}

void DbCore::reset_observations() {
  EUROPA::IntervalIntDomain future(m_reactor.getCurrentTick()+1,
                                   PLUS_INFINITY);
  // remove all the tokens merged to observations
  for(EUROPA::TokenSet::const_iterator i=m_observations.begin();
      m_observations.end()!=i;++i) {
    EUROPA::TokenSet merged = (*i)->getMergedTokens();
    for(EUROPA::TokenSet::iterator it=merged.begin();
        merged.end()!=it; ++it) {
      (*it)->cancel();
    }
  }
  // make sure that last observation end is correctly set to be 
  // after current tick
  for(state_map::const_iterator i=m_externals.begin();
      m_externals.end()!=i; ++i) {
    // if(i->second->isId()) {   // do not need this as I should have 
                               //an observation here
    i->second->end()->relax();
    i->second->end()->restrictBaseDomain(future);
    // }
  }
}

bool DbCore::reset_goal(EUROPA::TokenId const &tok, bool agressive) {
  TREX::transaction::TICK cur = m_reactor.getCurrentTick();
  EUROPA::IntervalIntDomain future(cur, PLUS_INFINITY);
  EUROPA::TokenId active = tok;
  if( tok->isMerged() )
    active = tok->getActiveToken();

  if( active->end()->baseDomain().getUpperBound()<=cur ) {
    // already completed => we can remove it from the goals
    m_goals.erase(tok);
    m_reactor.removed(tok);
    return agressive || tok->isMerged();
  }    
  
  if( tok->isActive() ) {
    // if it is active relax all the attributes
    std::vector<EUROPA::ConstrainedVariableId> const &vars = tok->getVariables();
    for(std::vector<EUROPA::ConstrainedVariableId>::const_iterator i=vars.begin();
        vars.end()!=i; ++i) 
      if( (*i)->canBeSpecified() && (*i)->isSpecified() )
        (*i)->reset();
  }
  if( tok->start()->baseDomain().getUpperBound()<=cur ) {
    // necessarily starts in the past 
    if( agressive ) {
      // when agressive just get rid of it
      m_goals.erase(tok);
      m_reactor.removed(tok);
      return true;
    }
  }
  if( !tok->isInactive() ) {
    tok->cancel();
    tok->start()->restrictBaseDomain(future);
  }
  return false;
}


bool DbCore::relax(bool agressive) {
  m_reactor.log("core")<<"Beginning database relax"
		       <<(agressive?" and forget the past":".");
  debugMsg("trex:relax", "START relax("<<(agressive?"true":"false")<<")");
  EUROPA::TokenSet all = m_reactor.assembly().plan_db()->getTokens();
  TREX::transaction::TICK cur = m_reactor.getCurrentTick();
  std::vector<EUROPA::TokenId> kept;
  
  for(EUROPA::TokenSet::const_iterator i=all.begin(); all.end()!=i; ++i) {
    if( (*i)->isFact() ) {
      if( agressive && (*i)->end()->baseDomain().getUpperBound()<=cur ) {
        // forget about the past in agressive mode
	debugMsg("trex:relax:discard", "DISCARD past fact "<<(*i)->toString());
        (*i)->discard();
      } else 
        kept.push_back(*i);
    } else if( m_reactor.assembly().internal(*i) ) {
      if( is_goal(*i) && reset_goal(*i, agressive) ) {
	debugMsg("trex:relax:discard", "DISCARD goal "<<(*i)->toString());
        (*i)->discard();
      } else if( (*i)->isMerged() || agressive ||
		 (*i)->start()->baseDomain().getLowerBound()>cur ) {
	condDebugMsg(!(*i)->isMerged(), 
		     "trex:relax:discard", "DISCARD internal token "<<(*i)->toString());
        (*i)->discard();
      } else 
        kept.push_back(*i);
    } else if( is_current(*i) ) {
      if( (*i)->isMerged() || agressive ) { 
	condDebugMsg(!(*i)->isMerged(), 
		     "trex:relax:discard", "DISCARD current token "<<(*i)->toString());	
        (*i)->discard();
      } else 
        kept.push_back(*i);
    } else {
      condDebugMsg(!(*i)->isMerged(), 
		   "trex:relax:discard", "DISCARD token "<<(*i)->toString());	
      (*i)->discard();
    }
  }
  // Now we should have only the tokens that matter : lets check that everything 
  // is fine 
  for(std::vector<EUROPA::TokenId>::const_iterator i=kept.begin();
      kept.end()!=i; ++i) {
    if( !(*i)->isActive() ) {
      size_t steps;
      debugMsg("trex:relax", "ACTIVATE token "<<(*i)->toString());
      if( !m_reactor.assembly().insert_token(*i, steps) ) {
	debugMsg("trex:relax", "FAILURE on "<<(*i)->toString());
      }      
    }
  }
  debugMsg("trex:relax", "SUCESS");	
  return true;
}


bool DbCore::synchronize() {
  size_t steps = 0;
  return propagate() && resolve(steps) 
  && update_internals(steps) && resolve(steps);
}

void DbCore::doNotify() {
  TREX::transaction::TICK cur = m_reactor.getCurrentTick();
  for(state_map::const_iterator i=m_internals.begin(); 
      m_internals.end()!=i; ++i) {
    if( i->second.isId() && i->second->start()->lastDomain().isMember(cur) )
      m_reactor.notify(i->first, i->second);
  }
}


void DbCore::remove_from_agenda(EUROPA::TokenId const &tok) {
  m_agenda.erase(tok);
}

// - europa callbacks 

void DbCore::notifyAdded(EUROPA::TokenId const &token) {
  debugMsg("trex:token", "["<<m_reactor.getCurrentTick()<<"] ADD "<<token->toString()
	   <<": "<<token->getPredicateName().toString());
  m_pending.insert(token);
}

void DbCore::notifyRemoved(EUROPA::TokenId const &token) {
  debugMsg("trex:token", "["<<m_reactor.getCurrentTick()<<"] REMOVE "<<token->toString()
	   <<": "<<token->getPredicateName().toString());
  m_pending.erase(token);
  m_goals.erase(token);
  remove_from_agenda(token);
  m_observations.erase(token);
  m_completed_obs.erase(token);
}

void DbCore::notifyActivated(EUROPA::TokenId const &token) {
  debugMsg("trex:token", "["<<m_reactor.getCurrentTick()<<"] ACTIVATE "<<token->toString()
	   <<": "<<token->getPredicateName().toString());
  remove_from_agenda(token);
}

void DbCore::notifyDeactivated(EUROPA::TokenId const &token) {
  debugMsg("trex:token", "["<<m_reactor.getCurrentTick()<<"] DEACTIVATE "<<token->toString()
	   <<": "<<token->getPredicateName().toString());
  add_to_agenda(token);
}

void DbCore::notifyMerged(EUROPA::TokenId const &token) {
  debugMsg("trex:token", "["<<m_reactor.getCurrentTick()<<"] MERGE "<<token->toString()
	   <<": "<<token->getPredicateName().toString());
  remove_from_agenda(token);
}

void DbCore::notifySplit(EUROPA::TokenId const &token) {
  debugMsg("trex:token", "["<<m_reactor.getCurrentTick()<<"] SPLIT "<<token->toString()
	   <<": "<<token->getPredicateName().toString());
  add_to_agenda(token);
}

void DbCore::notifyCommitted(EUROPA::TokenId const &token) {
  debugMsg("trex:token", "["<<m_reactor.getCurrentTick()<<"] COMMIT "<<token->toString()
	   <<": "<<token->getPredicateName().toString());
}

void DbCore::notifyRejected(EUROPA::TokenId const &token) {
  debugMsg("trex:token", "["<<m_reactor.getCurrentTick()<<"] REJECT "<<token->toString()
	   <<": "<<token->getPredicateName().toString());
  remove_from_agenda(token);
}

void DbCore::notifyTerminated(EUROPA::TokenId const &token) {
  debugMsg("trex:token", "["<<m_reactor.getCurrentTick()<<"] TERMINATE "<<token->toString()
	   <<": "<<token->getPredicateName().toString());
}
