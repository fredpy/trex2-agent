#include "EuropaReactor.hh"

# include <PLASMA/Token.hh>
# include <PLASMA/TokenVariable.hh>
# include <PLASMA/Timeline.hh>

using namespace TREX::europa;

/*
 * class TREX::europa::DbCore 
 */

// structors 

DbCore::DbCore(EuropaReactor &owner) 
  :EUROPA::PlanDatabaseListener(owner.assembly().plan_db()), 
   m_reactor(owner) {}

DbCore::~DbCore() {}

// modifiers :

void DbCore::set_internal(EUROPA::ObjectId const &obj) {
  m_internal_obs.insert(std::make_pair(obj, EUROPA::TokenId::noId()));
}

bool DbCore::initialize() {
  EUROPA::TokenSet const &tokens = m_reactor.assembly().plan_db()->getTokens();
  std::list<EUROPA::TokenId> facts;
  for(EUROPA::TokenSet::const_iterator i=tokens.begin(); tokens.end()!=i; ++i) {
    if( (*i)->isFact() ) 
      facts.push_back(*i);
  }
  apply_facts(facts);
  return propagate();  
}

void DbCore::notify(EUROPA::ObjectId const &obj, EUROPA::TokenId const &tok) {
  std::pair<current_state_map::iterator, bool>
    ret = m_external_obs.insert(std::make_pair(obj, tok));

  m_observations.insert(tok);
  if( !ret.second ) {
    m_completed_obs.insert(ret.first->second);
    ret.first->second = tok;
  }
  m_reactor.log("core")<<"OBS "<<tok->toString()
		       <<": "<<obj->getName().toString()<<'.'
		       <<tok->getUnqualifiedPredicateName().toString();
}

void DbCore::commit_and_restrict(EUROPA::TokenId const &token) {
  token->commit();
  token->getState()->touch(); // notify the solver
  // propagate constraints before doing anything 
  propagate();
  
  token->getObject()->restrictBaseDomain(token->getObject()->lastDomain());
  // restrict start to its current value 
  token->start()->restrictBaseDomain(token->start()->lastDomain());
  if( token->end()->lastDomain().getUpperBound() < m_reactor.getCurrentTick() )
    token->end()->restrictBaseDomain(token->end()->lastDomain());
  else 
    token->end()->restrictBaseDomain(EUROPA::IntervalIntDomain(m_reactor.getCurrentTick(), 
							       PLUS_INFINITY));
}

bool DbCore::complete_externals() {
  TREX::transaction::TICK cur = m_reactor.getCurrentTick();
  EUROPA::IntervalIntDomain future(cur+1, PLUS_INFINITY), 
    now(cur, cur);
  
  // Close completed observations
  for(EUROPA::TokenSet::iterator it=m_completed_obs.begin(); 
      m_completed_obs.end()!=it; ++it) {
    if( (*it)->end()->lastDomain().getLowerBound()>cur ) {
      m_reactor.log("synchronize")<<"Unexpected observation received.\n";
      m_reactor.assembly().mark_invalid();
      return false;
    }
    (*it)->end()->restrictBaseDomain(now);      
    if( !propagate() ) {
      m_reactor.log("synchronize")<<"Failed to terminate external observation "
        <<(*it)->getPredicateName().toString();
      m_reactor.assembly().mark_invalid();
      return false;
    }
  }
  m_completed_obs.clear();

  for(std::map<EUROPA::ObjectId, EUROPA::TokenId>::const_iterator it=m_external_obs.begin();
      m_external_obs.end()!=it; ++it) {
    if( it->second->end()->lastDomain().getUpperBound()==cur ) {
      m_reactor.log(it->first->getName().toString())<<"Missed expected observation.\n";
      m_reactor.assembly().mark_invalid();
      return false;
    }
    it->second->end()->restrictBaseDomain(future);
    if( !propagate() ) {
      m_reactor.log(it->first->getName().toString())
	<<"Failed to apply inertial value assumption.";
      m_reactor.assembly().mark_invalid();
      return false;
    }
  }
  return true;
}


void DbCore::commit() {
  if( !m_reactor.assembly().invalid() ) {
    for(EUROPA::TokenSet::const_iterator it=m_observations.begin(); 
	m_observations.end()!=it; ++it) {
#ifdef BACKWARD
      EUROPA::TokenId active = ((*it)->isActive()?*it:(*it)->getActiveToken());
#else 
      EUROPA::TokenId active = *it;
#endif
      if( !active->isCommitted() )
	commit_and_restrict(active);
      if( !propagate() )
	return;
    }
  }
}


void DbCore::apply_facts(std::list<EUROPA::TokenId> const &facts) {
  EUROPA::IntervalIntDomain mission_scope(m_reactor.getInitialTick(), 
					  m_reactor.getFinalTick()),
    end_scope(m_reactor.getInitialTick()+1, PLUS_INFINITY);

  for(std::list<EUROPA::TokenId>::const_iterator i=facts.begin(); 
      facts.end()!=i; ++i) {    
    if( !(*i)->getObject()->baseDomain().isSingleton() )
      throw TokenError(**i, "Facts should be linked to a unique object");
    
    EUROPA::ObjectDomain const &dom = (*i)->getObject()->baseDomain();

    EUROPA::ObjectId 
      object = dom.getObject(dom.getSingletonValue());
    
    if( m_reactor.assembly().isExternal(object) ) 
      throw TokenError(**i, "Facts cannot be applied to External timelines");
    
    if( m_reactor.assembly().isIgnored(object) ) {
      m_reactor.log("WARN")<<"Ignoring fact "<<(*i)->toString()
			   <<":"<<object->toString()<<'.'
			   <<(*i)->getUnqualifiedPredicateName().toString();
      m_reactor.assembly().ignore(*i);
    } else {
      EUROPA::IntervalIntDomain startBound(mission_scope),
	endBound(end_scope);
      startBound.intersect((*i)->start()->baseDomain());
      if( startBound.isEmpty() ) {
	m_reactor.log("WARN")<<"Ignoring fact "<<(*i)->toString()
			     <<" as it starts outside of mission scope";
	m_reactor.assembly().ignore(*i);
	continue;
      } 
      (*i)->start()->restrictBaseDomain(startBound);
      
      endBound.intersect((*i)->end()->baseDomain());
      if( endBound.isEmpty() ) {
	m_reactor.log("WARN")<<"Ignoring fact "<<(*i)->toString()
		      <<" as it cannot end after the initial tick.";
	m_reactor.assembly().ignore(*i);
	continue;
      }
      (*i)->end()->restrictBaseDomain(endBound);
      (*i)->activate();
      
      object->constrain(*i, *i);
    }
  }
}


bool DbCore::propagate() {
  if( m_reactor.assembly().invalid() )
    return false;
  if( !m_reactor.assembly().constraint_engine()->propagate() ) {
    m_reactor.log("core")<<"Inconsistent plan";
    m_reactor.assembly().mark_invalid();
  } else 
    processPending();
  
  return !m_reactor.assembly().invalid();
}

void DbCore::processPending() {
  EUROPA::TokenSet pending;
  std::swap(m_pending, pending);
  
  EUROPA::IntervalIntDomain mission_scope(m_reactor.getInitialTick(),
					  m_reactor.getFinalTick());

  for(EUROPA::TokenSet::const_iterator i=pending.begin(); pending.end()!=i; ++i) {
    if( m_reactor.assembly().ignored(*i) )
      ignore_token(*i);
    else {
      if( (*i)->master().isNoId() && 
	  (*i)->getState()->baseDomain().isMember(EUROPA::Token::REJECTED) ) {
	m_goals.insert(*i);
	m_reactor.log("core")<<"GOAL "<<(*i)->toString()
			     <<": "<<(*i)->getPredicateName().toString();
	// goals must start during the mission
	(*i)->start()->restrictBaseDomain(mission_scope);      
      }
    }
    add_to_agenda(*i);      
  }
}

bool DbCore::in_tick_horizon(EUROPA::TokenId const &tok) const {
  TREX::transaction::TICK cur = m_reactor.getCurrentTick();

  return tok->start()->lastDomain().getUpperBound()<=cur &&
    tok->end()->lastDomain().getLowerBound()>=cur &&
    tok->end()->lastDomain().getLowerBound()>m_reactor.getInitialTick();
}

bool DbCore::in_scope(EUROPA::TokenId const &tok) const {
  return !m_reactor.assembly().ignored(tok);
}

bool DbCore::is_unit(EUROPA::TokenId const &tok, EUROPA::TokenId &cand) {
  if( !tok->getObject()->lastDomain().isSingleton() )
    return false;
  std::vector<EUROPA::TokenId> compats;
  EUROPA::PlanDatabaseId db = m_reactor.assembly().plan_db();
  db->getCompatibleTokens(tok, compats, UINT_MAX, true);
  size_t choices(0);
  for(std::vector<EUROPA::TokenId>::const_iterator it=compats.begin(); 
      compats.end()!=it; ++it) {
    if( m_reactor.assembly().in_deliberation(*it) )
      continue;
    ++choices;
    cand = *it;
    if( choices>1 )
      break;
  }
  if( choices==1 && !db->hasOrderingChoice(tok) )
    return true;
  if( choices==0 ) {
    cand = EUROPA::TokenId::noId();
    return true;
  }
  return false;
}


bool DbCore::in_synch_scope(EUROPA::TokenId const &tok, EUROPA::TokenId &cand) {
  return in_tick_horizon(tok) && in_scope(tok) && 
    !m_reactor.assembly().in_deliberation(tok) && is_unit(tok, cand);
}


bool DbCore::merge_token(EUROPA::TokenId const &tok, EUROPA::TokenId const &cand) {
  if( m_reactor.assembly().invalid() || cand.isNoId() )
    return false;
  if( !( tok->isInactive() && 
	 tok->getState()->lastDomain().isMember(EUROPA::Token::MERGED) ) )
    return false;
  tok->merge(cand);
  propagate();
  return true;
}

bool DbCore::insert_token(EUROPA::TokenId const &tok, size_t &steps) {
  if( m_reactor.assembly().invalid() || 
      m_reactor.assembly().in_deliberation(tok) )
    return false;
  if( tok->end()->lastDomain().getUpperBound()<=m_reactor.getCurrentTick() ) {
    remove_from_agenda(tok);
    return true;
  }
  if( tok->isInactive() )
    tok->activate();
  EUROPA::ObjectDomain const &dom = tok->getObject()->lastDomain();

  EUROPA::ObjectId obj = dom.getObject(dom.getSingletonValue());
  if( EUROPA::TimelineId::convertable(obj) ) {

    std::vector<EUROPA::OrderingChoice> results;
    m_reactor.assembly().plan_db()->getOrderingChoices(tok, results, 1);
    if( results.empty() )
      return false;

    EUROPA::TokenId
      p = results[0].second.first,
      s = results[0].second.second;
    results[0].first->constrain(p, s);
  }
  propagate();
  for(EUROPA::TokenSet::const_iterator i=tok->slaves().begin(); 
      tok->slaves().end()!=i; ++i) {
    if( m_reactor.assembly().invalid() )
      return false;
    EUROPA::TokenId cand;
    if( !in_synch_scope(*i, cand) )
      continue;
    ++steps;
    if( !resolve_token(*i, steps, cand) )
      return false;
  }
  return true;
}

bool DbCore::resolve_token(EUROPA::TokenId const &tok, size_t &steps, 
			   EUROPA::TokenId const &cand) {
  if( merge_token(tok, cand) || insert_token(tok, steps) )
    return true;
  m_reactor.assembly().mark_invalid();
  return false;
}

void DbCore::recall(EUROPA::eint const &g) {
  m_recalls.insert(g);
}

bool DbCore::process_recalls() {
  bool recalls = false;
  
  for(std::set<EUROPA::eint>::iterator i=m_recalls.begin();
      m_recalls.end()!=i; ++i) {
    EUROPA::EntityId entity = EUROPA::Entity::getEntity(*i);
    if( entity.isId() ) {
      EUROPA::TokenId tok(entity);
      tok->discard();
      recalls = true;
    } 
  }
  m_recalls.clear();
  return recalls;
}

bool DbCore::is_current(EUROPA::TokenId const &tok) const {
  if( tok->isCommitted() && !tok->isTerminated() ) {
    TREX::transaction::TICK 
    end_max = EUROPA::cast_long(tok->end()->baseDomain().getUpperBound());
    
    if( m_reactor.assembly().internal(tok) ) 
      return end_max>=m_reactor.getCurrentTick();
    else 
      return end_max>m_reactor.getCurrentTick(); 
  }
  return false;
}


void DbCore::reset_observations() {
  EUROPA::TokenSet observations = m_observations;
  EUROPA::IntervalIntDomain future(m_reactor.getCurrentTick()+1, PLUS_INFINITY);
  
  for(EUROPA::TokenSet::iterator i=observations.begin(); observations.end()!=i; ++i) {
    EUROPA::TokenSet merged = (*i)->getMergedTokens();
    for(EUROPA::TokenSet::const_iterator j=merged.begin(); merged.end()!=j; ++j)
      (*j)->cancel();
    if( is_current(*i) ) {
      if( !( (*i)->isCommitted() || (*i)->isInactive() ) )
        (*i)->cancel();
      if( (*i)->end()->isSpecified() )
        (*i)->end()->reset();
      else 
        (*i)->end()->relax();
      (*i)->end()->restrictBaseDomain(future);
    } else 
      (*i)->discard();    
  }
}

void DbCore::reset_goals(bool discard) {
  std::vector<EUROPA::TokenId> past, present;
  EUROPA::TokenSet goals = m_goals;
  TREX::transaction::TICK cur = m_reactor.getCurrentTick();
  EUROPA::IntervalIntDomain future(cur, PLUS_INFINITY);
  
  for(EUROPA::TokenSet::const_iterator i=goals.begin(); goals.end()!=i; ++i) {
    EUROPA::TokenId 
      base = (*i)->isMerged()?(*i)->getActiveToken():*i;
    if( is_current(*i) ) {
      m_goals.erase(*i);
    } else if( discard ||
              base->end()->baseDomain().getUpperBound()<=cur ||
              ( (*i)->isRejected() &&   
               (*i)->start()->baseDomain().getUpperBound()<cur ) ||
              ( (*i)->isMerged() && is_current(base) ) )
      past.push_back(*i);
    else {
      if( (*i)->isActive() ) {
        std::vector<EUROPA::ConstrainedVariableId> const &
          vars = (*i)->getVariables();
        std::vector<EUROPA::ConstrainedVariableId>::const_iterator 
          j = vars.begin();
        for(; vars.end()!=j; ++j)
          if( (*j)->canBeSpecified() && (*j)->isSpecified() )
            (*j)->reset();
      }
      if( (*i)->start()->baseDomain().getUpperBound()<cur )
        past.push_back(*i);
      else if( !(*i)->isInactive() ) {
        (*i)->cancel();
        (*i)->start()->restrictBaseDomain(future);
      }
    }
  }
  
  for(std::vector<EUROPA::TokenId>::const_iterator i=past.begin();
      past.end()!=i; ++i) {
    m_reactor.removed(*i);
    (*i)->discard();
  }
}

void DbCore::copy_value(EUROPA::TokenId const &tok) {
  EUROPA::DbClientId cli = m_reactor.assembly().plan_db()->getClient();
  EUROPA::IntervalIntDomain future(m_reactor.getCurrentTick(), PLUS_INFINITY);
  EUROPA::TokenId 
    cpy = cli->createToken(tok->getPredicateName().c_str(), NULL, false);
  cpy->activate();
  
  for(size_t i=0; i<tok->parameters().size(); ++i) {
    cpy->parameters()[i]->restrictBaseDomain(tok->parameters()[i]->baseDomain());
  }
  cpy->getObject()->restrictBaseDomain(tok->getObject()->baseDomain());
  cpy->start()->restrictBaseDomain(tok->start()->baseDomain());
  if( is_observation(tok) ) {
    cpy->end()->restrictBaseDomain(future);
    m_observations.insert(cpy);    
  } else if( is_goal(tok) ) {
    cpy->duration()->restrictBaseDomain(tok->duration()->baseDomain());
    cpy->end()->restrictBaseDomain(tok->end()->baseDomain());
    m_goals.insert(cpy);
  }
  cpy->duration()->restrictBaseDomain(EUROPA::IntervalIntDomain(1, PLUS_INFINITY));
  cpy->commit();
}

void DbCore::reset_other_tokens(bool discard) {
  std::vector<EUROPA::TokenId> discards;
  EUROPA::TokenSet const all_toks = m_reactor.assembly().plan_db()->getTokens();
  for(EUROPA::TokenSet::const_iterator i=all_toks.begin();
      all_toks.end()!=i; ++i) {
    if( is_current(*i) ) {
      if( !discard || is_observation(*i) )
        copy_value(*i);
      discards.push_back(*i);
    } else if( !( is_goal(*i) || is_observation(*i) ) )
      discards.push_back(*i);
  }
  EUROPA::Entity::discardAll(discards);
}

bool DbCore::insert_copies() {
  for(EUROPA::TokenSet::const_iterator i=m_commited.begin(); m_commited.end()!=i; ++i) {
    size_t steps = 0;
    
    if( !insert_token(*i, steps) ) {
      m_reactor.log("core")<<"Failed to insert commtd value "
          <<(*i)->getPredicateName().toString();
      m_reactor.assembly().mark_invalid();
      return false;
    }
  }
  return true;
}

bool DbCore::relax(bool discard) {
  m_reactor.log("core")<<"Beginning database relax.";
  reset_observations();
  reset_goals(discard);
  reset_other_tokens(discard);
  if( insert_copies() )
    return true;
  m_reactor.log("core")<<"Relax failed.";
  return false;
}

bool DbCore::resolve_tokens(size_t &steps) {
  size_t last_count;

  do {
    EUROPA::TokenSet agenda = m_agenda;
    // m_reactor.log("core")<<agenda.size()<<" tokens in the agenda.";

    last_count = steps;

    if( !propagate() )
      return false;
    for(EUROPA::TokenSet::const_iterator i=agenda.begin(); 
	agenda.end()!=i; ++i) {
      if( !(*i)->getObject()->lastDomain().isSingleton() ) {
	// m_reactor.log("core")<<(*i)->toString()
	// 		     <<" skipped as its object is not a singleton :";
	EUROPA::ObjectDomain const &dom = (*i)->getObject()->lastDomain();
	std::list<EUROPA::ObjectId> objs = dom.makeObjectList();
	while( !objs.empty() ) {
	  m_reactor.log("core")<<"\t - "<<objs.front()->toString();
	  objs.pop_front();
	}
	continue;
      }
      if( !(*i)->isInactive() ) {
	// m_reactor.log("core")<<(*i)->toString()
	// 		     <<" skipped as it is not inactive.";
	continue;
      }
      EUROPA::TokenId candidate;
      if( !in_synch_scope(*i, candidate) ) {
	// m_reactor.log("core")<<(*i)->toString()
	// 		     <<" skipped as it is not in synchronization scope.";
	continue;
      }
      ++steps;
      if( !( resolve_token(*i, steps, candidate) && propagate() ) )
	return false;
    }      
  } while( last_count!=steps );
  return true;
}

bool DbCore::insert_default(EUROPA::ObjectId const &obj, EUROPA::TokenId &res, 
			    size_t &steps) {
  EUROPA::IntervalIntDomain now(m_reactor.getCurrentTick(),
				m_reactor.getCurrentTick());
  EUROPA::ConstrainedVariableId name = m_reactor.assembly().default_pred(obj);
  EUROPA::DataTypeId type = name->getDataType();
  std::string 
    short_pred = type->toString(name->getSpecifiedValue()),
    pred_name = obj->getType().toString()+"."+short_pred;
  EUROPA::DbClientId cli = m_reactor.assembly().plan_db()->getClient();
  res = cli->createToken(pred_name.c_str(), NULL, false);
  
  res->activate();
  res->start()->restrictBaseDomain(now);
  res->getObject()->specify(obj->getKey());
  return insert_token(res, steps);      
}

void DbCore::doNotify() {
  TREX::transaction::TICK cur = m_reactor.getCurrentTick();

  for(current_state_map::const_iterator i=m_internal_obs.begin();
      m_internal_obs.end()!=i; ++i) 
    if( i->second.isId() && i->second->start()->lastDomain().isMember(cur) )
      m_reactor.notify(i->first, i->second);
}

bool DbCore::complete_internals(size_t &steps) {
  if( m_reactor.assembly().invalid() )
    return false;
  TREX::transaction::TICK cur = m_reactor.getCurrentTick();
  EUROPA::IntervalIntDomain future(cur+1, PLUS_INFINITY);

  for(current_state_map::iterator i=m_internal_obs.begin(); m_internal_obs.end()!=i; ++i) {
    EUROPA::TimelineId tl = i->first;
    std::list<EUROPA::TokenId> const &tokens = tl->getTokenSequence();
    std::list<EUROPA::TokenId>::const_iterator it = tokens.begin();
    i->second = EUROPA::TokenId::noId();
    
    while( tokens.end()!=it ) {
      if( (*it)->start()->lastDomain().getLowerBound() > cur ) 
	break;
      if( (*it)->start()->lastDomain().getLowerBound()<=cur &&
	  (*it)->end()->lastDomain().getUpperBound()>cur ) {
	i->second = *it;
	break;
      }
      ++it;      
    }
    if( i->second.isNoId() ) {
      if( !insert_default(i->first, i->second, steps) )
	return false;
      // m_reactor.notify(i->first, i->second);
      continue;
    }
    if( i->second->start()->lastDomain().getUpperBound() < cur &&
	i->second->end()->lastDomain().getUpperBound() > cur ) 
      i->second->end()->restrictBaseDomain(future);      
    else {
      i->second->start()->specify(cur);
      // m_reactor.notify(i->first, i->second);
    }
    if( !propagate() )
      return false;      
  }
  return true;
}

bool DbCore::synchronize() {  
  size_t steps = 0;

  return propagate() && 
    resolve_tokens(steps) && 
    complete_internals(steps) && 
    resolve_tokens(steps);
}

void DbCore::ignore_token(EUROPA::TokenId const &token) {
  token->start()->deactivate();
  token->end()->deactivate();
  token->duration()->deactivate();
  
  std::vector<EUROPA::ConstrainedVariableId>::const_iterator 
    p = token->parameters().begin();
  for(; token->parameters().end()!=p; ++p)
    (*p)->deactivate();
  m_reactor.log("core")<<"Ignoring "<<token->toString();
}

void DbCore::add_to_agenda(EUROPA::TokenId const &token) {
  if( !token->isDiscarded() )
    if( m_agenda.insert(token).second )
      m_reactor.log("core")<<token->toString()<<" added to agenda";
}

void DbCore::remove_from_agenda(EUROPA::TokenId const &token) {
  if( m_agenda.erase(token) ) 
    m_reactor.log("core")<<token->toString()<<" removed from agenda";
}


// - callbacks 

void DbCore::notifyAdded(EUROPA::TokenId const &token) {
  m_reactor.log("core")<<"ADD "<<token->toString()
			  <<": "<<token->getPredicateName().toString();
  m_pending.insert(token);
}

void DbCore::notfiyRemoved(EUROPA::TokenId const &token) {
  m_reactor.log("core")<<"REMOVE "<<token->toString()
			  <<": "<<token->getPredicateName().toString();
  m_pending.erase(token);
  m_goals.erase(token);
  remove_from_agenda(token);
  m_reactor.removed(token);
  if( token->isCommitted() )
    m_commited.erase(token);
}

void DbCore::notifyActivated(EUROPA::TokenId const &token) {
  m_reactor.log("core")<<"ACTIVATE "<<token->toString()
			   <<": "<<token->getPredicateName().toString();
  remove_from_agenda(token);
}

void DbCore::notifyDeactivated(EUROPA::TokenId const &token) {
  m_reactor.log("core")<<"DEACTIVATE "<<token->toString()
			   <<": "<<token->getPredicateName().toString();
  remove_from_agenda(token);
}

void DbCore::notifyMerged(EUROPA::TokenId const &token) {
  m_reactor.log("core")<<"MERGE "<<token->toString()
			   <<": "<<token->getPredicateName().toString();
  remove_from_agenda(token);
}

void DbCore::notifySplit(EUROPA::TokenId const &token) {
  m_reactor.log("core")<<"SPLIT "<<token->toString()
			  <<": "<<token->getPredicateName().toString();
  add_to_agenda(token);
}

void DbCore::notifyCommitted(EUROPA::TokenId const &token) {
  m_reactor.log("core")<<"COMMIT "<<token->toString()
			  <<": "<<token->getPredicateName().toString();
  m_commited.insert(token);
}

void DbCore::notifyRejected(EUROPA::TokenId const &token) {
  m_reactor.log("core")<<"REJECT "<<token->toString()
			  <<": "<<token->getPredicateName().toString();
  remove_from_agenda(token);
}

void DbCore::notifyTerminated(EUROPA::TokenId const &token) {
  m_reactor.log("core")<<"TERMINATE "<<token->toString()
			  <<": "<<token->getPredicateName().toString();
}
