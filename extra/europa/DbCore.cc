#include "EuropaReactor.hh"

# include <PLASMA/Token.hh>
# include <PLASMA/TokenVariable.hh>

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

void DbCore::notify(EUROPA::TokenId const &tok) {
  m_observations.insert(tok);
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
