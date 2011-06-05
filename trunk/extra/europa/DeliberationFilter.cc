#include "EuropaReactor.hh"

#include <PLASMA/RuleInstance.hh>
#include <PLASMA/ConstrainedVariable.hh>
#include <PLASMA/Token.hh>
#include <PLASMA/TokenVariable.hh>

using namespace TREX::europa;
using namespace TREX::transaction;

EuropaReactor *DeliberationFilter::s_current = NULL;

void DeliberationFilter::set_current(EuropaReactor *me) {
  s_current = me;
}

DeliberationFilter::DeliberationFilter(EUROPA::TiXmlElement const &cfg)
  :EUROPA::SOLVERS::FlawFilter(cfg), m_assembly(s_current->assembly()) {
  s_current->set_filter(this);
}

void DeliberationFilter::set_horizon(TICK start, TICK end) {
  m_horizon = EUROPA::IntervalIntDomain(start, end);
}

bool DeliberationFilter::test(EUROPA::EntityId const &entity) {
  EUROPA::TokenId token;

  if( EUROPA::ConstrainedVariableId::convertable(entity) ) {
    EUROPA::ConstrainedVariableId var = entity;

    // exclude singletons
    if( var->lastDomain().isSingleton() )
      return true;

    EUROPA::EntityId parent = var->parent();
    if( parent.isNoId() || EUROPA::ObjectId::convertable(parent) )
      return true;

    if( EUROPA::RuleInstanceId::convertable(parent) ) 
      token = EUROPA::RuleInstanceId(parent)->getToken();
    else 
      token = parent;
    
    if( !token->isActive() )
      return true;
  } else 
    token = entity;
  if( m_assembly.ignored(token) )
    return true;
  EUROPA::IntervalIntDomain const &start_time = token->start()->lastDomain();
  EUROPA::IntervalIntDomain const &end_time = token->end()->lastDomain();
  
  return  end_time.getUpperBound() < m_horizon.getLowerBound() ||
    start_time.getLowerBound() >= m_horizon.getUpperBound();
}

