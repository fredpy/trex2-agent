#include "Bind.hh"
#include <trex/europa/Assembly.hh>

using namespace TREX::europa;


/*
 * class starfish::europa::Bind
 */
Bind::Bind(EUROPA::LabelStr const &name, 
	   EUROPA::LabelStr const &propagatorName,
	   EUROPA::ConstraintEngineId const &cstrEngine,
	   std::vector<EUROPA::ConstrainedVariableId> const &variables)
  :EUROPA::Constraint(name, propagatorName, cstrEngine, variables) {
  if( m_variables.size()!=2 )
    throw TREX::europa::EuropaException("constraint "+name.toString()+" requires exactly 2 arguments");
  m_target = m_variables[0];
  m_default = m_variables[1];
  // Check that types are compatibles
  if( !m_target->getDataType()->isAssignableFrom(m_default->getDataType()) ) 
    throw EuropaException("constraint "+name.toString()
			  +" requires that the 2nd argument type is convertible to"
			  " the first argement type");
}

bool Bind::guardSatisfied() const {
  return m_target->isActive() && m_default->lastDomain().isSingleton() && 
    !m_target->lastDomain().isSingleton();
}

void Bind::handleExecute() {
  if( guardSatisfied() ) {
    EUROPA::Domain &domain = getCurrentDomain(m_target);
    debugMsg("trex:bind", "BEFORE "<<m_target->getName().toString()
	     <<"="<<m_target->toString()
	     <<" "<<m_default->toString());
    if( domain.intersects(m_default->lastDomain()) ) {
      domain.intersect(m_default->lastDomain());
    } else {
      EUROPA::edouble lb, ub, val = m_default->lastDomain().getSingletonValue();
      domain.getBounds(lb, ub);
      EUROPA::edouble dlo = std::abs(lb-val), dhi = std::abs(ub-val);
      if( dhi<dlo )
	domain.intersect(ub, ub);
      else
	domain.intersect(lb, lb);
    }
    debugMsg("trex:bind", "AFTER "<<m_target->getName().toString()
	     <<"="<<m_target->toString()
	     <<" "<<m_default->toString());
  }
}
