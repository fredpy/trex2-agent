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
#include "Constraints.hh"

#include <PLASMA/RuleInstance.hh>
#include <PLASMA/Token.hh>
#include <PLASMA/TokenVariable.hh>

EUROPA::TokenId TREX::europa::details::parent_token(EUROPA::ConstrainedVariableId const &var) {
  EUROPA::EntityId e = var->parent();

  if( e.isId() ) {
    if( EUROPA::TokenId::convertable(e) )
      return e;
    
    if( EUROPA::RuleInstanceId::convertable(e) ) {
      EUROPA::RuleInstanceId r(e);
      return r->getToken();
    }
  }
  return EUROPA::TokenId::noId();
}


using namespace TREX::europa;


/*
 * class TREX::europa::ReactorConstraint
 */

ReactorConstraint::ReactorConstraint(EUROPA::LabelStr const &name, 
				     EUROPA::LabelStr const &propagatorName,
				     EUROPA::ConstraintEngineId const &cstrEngine,
				     std::vector<EUROPA::ConstrainedVariableId> const &vars)
  :EUROPA::Constraint(name, propagatorName, cstrEngine, vars), m_assembly(NULL) {
}

Assembly &ReactorConstraint::assembly() {
  if( NULL==m_assembly ) {
    EUROPA::PropagatorId prop = getPropagator();
    if( EUROPA::Id<ReactorPropagator>::convertable(prop) ) {
      EUROPA::Id<ReactorPropagator> me(prop);
      m_assembly = me->m_assembly;
    } else 
      throw EuropaException("Constraint "+getName().toString()+" is attached to "
			    +getPropagator()->getName().toString()
			    +" which is not a reactor propagator");
  }
  return *m_assembly;
}


/*
 * class TREX::europa::CheckExternal
 */
CheckExternal::CheckExternal(EUROPA::LabelStr const &name, 
			     EUROPA::LabelStr const &propagatorName,
			     EUROPA::ConstraintEngineId const &cstrEngine,
			     std::vector<EUROPA::ConstrainedVariableId> const &variables) 
  :ReactorConstraint(name, propagatorName, cstrEngine, variables), m_obj(NULL) {
  if( m_variables.size()==2 ) {
    m_obj = &(getCurrentDomain(m_variables[1]));
  } else {
    if( m_variables.size()!=1 )
      throw EuropaException("Constraint "+name.toString()+" require 1 or 2 arguments");
    EUROPA::TokenId tok = details::parent_token(m_variables[0]);
    if( tok.isId() ) {
      m_obj = &(getCurrentDomain(tok->getObject()));
    }
  }
  m_test = &(getCurrentDomain(m_variables[0]));
}

void CheckExternal::handleExecute() {
  if( NULL!=m_obj ) {
    Assembly &r = assembly();
    EUROPA::DataTypeId type = m_obj->getDataType();

    if( r.schema()->isA(type->getName(), Assembly::TREX_TIMELINE) ) {      
      EUROPA::ObjectDomain const &dom(*m_obj);
      std::list<EUROPA::ObjectId> exts, n_exts, vals = dom.makeObjectList();

      for(std::list<EUROPA::ObjectId>::const_iterator i=vals.begin(); vals.end()!=i; ++i) {
	if( r.isExternal(*i) )
	  exts.push_back(*i);
	else 
	  n_exts.push_back(*i);
      }
      if( !exts.empty() ) {
	if( n_exts.empty() ) {
	  m_test->intersect(true, true);
	} else if( m_test->isSingleton() ) {
	  std::auto_ptr<EUROPA::ObjectDomain> tmp;
	  if( m_test->isMember(true) )
	    tmp.reset(new EUROPA::ObjectDomain(type, exts));
	  else 
	    tmp.reset(new EUROPA::ObjectDomain(type, n_exts));
	  m_obj->intersect(*tmp);
	}
      }
    }
  }
  m_test->intersect(false, false);
}

/*
 * class TREX::europa::CheckInternal
 */
CheckInternal::CheckInternal(EUROPA::LabelStr const &name, 
			     EUROPA::LabelStr const &propagatorName,
			     EUROPA::ConstraintEngineId const &cstrEngine,
			     std::vector<EUROPA::ConstrainedVariableId> const &variables) 
  :ReactorConstraint(name, propagatorName, cstrEngine, variables) {
  if( m_variables.size()==2 ) {
    m_obj = &(getCurrentDomain(m_variables[1]));
  } else {
    if( m_variables.size()!=1 )
      throw EuropaException("Constraint "+name.toString()+" require 1 or 2 arguments");
    EUROPA::TokenId tok = details::parent_token(m_variables[0]);
    if( tok.isId() ) {
      m_obj = &(getCurrentDomain(tok->getObject()));
    }
  }
  m_test = &(getCurrentDomain(m_variables[0]));
}

void CheckInternal::handleExecute() {
  if( NULL!=m_obj ) {
    Assembly &r = assembly();
    EUROPA::DataTypeId type = m_obj->getDataType();

    if( r.schema()->isA(type->getName(), Assembly::TREX_TIMELINE) ) {      
      EUROPA::ObjectDomain const &dom(*m_obj);
      std::list<EUROPA::ObjectId> ints, n_ints, vals = dom.makeObjectList();

      for(std::list<EUROPA::ObjectId>::const_iterator i=vals.begin(); vals.end()!=i; ++i) {
	if( r.isInternal(*i) )
	  ints.push_back(*i);
	else 
	  n_ints.push_back(*i);
      }
      if( !ints.empty() ) {
	if( n_ints.empty() ) {
	  m_test->intersect(true, true);
	} else if( m_test->isSingleton() ) {
	  std::auto_ptr<EUROPA::ObjectDomain> tmp;
	  if( m_test->isMember(true) )
	    tmp.reset(new EUROPA::ObjectDomain(type, ints));
	  else 
	    tmp.reset(new EUROPA::ObjectDomain(type, n_ints));
	  m_obj->intersect(*tmp);
	}
      }
    }
  }
  m_test->intersect(false, false);
}

/*
 * class TREX::europa::Bind
 */
Bind::Bind(EUROPA::LabelStr const &name, 
	   EUROPA::LabelStr const &propagatorName,
	   EUROPA::ConstraintEngineId const &cstrEngine,
	   std::vector<EUROPA::ConstrainedVariableId> const &variables)
  :EUROPA::Constraint(name, propagatorName, cstrEngine, variables) {
  if( m_variables.size()!=2 )
    throw EuropaException("constraint "+name.toString()+" requires exactly 2 arguments");
  m_target = &(getCurrentDomain(m_variables[0]));
  m_default = &(getCurrentDomain(m_variables[1]));
  // Check that types are compatibles
  if( !m_target->getDataType()->isAssignableFrom(m_default->getDataType()) ) 
    throw EuropaException("constraint "+name.toString()
			  +" requires that the 2nd argument type is convertible to"
			  " the first argement type");
}

bool Bind::guardSatisfied() const {
  return m_default->isSingleton() && !m_target->isSingleton();
}

void Bind::handleExecute() {
  if( guardSatisfied() ) {
    if( m_target->intersects(*m_default) )
      m_target->intersect(*m_default);
    else {
      EUROPA::edouble lb, ub, val = m_default->getSingletonValue();
      m_target->getBounds(lb, ub);
      EUROPA::edouble dlo = std::abs(lb-val), dhi = std::abs(ub-val);
      if( dhi<dlo )
	m_target->intersect(ub, ub);
      else
	m_target->intersect(lb, lb);
    }
  }
}
