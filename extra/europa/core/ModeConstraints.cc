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
#include "trex/europa/ModeConstraints.hh"
#include "trex/europa/Assembly.hh"
#include "trex/europa/bits/europa_helpers.hh"

#include <trex/utils/platform/memory.hh>

// include plasma header as system files in order to disable warnings
# define TREX_PP_SYSTEM_FILE <PLASMA/TokenVariable.hh>
# include <trex/europa/bits/system_header.hh>

using namespace TREX::europa;

/*
 * class TREX::europa::CheckInternal
 */
// structors 

CheckInternal::CheckInternal(EUROPA::LabelStr const &name, 
			     EUROPA::LabelStr const &propagatorName,
			     EUROPA::ConstraintEngineId const &cstrEngine,
			     std::vector<EUROPA::ConstrainedVariableId> const &variables)
  :ReactorConstraint(name, propagatorName, cstrEngine, variables) {
  if( m_variables.size()==2 ) {
    m_obj = &(getCurrentDomain(m_variables[1]));
  } else {
    if( m_variables.size()!=1 ) 
      throw EuropaException("Constraint "+name.toString()+
			    " accepts only 1 or 2 arguments");
    EUROPA::TokenId const &tok = details::parent_token(m_variables[0]);
    if( tok.isId() )
      m_obj = &(getCurrentDomain(tok->getObject()));
  }
  m_test = &(getCurrentDomain(m_variables[0]));
}

// manipulators

void CheckInternal::handleExecute() {
  if( NULL!=m_obj ) {
    Assembly &a = assembly();
    EUROPA::DataTypeId type = m_obj->getDataType();

    if( a.is_agent_timeline(type) ) {
      EUROPA::ObjectDomain const &dom(*m_obj);
      std::list<EUROPA::ObjectId> internals, 
	non_internals, vals = dom.makeObjectList();

      // Separate internal objects from the others
      for(std::list<EUROPA::ObjectId>::const_iterator o=vals.begin();
	  vals.end()!=o; ++o) {
	if( a.internal(*o) )
	  internals.push_back(*o);
	else 
	  non_internals.push_back(*o);
      }

      if( !internals.empty() ) {
	if( non_internals.empty() ) {
	  // Only internals
	  m_test->intersect(true, true);
	  return;
	} else if( m_test->isSingleton() ) {
	  // Build the new domain based on test value
	  UNIQ_PTR<EUROPA::ObjectDomain> tmp;
	  if( m_test->isMember(true) )
	    tmp.reset(new EUROPA::ObjectDomain(type, internals));
	  else 
	    tmp.reset(new EUROPA::ObjectDomain(type, non_internals));
	  // Restrict the domain of m_obj
	  m_obj->intersect(*tmp);
	  return;
	}
      }
    }
  }
  // If I am here then I knwo that te object is not internal
  m_test->intersect(false, false);
}

/*
 * class TREX::europa::CheckExternal
 */
// structors 

CheckExternal::CheckExternal(EUROPA::LabelStr const &name, 
			     EUROPA::LabelStr const &propagatorName,
			     EUROPA::ConstraintEngineId const &cstrEngine,
			     std::vector<EUROPA::ConstrainedVariableId> const &variables)
  :ReactorConstraint(name, propagatorName, cstrEngine, variables) {
  if( m_variables.size()==2 ) {
    m_obj = &(getCurrentDomain(m_variables[1]));
  } else {
    if( m_variables.size()!=1 ) 
      throw EuropaException("Constraint "+name.toString()+
			    " accepts only 1 or 2 arguments");
    EUROPA::TokenId const &tok = details::parent_token(m_variables[0]);
    if( tok.isId() )
      m_obj = &(getCurrentDomain(tok->getObject()));
  }
  m_test = &(getCurrentDomain(m_variables[0]));
}

// manipulators

void CheckExternal::handleExecute() {
  if( NULL!=m_obj ) {
    Assembly &a = assembly();
    EUROPA::DataTypeId type = m_obj->getDataType();

    if( a.is_agent_timeline(type) ) {
      EUROPA::ObjectDomain const &dom(*m_obj);
      std::list<EUROPA::ObjectId> externals, 
	non_externals, vals = dom.makeObjectList();

      // Separate external objects from the others
      for(std::list<EUROPA::ObjectId>::const_iterator o=vals.begin();
	  vals.end()!=o; ++o) {
	if( a.external(*o) )
	  externals.push_back(*o);
	else 
	  non_externals.push_back(*o);
      }

      if( !externals.empty() ) {
	if( non_externals.empty() ) {
	  // Only externals
	  m_test->intersect(true, true);
	  return;
	} else if( m_test->isSingleton() ) {
	  // Build the new domain based on test value
	  UNIQ_PTR<EUROPA::ObjectDomain> tmp;
	  if( m_test->isMember(true) )
	    tmp.reset(new EUROPA::ObjectDomain(type, externals));
	  else 
	    tmp.reset(new EUROPA::ObjectDomain(type, non_externals));
	  // Restrict the domain of m_obj
	  m_obj->intersect(*tmp);
	  return;
	}
      }
    }
  }
  // If I am here then I knwo that te object is not external
  m_test->intersect(false, false);
}
