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
#include "bits/europa_helpers.hh"
#include "Assembly.hh"

#include <PLASMA/RuleInstance.hh>

using namespace TREX::europa;

EUROPA::TokenId TREX::europa::details::parent_token(EUROPA::ConstrainedVariableId const &var) {
  EUROPA::EntityId ent = var->parent();
  
  if( ent.isId() ) {
    
    if( EUROPA::TokenId::convertable(ent) )
      return ent;
    if( EUROPA::RuleInstanceId::convertable(ent) ) {
      EUROPA::RuleInstanceId rule(ent);
      return rule->getToken();
    }
  }
  // No parent token
  return EUROPA::TokenId::noId();
}

Assembly &TREX::europa::details::assembly_of(EUROPA::EngineComponentId const &component) {
  EUROPA::EngineId engine = component->getEngine();

  if( !EUROPA::Id<Assembly>::convertable(engine) )
    throw EuropaException("Component's engine is not a TREX assembly.");
  return *(EUROPA::Id<Assembly>(engine));
}

std::string TREX::europa::details::predicate_name(EUROPA::ObjectId const &obj, 
						  std::string const &predicate) {
  return obj->getType().toString()+"."+predicate;
}

/*
 * class TREX::europa::details::scoped_split
 */

// structors 

details::scoped_split::scoped_split(EUROPA::TokenId const &token)
  :m_token(token), m_active(token->getActiveToken()) {
  if( m_active.isId() )
    m_token->cancel();
}

details::scoped_split::~scoped_split() {
  if( m_active.isId() ) 
    m_token->merge(m_active);
}

// observers

EUROPA::TokenId details::scoped_split::active() const {
  if( m_active.isId() )
    return m_active;
  else 
    return m_token;
}
	
EUROPA::Token *details::scoped_split::operator->() const {
  return m_token.operator->();
}
