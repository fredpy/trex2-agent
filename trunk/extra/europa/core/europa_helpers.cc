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
#include "trex/europa/bits/europa_helpers.hh"
#include "trex/europa/Assembly.hh"

#include <PLASMA/RuleInstance.hh>

using namespace TREX::europa;

void TREX::europa::details::restrict_base(EUROPA::TokenId const &tok, 
                                          EUROPA::ConstrainedVariableId const &var, 
                                          EUROPA::Domain const &dom) {
  if( tok->isMerged() ) {
    EUROPA::TokenId active = tok->getActiveToken();
    EUROPA::ConstrainedVariableId avar = active->getVariable(var->getName()); 
    
    size_t count=0;
    for(; !var->isActive(); ++count ) 
      var->undoDeactivation();
    var->restrictBaseDomain(dom);
    for(size_t i=0; i<count; ++i)
      var->deactivate();
    avar->handleBase(var->baseDomain());
  } else 
    var->restrictBaseDomain(dom);
}

void TREX::europa::details::restrict_bases(EUROPA::TokenId const &tok) {
  if( tok->isMerged() ) {    
    EUROPA::TokenId active = tok->getActiveToken();
    std::vector<EUROPA::ConstrainedVariableId> const &tvar = tok->getVariables();
    std::vector<EUROPA::ConstrainedVariableId> const &avar = active->getVariables();
    for(size_t i=1; i<tvar.size(); ++i) {
      size_t count=0;
      for(; !tvar[i]->isActive(); ++count ) 
        tvar[i]->undoDeactivation();
      tvar[i]->restrictBaseDomain(avar[i]->lastDomain());
      for(size_t j=0; j<count; ++j)
        tvar[i]->deactivate();
      avar[i]->handleBase(tvar[i]->baseDomain());
    }
  } else 
    tok->restrictBaseDomains();
}

void TREX::europa::details::restrict_attributes(EUROPA::TokenId const &tok) {
  EUROPA::TokenId active = tok;
  if( tok->isMerged() )
    active = tok->getActiveToken();
  
  std::vector<EUROPA::ConstrainedVariableId> const &tvar = tok->parameters();
  std::vector<EUROPA::ConstrainedVariableId> const &avar = active->parameters();
  for(size_t i=0; i<tvar.size(); ++i)
    restrict_base(tok, tvar[i], avar[i]->lastDomain());
}



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
    if( m_active.isId() ) {
      m_merged = m_active->getMergedTokens();
      for(EUROPA::TokenSet::const_iterator i=m_merged.begin(); m_merged.end()!=i; ++i)
        (*i)->cancel();      
    }
}

details::scoped_split::~scoped_split() {
  if( m_active.isId() ) {
//    EUROPA::TokenSet const &merged = m_active->getMergedTokens();
//    for(EUROPA::TokenSet::const_iterator i=merged.begin(); merged.end()!=i; ++i) {
//      (*i)->cancel(); (*i)->merge(m_active);
//    }
    for(EUROPA::TokenSet::const_iterator i=m_merged.begin(); m_merged.end()!=i; ++i)
      (*i)->merge(m_active);      
  }
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
