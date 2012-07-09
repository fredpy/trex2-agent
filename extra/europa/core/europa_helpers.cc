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
#include <PLASMA/TokenVariable.hh>

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
    avar->handleBase(var->baseDomain());
    if( var->isSpecified() )
      avar->handleSpecified(var->lastDomain().getSingletonValue());
    for(size_t i=0; i<count; ++i)
      var->deactivate();
  } else 
    var->restrictBaseDomain(dom);
}

void TREX::europa::details::restrict_bases(EUROPA::TokenId const &dest, 
                                           EUROPA::TokenId const &src) {
  std::vector<EUROPA::ConstrainedVariableId> const &dest_v = dest->getVariables();
  std::vector<EUROPA::ConstrainedVariableId> const &src_v = src->getVariables();
  EUROPA::TokenId active = dest;
  if( dest->isMerged() )
    active = dest->getActiveToken();
  
  for(size_t i=1; i<dest_v.size(); ++i) {
    size_t count = 0;
    for(; !dest_v[i]->isActive(); ++count)
      dest_v[i]->undoDeactivation();
    dest_v[i]->restrictBaseDomain(src_v[i]->lastDomain());
    if( active!=dest ) {
      active->getVariables()[i]->handleBase(dest_v[i]->baseDomain());
      if( dest->getVariables()[i]->isSpecified() )
	active->getVariables()[i]->handleSpecified(dest->getVariables()[i]->lastDomain().getSingletonValue());
    }
    for(size_t j=0; j<count; ++j)
      dest_v[i]->deactivate();
  }
}

void TREX::europa::details::restrict_bases(EUROPA::TokenId const &tok) {
  if( tok->isMerged() )
    restrict_bases(tok, tok->getActiveToken());
  else 
    tok->restrictBaseDomains();
}

void TREX::europa::details::restrict_attributes(EUROPA::TokenId const &tok, EUROPA::TokenId const &other) {
  std::vector<EUROPA::ConstrainedVariableId> const &tvar = tok->parameters();
  std::vector<EUROPA::ConstrainedVariableId> const &avar = other->parameters();
  for(size_t i=0; i<tvar.size(); ++i)
    restrict_base(tok, tvar[i], avar[i]->lastDomain());
}

void TREX::europa::details::restrict_attributes(EUROPA::TokenId const &tok) {
  EUROPA::TokenId active = tok;
  if( tok->isMerged() )
    active = tok->getActiveToken();
  restrict_attributes(tok, active);
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

std::ostream &TREX::europa::details::var_print(std::ostream &out, EUROPA::ConstrainedVariableId const &var) {
  EUROPA::TokenId tok = parent_token(var);
  if( tok.isId() )
    out<<tok->getPredicateName().toString()<<'('<<tok->getKey()<<").";
  return out<<var->getName().toString()<<'('<<var->getKey()<<')';
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

/*
 * struct TREX::europa::details::is_rejectable
 */

bool details::is_rejectable::operator()(EUROPA::TokenId const &tok) const {
  return tok->getState()->baseDomain().isMember(EUROPA::Token::REJECTED);
}
