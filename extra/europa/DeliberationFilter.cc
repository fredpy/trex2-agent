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
#include "EuropaReactor.hh"

#include <PLASMA/RuleInstance.hh>
#include <PLASMA/ConstrainedVariable.hh>
#include <PLASMA/Token.hh>
#include <PLASMA/TokenVariable.hh>

#include <PLASMA/Debug.hh>

using namespace TREX::europa;
using namespace TREX::transaction;

EuropaReactor *DeliberationFilter::s_current = NULL;

void DeliberationFilter::set_current(EuropaReactor *me) {
  s_current = me;
}

DeliberationFilter::DeliberationFilter(EUROPA::TiXmlElement const &cfg)
  :EUROPA::SOLVERS::FlawFilter(cfg, true), m_assembly(s_current->assembly()) {
  // TREX::utils::SingletonUse<details::Schema> schema;
  debugMsg("trex:horizon", "Created for reactor "<<s_current->getName()
	   <<"\nid is "<<getId());
  setExpression(toString()+":horizon_"+s_current->getName().str());
  s_current->set_filter(this);  
}

void DeliberationFilter::set_horizon(TICK start, TICK end) {
  debugMsg("trex:horizon", "horizon set to ["<<start<<", "<<end<<")");;
  m_horizon = EUROPA::IntervalIntDomain(start, end);
}

bool DeliberationFilter::test(EUROPA::EntityId const &entity) {
  EUROPA::TokenId token;

  debugMsg("trex:horizon", "check for "<<entity->getKey());
  // std::cerr<<"Filtering "<<entity->toString()<<std::endl;
  if( EUROPA::ConstrainedVariableId::convertable(entity) ) {
    EUROPA::ConstrainedVariableId var = entity;

    // exclude singletons
    if( var->lastDomain().isSingleton() ) {
      debugMsg("trex:horizon", "excluding singleton "<<var->getName().toString());
      return true;
    }

    EUROPA::EntityId parent = var->parent();
    if( parent.isNoId() || EUROPA::ObjectId::convertable(parent) ) {
      debugMsg("trex:horizon", "global variable "<<var->getName().toString());
      return true;
    }

    if( EUROPA::RuleInstanceId::convertable(parent) ) 
      token = EUROPA::RuleInstanceId(parent)->getToken();
    else 
      token = parent;
    
    if( m_assembly.ignored(token) ) {
      debugMsg("trex:horizon", "excluding ignored token "<<token->toString());
      return true;
    }
    if( !token->isActive() ) {
      debugMsg("trex:horizon", "excluding inactive token "<<token->toString());
      return true;
    } 
  } else 
    token = entity;
  if( m_assembly.ignored(token) ) {
    debugMsg("trex:horizon", "excluding ignored token "<<token->toString());
    return true;
  }
  EUROPA::IntervalIntDomain const &start_time = token->start()->lastDomain();
  EUROPA::IntervalIntDomain const &end_time = token->end()->lastDomain();
  
  if( end_time.getUpperBound() < m_horizon.getLowerBound() ||
      start_time.getLowerBound() >= m_horizon.getUpperBound() ) {
    debugMsg("trex:horizon", "excluding out of window token "<<token->toString());
    return true;
  }
  return false;
}

