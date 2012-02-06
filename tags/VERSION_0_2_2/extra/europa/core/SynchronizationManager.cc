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
 *     notice, this list of conditions and the following disclaimer.s
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
#include "SynchronizationManager.hh"
#include "Assembly.hh"
#include "bits/europa_helpers.hh"
#include "private/UpdateFlawIterator.hh"
#include "private/CurrentState.hh"

#include <PLASMA/Timeline.hh>
#include <PLASMA/Token.hh>
#include <PLASMA/TokenVariable.hh>

using namespace TREX::europa;

/*
 * class TREX::europa::details::UpdateFlawIterator
 */

// structors

details::UpdateFlawIterator::UpdateFlawIterator(SynchronizationManager &manager) 
  :EUROPA::SOLVERS::FlawIterator(manager), m_assembly(*(manager.m_assembly)), 
   m_it(manager.m_assembly->begin()) {
  advance();
}

// modifiers

EUROPA::EntityId const details::UpdateFlawIterator::nextCandidate() {
  // Find the next candidate
  for(;m_assembly.end()!=m_it; ++m_it) {
    // Filter out all the agent timelines that are not internal
    if( m_assembly.internal(**m_it) ) {
      EUROPA::EntityId candidate = *m_it;
      ++m_it;
      return candidate;
    }
  }
  // No candidate found
  return EUROPA::EntityId::noId();
}

/*
 * class TREX::europa::SynchronizationManager
 */
// structors 

SynchronizationManager::SynchronizationManager(EUROPA::TiXmlElement const &cfg)
  :EUROPA::SOLVERS::FlawManager(cfg) {}

// manipulators

void SynchronizationManager::handleInitialize() {
  // Extract the TREX assembly I am connected to 
  m_assembly = &details::assembly_of(getPlanDatabase());
}

bool SynchronizationManager::staticMatch(EUROPA::EntityId const &entity) {
  return !details::CurrentStateId::convertable(entity);
}

bool SynchronizationManager::dynamicMatch(EUROPA::EntityId const &entity) {
  if( staticMatch(entity) )
    return true;
  else {
    details::CurrentStateId state(entity);
    return state->identified();
  }
}

EUROPA::IteratorId SynchronizationManager::createIterator() {
  return (new details::UpdateFlawIterator(*this))->getId();
}

// observers

std::string SynchronizationManager::toString(EUROPA::EntityId const &entity) const {
  details::CurrentStateId update(entity);
  std::ostringstream oss;
  
  oss<<"trex.STATE["<<update->now()<<"]: "<<update->timeline()->toString();
  return oss.str();
}
