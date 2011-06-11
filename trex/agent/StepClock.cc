/** @file "StepClock.cc"
 * @brief StepClock class implementation
 * 
 * @ingroup agent
 */
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
#include "StepClock.hh"

using namespace TREX::transaction;
using namespace TREX::agent;
using namespace TREX::utils;

namespace {
  /** @brief Clock XML factor declaration for StepClock
   * @relates TREX::agent::StepClock
   *
   * This variable provides an access to the Clock XML factory
   * to allow automatic parsing of StepClock from xml. The tag
   * associated to this is @c "Step"
   *
   * @sa TREX::transaction::Clock::xml_factory
   * @sa StepClock(rapidxml::xml_node<> const &)
   * @ingroup agent
   */
  Clock::xml_factory::declare<StepClock> decl("StepClock");

} // ::

/*
 * class TREX::agent::StepClock
 */
// statics

SingletonUse<LogManager> StepClock::m_log;

unsigned int StepClock::selectStep(unsigned int stepsPerTick) {
  if( stepsPerTick<=0 ) {
    m_log->syslog("Clock")<<"Requested an invalid number of steps per tick ("
			  <<stepsPerTick<<").\n\tSetting it to 50.\n";
    stepsPerTick = 50;
  }
  return stepsPerTick;
}

unsigned int StepClock::parseStep(rapidxml::xml_attribute<> *steps) {
  if( NULL==steps ) {
    m_log->syslog("Clock")<<"missing steps attribute in XML definition.\n"
			  <<"\tSetting it to 50.\n";
    return 50;
  } else 
    return selectStep(string_cast<unsigned int>(std::string(steps->value(),
							    steps->value_size())));
}

// structors :

StepClock::StepClock(double sleepSeconds, unsigned int stepsPerTick) 
  :Clock(sleepSeconds), m_tick(0), m_currentStep(0), 
   m_stepsPerTickDefault(selectStep(stepsPerTick)) {
  m_stepsPerTick = m_stepsPerTickDefault;
}

StepClock::StepClock(rapidxml::xml_node<> const &node) 
  :Clock(0.0), m_tick(0), m_currentStep(0),
   m_stepsPerTickDefault(parseStep(node.first_attribute("steps"))) {}

// modifiers :

TICK StepClock::getNextTick() {
  if( m_currentStep>=m_stepsPerTick ) {
    Clock::advanceTick(m_tick);
    m_stepsPerTick = m_stepsPerTickDefault;
    m_currentStep = 0;
  } else 
    ++m_currentStep;
  return m_tick;
}

// observers :

void StepClock::setMaxSteps(unsigned int nSteps) const {
  if( nSteps>0 ) {
    m_stepsPerTick = nSteps;
    m_currentStep = 0;
  }
}
