/** @file "StepClock.cc"
 * @brief StepClock class implementation
 * 
 * @ingroup agent
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
