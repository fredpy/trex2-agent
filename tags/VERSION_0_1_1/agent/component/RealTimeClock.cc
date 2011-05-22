/** @file "RealTimeClock.cc"
 * @brief RealTimeClock class implementation
 * @ingroup agent
 */
#include "RealTimeClock.hh"

using namespace TREX::utils;
using namespace TREX::transaction;
using namespace TREX::agent;

namespace {
  
  /** @brief Clock XML factor declaration for RealTimeClock
   * @relates TREX::agent::RealTimeClock
   *
   * This variable provides an access to the Clock XML factory
   * to allow automatic parsing of RealTimeClock from xml. The tag
   * associated to this is @c "RealTime"
   *
   * @sa TREX::transaction::Clock::xml_factory
   * @sa RealTimeClock(rapidxml::xml_node<> const &)
   * @ingroup agent
   */
  Clock::xml_factory::declare<RealTimeClock> rt_decl("RealTime");

} // ::

/*
 * class TREX::agent::RealTimeClock
 */
// statics :

void RealTimeClock::getDate(timeval &date) {
  gettimeofday(&date, NULL);
}

// structors :

RealTimeClock::RealTimeClock(double secondsPerTick) 
  :Clock(secondsPerTick/1000),
   m_started(false), m_tick(0), m_floatTick(secondsPerTick),
   m_secondsPerTick(to_timeval(secondsPerTick)) {}

RealTimeClock::RealTimeClock(rapidxml::xml_node<> const &node)
  :Clock(0.001), m_started(false), m_tick(0) {
  rapidxml::xml_attribute<> *tick = node.first_attribute("tick");
  if( NULL==tick ) 
    throw Exception("Missing tick attribute.");
  m_floatTick = string_cast<double>(std::string(tick->value(),
						tick->value_size()));
  if( m_floatTick<=0.0 )
    throw Exception("Negative duration in tick attribute.");
  m_secondsPerTick = to_timeval(m_floatTick);
}

// modifiers :

void RealTimeClock::start() {
  getDate(m_nextTickDate);
  setNextTickDate();
  m_started = true;
}

void RealTimeClock::setNextTickDate(unsigned factor) {
  m_nextTickDate += m_secondsPerTick*factor;
}

TICK RealTimeClock::getNextTick() {
  mutex_type::scoped_lock guard(m_lock);
  
  if( m_started ) {
    double howLate = -timeLeft();

    if( howLate>=0.0 ) {
      double ratio = howLate/m_floatTick;
      int tickIncr = 1+static_cast<int>(std::floor(ratio));
      
      m_tick += tickIncr;
      setNextTickDate(tickIncr);
      // If more than 10% late indicate the issue in the log
      if( ratio>=0.1 ) {
	std::ostringstream oss;
	oss<<"Clock]["<<m_tick;
	m_log->syslog(oss.str())<<howLate<<" secs late."<<std::endl;
      }
    }
  }
  return m_tick;
}

// observers :

double RealTimeClock::timeLeft() const {
  timeval tv;
  getDate(tv);
  return to_double(m_nextTickDate-tv);
}

double RealTimeClock::getSleepDelay() const {
  if( m_started ) {
    double delay;
    TICK tick;
    {
      mutex_type::scoped_lock guard(m_lock);
      delay = timeLeft();
      tick = m_tick;
    }
    if( delay<0.0 ) {
      // If we are already late log it
      std::ostringstream oss;
      oss<<"Clock]["<<m_tick;
      m_log->syslog(oss.str())<<(-delay)
			      <<" secs late before sleep."
			      <<std::endl;
      delay = 0.0;
    }
    return delay;
  } else 
    return Clock::getSleepDelay();
}
  
