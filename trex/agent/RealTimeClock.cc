/** @file "RealTimeClock.cc"
 * @brief RealTimeClock class implementation
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
#include <cmath>
#include <boost/date_time/posix_time/posix_time.hpp>

#include "RealTimeClock.hh"

using namespace TREX::utils;
using namespace TREX::transaction;
using namespace TREX::agent;

namespace bt = boost::posix_time;


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

RealTimeClock::RealTimeClock(double secondsPerTick, double ratio) 
  :Clock(secondsPerTick/1000),
   m_started(false), m_tick(0), m_floatTick(secondsPerTick),
   m_freeRatio(ratio),
   m_secondsPerTick(to_timeval(secondsPerTick)),
   m_freeDuration(to_timeval(ratio*secondsPerTick)) {
  std::ostringstream oss;
  
  oss<<"[clock] Sleep ratio of "<<ratio;
  if( ratio*secondsPerTick <=0.0 ) {
    oss<<" resulted on a delay before sleep of 0 or less";
    throw Exception(oss.str());
  } else if( ratio>1.0 ) {
    oss<<" is greater than 1.0";
    throw Exception(oss.str());
  }
}

RealTimeClock::RealTimeClock(boost::property_tree::ptree::value_type &node)
  :Clock(0.001), m_started(false), m_tick(0),
   m_floatTick(parse_attr<double>(node, "tick")),
   m_freeRatio(parse_attr<double>(0.9, node, "tick")) {
  if( m_floatTick<=0.0 )
    throw Exception("[clock] Negative duration in tick attribute.");
  m_secondsPerTick = to_timeval(m_floatTick);
  double slp_s = m_freeRatio*m_floatTick;
  std::ostringstream oss;
  
  oss<<"[clock] Sleep ratio of "<<m_freeRatio;
  if( slp_s <=0.0 ) {
    oss<<" resulted on a delay before sleep of 0 or less";
    throw Exception(oss.str());
  } else if( m_freeRatio>1.0 ) {
    oss<<" is greater than 1.0";
    throw Exception(oss.str());
  }
  m_freeDuration = to_timeval(slp_s);
}

// modifiers :

void RealTimeClock::start() {
  getDate(m_nextTickDate);
  setNextTickDate();
  m_started = true;
}

void RealTimeClock::setNextTickDate(unsigned factor) {
  m_nextTickDate += m_secondsPerTick*factor;
  m_endFreeDate = m_nextTickDate + m_freeDuration;
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

TICK RealTimeClock::timeToTick(time_t secs, suseconds_t usecs) const {
  timeval tv;
  TICK current;
  double delta;

  tv.tv_sec = secs;
  tv.tv_usec = usecs;
  
  {
    mutex_type::scoped_lock guard(m_lock);
    delta = to_double(tv-m_nextTickDate);
    current = m_tick+1;
  }
  delta /= tickDuration();
  current += std::floor(delta+0.5); // do a rounding to reduce precision impact
  return current;
}

double RealTimeClock::tickToTime(TICK cur) const {
  TICK current;
  double next;
  {
    mutex_type::scoped_lock guard(m_lock);
    next = to_double(m_nextTickDate);
    current = m_tick+1;
  }  
  double d_tick = cur-current;
  d_tick *= tickDuration();

  return next+d_tick;
}

bool RealTimeClock::free() const {
  timeval tv;
  getDate(tv);
  return tv <= m_endFreeDate;
}

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
      if( (-delay)>0.05*tickDuration() ) {
        // If we are already late log it
        std::ostringstream oss;
        oss<<"Clock]["<<tick;
        m_log->syslog(oss.str())<<(-delay)
                                <<" secs late before sleep."
                                <<std::endl;
      }
      delay = 0.0;
    }
    return delay;
  } else 
    return Clock::getSleepDelay();
}
  
std::string RealTimeClock::date_str(TICK &tick) const {
  timeval tv;
  getDate(tv);
  bt::ptime date = bt::from_time_t(tv.tv_sec) + bt::microsec(tv.tv_usec);
  std::ostringstream oss;
  oss<<date;
  return oss.str();
}
