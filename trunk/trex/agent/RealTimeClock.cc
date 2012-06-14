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

std::string RealTimeClock::date_str(TICK &tick) const {
  timeval tv;
  getDate(tv);
  bt::ptime date = bt::from_time_t(tv.tv_sec) + bt::microsec(tv.tv_usec);
  std::ostringstream oss;
  oss<<date;
  return oss.str();
}
*/