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
#include "location.hh"
#include <boost/date_time/posix_time/time_period.hpp>

using namespace mbari;

void location::update(location::date_type const &date, double north, double east) {
  point<2> former(m_last_pos);
  m_last_pos[0] = north;
  m_last_pos[1] = east;
  
  if( m_valid ) {
    long double dt = (date - m_date).total_nanoseconds();
    dt /= boost::posix_time::seconds(1).total_nanoseconds();
    
    m_speed = m_last_pos - former;
    m_speed /= dt;
    m_have_speed = true;
  } else {
    m_valid = true;
    m_have_speed = false;
  }
  m_date = date;
  m_valid = true;
}

point<2> location::position(location::date_type const &now, 
                            location::duration_type &delta_t, 
                            bool projected) const {
  point<2> estimate(m_last_pos);
  delta_t = now-m_date;
  if( projected && have_speed() ) {
    long double dt = delta_t.total_nanoseconds();
    dt /= boost::posix_time::seconds(1).total_nanoseconds();
    
    estimate += m_speed*dt;
  }
  return estimate;
}
