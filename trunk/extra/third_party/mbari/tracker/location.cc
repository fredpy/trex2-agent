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

point<2> location::speed() const {
  point<2> ret;
  ret[0] = m_speed*cosl(earth_point::to_rad(m_heading));
  ret[1] = m_speed*sinl(earth_point::to_rad(m_heading));
  return ret;
}

void location::update(location::date_type const &date, earth_point const &pos) {
  boost::optional<earth_point> pred = m_last_pos;
  m_last_pos = pos;
  
  if( pred ) {
    long double dt = (date-m_date).total_nanoseconds();
    dt /= boost::posix_time::seconds(1).total_nanoseconds();
    
    m_speed = pred->distance(pos, m_nav_calc);
    m_speed /= dt;
    
    m_heading = pred->bearing(pos, m_nav_calc);
    m_have_speed = true;
  } else
    m_have_speed = false;
  m_date = date;
}

earth_point location::position(location::date_type const &now, location::duration_type &delta,
                               bool projected) const {
  delta = now-m_date;
  if( projected && have_speed() ) {
    long double dt = delta.total_nanoseconds();
    dt /= boost::posix_time::seconds(1).total_nanoseconds();
    
    return m_last_pos->destination(m_heading, m_speed*dt, m_nav_calc);
  } else
    return *m_last_pos;
}
