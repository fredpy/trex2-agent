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
#ifndef H_mbari_location
# define H_mbari_location


# include <boost/date_time/posix_time/ptime.hpp>
# include <mbari/shared/earth_point.hh>
# include "point.hh"

# include <boost/optional.hpp>

namespace mbari {

  class location {
  public:
    typedef boost::posix_time::ptime         date_type;
    typedef boost::posix_time::time_duration duration_type;
    
    location():m_have_speed(false) {}
    
    bool is_valid() const {
      return m_last_pos;
    }
    bool have_speed() const {
      return m_have_speed;
    }
    double heading() const {
      return m_heading;
    }
    long double lin_speed() const {
      return m_speed;
    }
    point<2> speed() const;

    void update(date_type const &date, earth_point const &pos);
    earth_point position(date_type const &now, duration_type &delta,
                         bool projected=true) const;
  
    
  private:
    date_type     m_date;
    mutable rhumb_lines m_nav_calc;
    
    bool m_have_speed;
    boost::optional<earth_point> m_last_pos;
    
    double m_heading;
    long double m_speed;
  };
  
} // mbari

#endif 
