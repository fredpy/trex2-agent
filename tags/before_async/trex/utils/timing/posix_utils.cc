/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, Frederic Py.
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
#include "posix_utils.hh"

#include <boost/date_time/local_time/local_time.hpp>
#include <boost/io/ios_state.hpp>

using namespace TREX::utils;
namespace posix=boost::posix_time;

// rt_duration

std::ostream &TREX::utils::operator<<(std::ostream &out, rt_duration const &d) {
  return out<<d.value;
}

std::istream &TREX::utils::operator>>(std::istream &in, rt_duration &d) {
  return in>>d.value;
}

// rt_date

std::ostream &TREX::utils::operator<<(std::ostream &out, rt_date const &d) {
  return out<<posix::to_iso_extended_string(d.value);
}

std::istream &TREX::utils::operator>>(std::istream &in, rt_date &d) {
  typedef boost::posix_time::time_input_facet facet;
  boost::posix_time::ptime time(boost::posix_time::not_a_date_time);
  boost::posix_time::time_duration dt(0,0,0);

  // Unluckily boost do not parse time zones as input (eg +07:00) as
  // such format is not enough to fully identify the timezone.
  //
  // We handle it with a simple and non robust peeking on the next char
  // although this may put us in trouble as we do not rollback the stream
  // perfectly if the error occurs on the time zone
  facet *f = new facet("%Y-%m-%dT%H:%M:%S%F");
  f->time_duration_format("%H:%M");
  {
    boost::io::ios_locale_saver save(in);
    in.imbue(std::locale(in.getloc(), f));
   
    in>>time;
    std::istream::int_type val = in.peek();

    if( 'Z'==val ) {
      in.get();
    } else if( '+'==val || '-'==val ) {
      bool neg = ('-'==in.get());
      
      in>>dt;
      if( neg )
        time += dt;
      else
        time -= dt;
    }
  }
  if( !!in )
    d.value = time;
  return in;
}

rt_date const &rt_date::epoch() {
  static rt_date const value(posix::from_time_t(0));
  return value;
}

rt_date const &rt_date::max() {
  static rt_date const value(posix::max_date_time);
  return value;
}

rt_date::rt_date():value(posix::from_time_t(0)) {}

