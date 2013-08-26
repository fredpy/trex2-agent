/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, MBARI.
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
#include "TimeUtils.hh"

#include <boost/date_time/local_time/local_time.hpp>
#include <boost/io/ios_state.hpp>

using namespace TREX::utils;
namespace posix=boost::posix_time;
namespace greg=boost::gregorian;

/*
 * class TREX::utils::rt_duration
 */

// friends, etc

std::ostream &TREX::utils::operator<<(std::ostream &out, rt_duration const &dur) {
  return out<<dur.value;
}

std::istream &TREX::utils::operator>>(std::istream &in, rt_duration &dur) {  
  return in>>dur.value;
}

  /*
 * class TREX::utils::rt_date
 */

// statics

rt_date const &rt_date::epoch() {
  static rt_date const value(posix::from_time_t(0));
  return value;
}

rt_date const &rt_date::max() {
  static rt_date const value(posix::max_date_time);
  return value;
}


// friends, etc

std::ostream &TREX::utils::operator<<(std::ostream &out, rt_date const &date) {
  return out<<posix::to_iso_extended_string(date.value);
}

std::istream &TREX::utils::operator>>(std::istream &in, rt_date &date) {
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
    date.value = time;
  return in;  
}

  
//
//  
//  
//  typedef boost::posix_time::time_input_facet facet;
//  rt_date::base_type tmp;
//  {
//    scoped_locale loc(in, std::locale(in.getloc(), new facet("%Y-%m-%dT%H:%M:%S%F")));
//    in>>tmp;
//  }
//  
//  if( in ) {
//    std::istream::int_type nxt = in.peek();
//    std::cerr<<"base_date: "<<rt_date(tmp)<<", next:"<<char(nxt)<<std::endl;
//    
//
//    if( 'Z'==nxt ) {
//      std::cerr<<"Found Z"<<std::endl;
//      in.get();
//    } else if( in.good() && !std::isspace(nxt) ) {
//      rt_duration tz;
//      std::cerr<<"reading tz"<<std::endl;
//      if( (in>>tz) ) {
//        std::cerr<<"shifting for: "<<tz<<std::endl;
//        tmp -= tz.value;
//      }
//    }
//  }
//  if( in ) {
//    date.value = tmp;
//    std::cerr<<"result: "<<date<<std::endl;
//  } else
//    std::cerr<<"error"<<std::endl;
//  return in;
//}
