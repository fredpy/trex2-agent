/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, Frederic Py
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
#ifndef H_trex_utils_timing_asio_tick_timer
# define H_trex_utils_timing_asio_tick_timer

# include "../posix_utils.hh"

# include <trex/config/bits/asio_conf.hh>
# include <boost/asio/basic_deadline_timer.hpp>

namespace TREX {
  namespace utils {
    namespace internals {
      
      template<typename Clock>
      struct tick_clock_traits {
        typedef typename Clock::time_point time_type;
        typedef typename Clock::duration   duration_type;
        
        static time_type now() {
          return Clock::now();
        }
        static time_type add(time_type t, duration_type d) {
          return t + d;
        }
        static duration_type substract(time_type t1, time_type t2) {
          return t1 - t2;
        }
        static bool less_than(time_type t1, time_type t2) {
          return t1 < t2;
        }
        static boost::posix_time::time_duration
        to_posix_duration(duration_type d) {
          return chrono_posix_duration<duration_type>::to_posix(d);
        }
        
      }; // TREX::utils::internals::tick_clock_traits<>
      
    } // TREX::utils::internals
    
  } // TREX::utils
} // TREX


#endif // H_trex_utils_timing_asio_tick_timer

