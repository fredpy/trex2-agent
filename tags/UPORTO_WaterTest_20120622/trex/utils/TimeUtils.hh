/* -*- C++ -*-
 */
/** @file "TimeUtils.hh"
 *
 * @brief C++ utilities for @c timeval
 *
 * This file defines some C++ operators to manipulate @c timeval
 * structure in a mor intuitive way.
 *
 * @author Frederic Py <fpy@mbari.org>
 * @ingroup utils
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
#ifndef H_TimeUtils
# define H_TimeUtils

# include <boost/chrono/duration.hpp>
# include <boost/date_time/posix_time/ptime.hpp>
# include <boost/config.hpp>


namespace TREX {
  namespace utils {
  
    template<class ChronoDuration>
    struct chrono_posix_convert;
  
    template<typename Rep, typename Period>
    struct chrono_posix_convert< boost::chrono::duration<Rep, Period> > {
      typedef boost::chrono::duration<Rep, Period> chrono_duration;
      typedef boost::posix_time::time_duration     posix_duration;

      typedef boost::chrono::nanoseconds ns_duration;
      
      static chrono_duration to_chrono(posix_duration const &pd) {
        return boost::chrono::duration_cast<chrono_duration>(ns_duration(pd.total_nanoseconds()));
      }
      
      static posix_duration to_posix(chrono_duration const &cd) {
        typename ns_duration::rep ns = boost::chrono::duration_cast<ns_duration>(cd).count(),
          secs = ns/1000000000l,
          nsecs = ns%1000000000l;
          
          return boost::posix_time::seconds(static_cast<long long>(secs))+
# ifdef BOOST_DATE_TIME_HAS_NANOSECONDS
            boost::posix_time::nanoseconds(nsecs);
# else 
            boost::posix_time::microseconds((nsecs+500)/1000);
#endif // BOOST_DATE_TIME_HAS_NANOSECONDS
      }
      
    }; // TREX::utils::chrono_posix_convert

  } // utils 
} // TREX

#endif // H_TimeUtils 
