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

# include "platform/chrono.hh"

# include <boost/date_time/posix_time/ptime.hpp>
# include <boost/config.hpp>

# ifndef CPP11_HAS_CHRONO
#  include <boost/chrono/chrono_io.hpp>
# endif
# include <boost/date_time/posix_time/posix_time_io.hpp>


namespace TREX {
  namespace utils {
    
    namespace internals {
      
      template<class ChronDuration>
      struct chrono_posix_conv;
      
      template<typename Rep, class Period>
      struct chrono_posix_conv< CHRONO::duration<Rep, Period> > {
        typedef CHRONO::duration<Rep, Period>    chrono_duration;
        typedef boost::posix_time::time_duration posix_duration;
        
        static chrono_duration to_chrono(posix_duration const &pd) {
          chrono_duration
            result = CHRONO::duration_cast<chrono_duration>(CHRONO::hours(pd.hours()));
          result += CHRONO::duration_cast<chrono_duration>(CHRONO::minutes(pd.minutes()));
          result += CHRONO::duration_cast<chrono_duration>(CHRONO::seconds(pd.seconds()));
          
          long long cpt_s = pd.fractional_seconds(), // remaining seconds
            max_s = pd.ticks_per_second(); // posix clock subseconds accuracy

          if( 1000000000L != max_s ) // if clock is not nanosecond accurate
            cpt_s = (cpt_s*1000000000L)/max_s; // convert cpt_s in ns

          CHRONO::nanoseconds ns(cpt_s);
          return result+CHRONO::duration_cast<chrono_duration>(ns);
        }
        
        static posix_duration  to_posix(chrono_duration const &cd) {
          CHRONO::hours h = CHRONO::duration_cast<CHRONO::hours>(cd);
          posix_duration result = boost::posix_time::hours(h.count());
          typename CHRONO_NS::common_type<chrono_duration, CHRONO::hours>::type sub_hours = cd-h;
          
          CHRONO::seconds secs = CHRONO::duration_cast<CHRONO::seconds>(sub_hours);
          result += boost::posix_time::seconds(secs.count());
          
          CHRONO::nanoseconds nsecs = CHRONO::duration_cast<CHRONO::nanoseconds>(sub_hours-secs);
# ifdef BOOST_DATE_TIME_HAS_NANOSECONDS
          result += boost::posix_time::nanoseconds(nsecs.count());
# else
          result += boost::posix_time::microseconds((nsecs.count()+500)/1000);
# endif
          return result;
        }
        
      };
      
      
    } // TREX::utils::internals
    

    struct rt_duration {
      typedef boost::posix_time::time_duration base_type;
      
      rt_duration() {}
      rt_duration(base_type const &val):value(val) {}
      
      template<class ChronoDuration>
      explicit rt_duration(ChronoDuration const &dur)
      :value(internals::chrono_posix_conv<ChronoDuration>::to_posix(dur)) {}
      
      ~rt_duration() {}
      
      template<class ChronoDuration>
      ChronoDuration to_chrono() const {
        return internals::chrono_posix_conv<ChronoDuration>::to_chrono(value);
      }
      
      base_type value;
    };

    std::ostream &operator<<(std::ostream &out, rt_duration const &dur);
    std::istream &operator>>(std::istream &in, rt_duration &dur);

    struct rt_date {
      typedef boost::posix_time::ptime base_type;
      typedef rt_duration              duration_type;

      static  rt_date const &epoch();
      static  rt_date const &max();
      
      rt_date() {}
      rt_date(base_type const &val):value(val) {}
      ~rt_date() {}
      
      inline duration_type since(rt_date const &other) const {
        return value-other.value;
      }
      inline duration_type since_epoch() const {
        return since(epoch());
      }
      inline rt_date add(duration_type const &delta) const {
        return value+delta.value;
      }
      
      base_type value;
    };
    
    inline rt_duration operator-(rt_date const &a, rt_date const &b) {
      return a.since(b);
    }
    inline rt_date operator+(rt_date const &a, rt_duration const &b) {
      return a.add(b);
    }

    std::ostream &operator<<(std::ostream &out, rt_date const &date);
    std::istream &operator>>(std::istream &in, rt_date &date);
    

  } // utils 
} // TREX

#endif // H_TimeUtils
