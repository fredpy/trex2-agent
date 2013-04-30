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

# include <boost/chrono/chrono_io.hpp>
# include <boost/date_time/posix_time/posix_time_io.hpp>


namespace TREX {
  namespace utils {
  
# ifndef DOXYGEN

    template<class ChronoDuration>
    struct chrono_posix_convert;
  
    template<typename Rep, typename Period>
    struct chrono_posix_convert< CHRONO::duration<Rep, Period> > {
      typedef CHRONO::duration<Rep, Period> chrono_duration;
      typedef boost::posix_time::time_duration     posix_duration;
      
      

      typedef CHRONO::nanoseconds ns_duration;
      
      static chrono_duration to_chrono(posix_duration const &pd) {
        chrono_duration
        result = CHRONO::duration_cast<chrono_duration>(CHRONO::hours(pd.hours()));
        
        result += CHRONO::duration_cast<chrono_duration>(CHRONO::minutes(pd.minutes()));
        
        result += CHRONO::duration_cast<chrono_duration>(CHRONO::seconds(pd.seconds()));
        
        long long cpt_s = pd.fractional_seconds(),
          max_s = pd.ticks_per_second();
      
        if( 1000000000L != max_s )
          cpt_s = (cpt_s*1000000000L)/max_s;
        
        CHRONO::nanoseconds ns(cpt_s);
        
        return result+CHRONO::duration_cast<chrono_duration>(CHRONO::nanoseconds(cpt_s));
      }
      
      static posix_duration to_posix(chrono_duration cd) {
        CHRONO::hours h = CHRONO::duration_cast<CHRONO::hours>(cd);
        posix_duration result = boost::posix_time::hours(h.count());
        typename CHRONO_NS::common_type<chrono_duration, CHRONO::hours>::type
        sub_hours = cd-h;
        
        
        
        CHRONO::seconds secs = CHRONO::duration_cast<CHRONO::seconds>(sub_hours);
        result += boost::posix_time::seconds(secs.count());
        
        
        ns_duration nsecs = CHRONO::duration_cast<ns_duration>(sub_hours-secs);
#  ifdef BOOST_DATE_TIME_HAS_NANOSECONDS
        result += boost::posix_time::nanoseconds(nsecs.count());
#  else 
        result += boost::posix_time::microseconds((nsecs.count()+500)/1000);
#  endif
        return result;
      }
      
    }; // TREX::utils::chrono_posix_convert

# else // DOXYGEN

    /** @brief Boost chrono to POSIX duration conversion
     *
     * @tparam ChronoDuration A boost chrono duration type
     *
     * An helper to convert back and forth POSIX durations to/from their 
     * @p ChronoDuration equivalent.
     *
     * This helper exists as in Boost posix dates and duration  are not 
     * represented using boost chrono which is used in trex to describe 
     * a tick duration.Therefore, in order ot relate trex ticks to human-readable 
     * real-time values, we needed simple way to go back anf forth between the 
     * two representaions  
     *
     * @note Depending omn  the system we are running on and the precision of 
     * @p ChronoDuration these conversions can result on rounding errors.
     *
     * @ingroup utils
     */
    template<class ChronoDuration>
    struct chrono_posix_convert {
      /** @brief Type used for chrono duration
       */
      typedef ChronoDuration                   chrono_duration;
      /** @brief Type used to represent posix duration 
       */
      typedef boost::posix_time::time_duration posix_duration;
      
      /** @brief Conversion from posix to chrono
       *
       * @param[in] pd A duration in POSIX representation
       * @return The equivalent of @p pd as a `chrono_duration`
       *
       * @sa to_posix(chrono_duration const &)
       */
      static chrono_duration to_chrono(posix_duration const &pd);
      /** @brief Conversion from chrono to posix
       *
       * @param[in] cd A duration as a `chrono_duyration`
       * @return The equivalent of @p cd as a posix duration
       *
       * @sa to_chrono(posix_duration const &)
       */
      static posix_duration  to_posix(chrono_duration const &cd);
       
    };
   
# endif // DOXYGEN

  } // utils 
} // TREX

#endif // H_TimeUtils 
