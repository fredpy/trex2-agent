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
/** @file "trex/utils/chrono_helper.hh"
 * @brief chrono class helper utilities
 *
 * This header define utilities that helps manipulate and display chrono 
 * instances.
 *
 * @author Frederic Py <fpy@mbari.org>
 * @ingroup utils
 */
#ifndef H_trex_utils_chrono_helper
# define H_trex_utils_chrono_helper

# include "platform/chrono.hh"
# include "platform/cpp11_deleted.hh"
# include <ostream>
# include <iostream>


namespace TREX {
  namespace utils {
    
    /** @brief Human readable duration display
     *
     * @tparam CharT stream char type
     * @tparam Traits stream char traits
     * @tparam Rep duration base type
     * @tparam Period duration precision period
     *
     * @param[in,out] out An output stream
     * @param[in] d A duration
     *
     * Serialize the value of the duration @p d into @p out as an
     * "human readable" value.
     *
     * @return @p out after the operation
     */
    template <class CharT, class Traits, class Rep, class Period>
    std::basic_ostream<CharT, Traits> &display(std::basic_ostream<CharT, Traits> &out,
                                               CHRONO::duration<Rep, Period> d) {
      if( d < CHRONO::duration<Rep, Period>::zero() ) {
        d = -d;
        out.put('-');
      }
      // Extract the number of seconds
      CHRONO::seconds
      s = CHRONO::duration_cast<CHRONO::seconds>(d);
      
      if( s>CHRONO::seconds::zero() ) {
        d -= s;
        
        // Extract number of minutes
        CHRONO::minutes
        m = CHRONO::duration_cast<CHRONO::minutes>(s);
        
        if( m > CHRONO::minutes::zero() ) {
          s -= m;
          
          // Extract the number of hours
          CHRONO::hours
          h = CHRONO::duration_cast<CHRONO::hours>(m);
          
          if( h > CHRONO::hours::zero() ) {
            m -= h;
            // Display as hours:MM:SS.XXXX
            out<<h.count()<<':';
            if( m < CHRONO::minutes(10) )
              out.put('0'); // Need 2 digits
          }
          out<<m.count()<<':';
          if( s < CHRONO::seconds(10) )
            out.put('0'); // Need 2 digits
        }
      }
      typedef CHRONO::duration<long double> f_secs;
      f_secs ss = CHRONO::duration_cast<f_secs>(s);
      ss += d;
# ifndef CPP11_HAS_CHRONO
      return CHRONO::duration_short(out)<<ss;
# else
      return out<<ss.count()<<" s";
# endif
    }
    
    template<class Clock>
    class chronograph :boost::noncopyable {
    public:
      typedef typename Clock::time_point time_point;
      typedef typename Clock::duration   duration;
      
      chronograph(duration &dest):m_start(Clock::now()), output(dest) {
        output = duration();
      }
      ~chronograph() {
        output = Clock::now()-m_start;
      }
      
    private:
      time_point m_start;
      duration &output;
      
      chronograph() DELETED;
    };
    

    class cpu_clock {
    public:
      typedef CHRONO::nanoseconds duration;
      typedef duration::rep       rep;
      typedef duration::period    period;
      
      typedef CHRONO::time_point<cpu_clock> time_point;
      static const bool is_steady = true;
      
      static time_point now();
    };
    
    
  } // TREX::utils
} // TREX

namespace boost {
  namespace chrono {
    
    
//    template<class CharT>
//    struct clock_string<TREX::LSTS::dune_posix_clock, CharT> {
//      /** @brief Clock name */
//      static std::basic_string<CharT> name() {
//        static const CharT u[] =
//        { 'D', 'u', 'n', 'e', '_', 'p', 'o', 's', 'i', 'x', '_', 'c', 'l', 'o', 'c', 'k' };
//        static const std::basic_string<CharT> str(u, u + sizeof(u) / sizeof(u[0]));
//        return str;
//      }
//      /** @brief Clock epoch */
//      static std::basic_string<CharT> since() {
//        static const CharT u[] =
//        { ' ', 's', 'i', 'n', 'c', 'e', ' ', 'J', 'a', 'n', ' ', '1', ',', ' ', '1', '9', '7', '0' };
//        static const std::basic_string<CharT> str(u, u + sizeof(u) / sizeof(u[0]));
//        return str;
//      }
//    }; // boost::chrono::clock_string<TREX::LSTS::dune_posix_clock, >

  }
}

#endif // H_trex_utils_chrono_helper
