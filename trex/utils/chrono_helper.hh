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
#ifndef H_trex_utils_chrono_helper
# define H_trex_utils_chrono_helper

# include <boost/chrono/chrono_io.hpp>

# include <ostream>
# include <iostream>

namespace TREX {
    namespace utils {
        
        /** @brief Human readable duration display
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
                                                   boost::chrono::duration<Rep, Period> d) {
            if( d < boost::chrono::duration<Rep, Period>::zero() ) {
                d = -d;
                out.put('-');
            }
            // Extract the number of seconds
            boost::chrono::seconds
            s = boost::chrono::duration_cast<boost::chrono::seconds>(d);
            
            if( s>boost::chrono::seconds::zero() ) {
                d -= s;
                
                // Extract number of minutes
                boost::chrono::minutes
                m = boost::chrono::duration_cast<boost::chrono::minutes>(s);
                
                if( m > boost::chrono::minutes::zero() ) {
                    s -= m;
                    
                    // Extract the number of hours
                    boost::chrono::hours
                    h = boost::chrono::duration_cast<boost::chrono::hours>(m);
                    
                    if( h > boost::chrono::hours::zero() ) {
                        m -= h;
                        // Display as hours:MM:SS.XXXX
                        out<<h.count()<<':';
                        if( m < boost::chrono::minutes(10) )
                            out.put('0'); // Need 2 digits
                    }
                    out<<m<<':';
                    if( s < boost::chrono::seconds(10) )
                        out.put('0'); // Need 2 digits
                }
            } 
            typedef boost::chrono::duration<long double> f_secs;
            f_secs ss = boost::chrono::duration_cast<f_secs>(s);
            ss += d;
            return boost::chrono::duration_short(out)<<ss;
        }
        
    } // TREX::utils
} // TREX 

#endif // H_trex_utils_chrono_helper
