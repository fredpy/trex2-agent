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

# include <sys/time.h>

# include <cmath>

# include <iostream>
# include <iomanip>

/** @brief Number of micro seconds in a second  
 * @ingroup utils
 */
# define TV_USEC_F 1000000l

namespace TREX {
  namespace utils {
  
    /** @brief "Less than" operator
     *
     * @param t1 a date
     * @param t2 a date
     *
     * @retval true if @e t1 is before @e t2
     * @retval false else.
     * @ingroup utils
     */
    inline bool operator< (timeval const &t1, timeval const &t2) {
      return t1.tv_sec < t2.tv_sec ||
	(t1.tv_sec == t2.tv_sec && t1.tv_usec < t2.tv_usec);
    }

    /** @brief "Equal to" operator
     *
     * @param t1 a date
     * @param t2 a date
     *
     * @retval true if @e t1 has the same value as @e t2
     * @retval false else.
     * @ingroup utils
     */
    inline bool operator==(timeval const &t1, timeval const &t2) {
      return (t1.tv_sec == t2.tv_sec && t1.tv_usec == t2.tv_usec);
    }
    
    /** @brief "Not equal to" operator
     *
     * @param t1 a date
     * @param t2 a date
     *
     * @retval true if @e t1 is different from @e t2
     * @retval false else.
     * @ingroup utils
     */
    inline bool operator!=(timeval const &t1, timeval const &t2) {
      return !operator==(t1, t2);
    }

    /** @brief "Greater than" operator
     *
     * @param t1 a date
     * @param t2 a date
     *
     * @retval true if @e t1 is after @e t2
     * @retval false else.
     * @ingroup utils
     */
    inline bool operator> (timeval const &t1, timeval const &t2) {
      return t2<t1;
    }
    
    /** @brief "Less or equal to" operator
     *
     * @param t1 a date
     * @param t2 a date
     *
     * @retval true if @e t1 is before or equal to @e t2
     * @retval false else.
     * @ingroup utils
     */
    inline bool operator<=(timeval const &t1, timeval const &t2) {
      return !operator> (t1, t2);
    }

    /** @brief "Greater or equal to" operator
     *
     * @param t1 a date
     * @param t2 a date
     *
     * @retval true if @e t1 is after or equal to @e t2
     * @retval false else.
     * @ingroup utils
     */
    inline bool operator>=(timeval const &t1, timeval const &t2) {
      return !operator< (t1, t2);
    }
    
    /** @brief Addition 
     *
     * @param t1 a date
     * @param t2 a date
     *
     * @return The sum of @e t1 and @e t2
     * @ingroup utils
     */
    inline timeval operator+ (timeval const &t1, timeval const &t2) {
      timeval result;
      
      result.tv_sec = t1.tv_sec + t2.tv_sec;
      if ( (result.tv_usec = t1.tv_usec + t2.tv_usec)>=TV_USEC_F ) {
	++result.tv_sec;
	result.tv_usec -= TV_USEC_F;
      }
      return result;
    }

    /** @brief Conversion to double
     *
     * @param t a date
     *
     * Convert @a t to a @c double value.
     *
     * @warning There may be a precision loss during conversion.
     *
     * @return A double value corresponding to @e t
     * @ingroup utils
     */
    inline double to_double(timeval const &t) {
      double result = t.tv_usec;
      
      result /= TV_USEC_F;
      result += t.tv_sec;
      return result;
    }

    /** @brief conversion of a @c double to a @c timeval
     * @param v A @c double representation of time in seconds
     *
     * @return the conversion of @a v to a @c timeval
     * @ingroup utils
     */
    inline timeval to_timeval(double v) {
      timeval res;
      res.tv_sec = static_cast<long>(std::floor(v));
      res.tv_usec = static_cast<long>(std::floor((v-res.tv_sec)*1e6));
      return res;
    }
    
    /** @brief Increment operator
     *
     * @param t1 a date
     * @param t2 a date
     *
     * This operator increments @e t1 by the value of @e t2.
     *
     * @return @e t1 after the operation
     * @ingroup utils
     */
    inline timeval &operator+=(timeval &t1, timeval const &t2) {
      t1.tv_sec += t2.tv_sec;
      if ( (t1.tv_usec += t2.tv_usec) >= TV_USEC_F ) {
	++t1.tv_sec;
	t1.tv_usec -= TV_USEC_F;
      }
      return t1;
    }

    /** @brief Substract
     *
     * @param t1 a date
     * @param t2 a date
     *
     * @return the difference between @e t1 and @e t2
     * @ingroup utils
     */
    inline timeval operator- (timeval const &t1, timeval const &t2) {
      timeval result;
      result.tv_sec = t1.tv_sec - t2.tv_sec;
      if ( (result.tv_usec = t1.tv_usec - t2.tv_usec) < 0 ) {
	--result.tv_sec;
	result.tv_usec += TV_USEC_F;
      }
      return result;
    }

    /** @brief Divide 
     *
     * @param t a date
     * @param n The divider 
     *
     * @return The division of @e t by @e n
     * @ingroup utils
     */
    inline timeval operator/ (timeval const &t, long n) {
      timeval result;
      
      result.tv_sec = t.tv_sec / n;
      result.tv_usec = ((t.tv_sec % n) * TV_USEC_F + t.tv_usec) / n;
      
      return result;
    }

    /** @brief Multiply 
     *
     * @param t a date
     * @param n The factor 
     *
     * @return The mutlplication of @e t by @e n
     * @ingroup utils
     */
    inline timeval operator* (timeval const &t, long n) {
      timeval result;
      
      result.tv_usec = t.tv_usec * n;
      result.tv_sec = t.tv_sec * n + (result.tv_usec) / TV_USEC_F;
      result.tv_usec = result.tv_usec % TV_USEC_F;
      
      return result;    
    }
    
    /** @brief Decrement operator
     *
     * @param t1 a date
     * @param t2 a date
     *
     * This operator decrement the value of @e t1 by @e t2.
     *
     * @return @e t1 after the operation 
     * @ingroup utils
     */
    inline timeval &operator-=(timeval &t1, timeval const &t2) {
      t1.tv_sec -= t2.tv_sec;
      if ( (t1.tv_usec -= t2.tv_usec) < 0 ) {
	--t1.tv_sec;
	t1.tv_usec += TV_USEC_F;
      }
      return t1;
    }

    /** @brief Print operator
     *
     * @param os An output stream
     * @param t a date
     *
     * Thsi method writes the value of @e t in @e os. The value
     * format is @e seconds.microseconds
     *
     * @return @e os after the operation
     * @ingroup utils
     */
    inline std::ostream &operator<<(std::ostream &os, timeval const& t) {
      if( t.tv_sec>=0 ) 
	return os<<t.tv_sec<<'.'<<std::setw(6)<<std::setfill('0')<<t.tv_usec;
      else {
	if( -1==t.tv_sec )
	  os<<'-';
	return os<<(t.tv_sec+1)<<'.'<<std::setw(6)<<std::setfill('0')
		 <<(TV_USEC_F-t.tv_usec);
      }	
    }
    
  } // utils 
} // TREX

#endif // H_TimeUtils 
