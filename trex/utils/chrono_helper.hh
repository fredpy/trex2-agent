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
#pragma once
#include <chrono>
#include <iostream>
#include <ostream>

namespace TREX::utils {

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
std::basic_ostream<CharT, Traits> &
display(std::basic_ostream<CharT, Traits> &out,
        std::chrono::duration<Rep, Period> d) {
  if (d < std::chrono::duration<Rep, Period>::zero()) {
    d = -d;
    out.put('-');
  }
  // Extract the number of seconds
  std::chrono::seconds s = std::chrono::duration_cast<std::chrono::seconds>(d);

  if (s > std::chrono::seconds::zero()) {
    d -= s;

    // Extract number of minutes
    std::chrono::minutes m =
        std::chrono::duration_cast<std::chrono::minutes>(s);

    if (m > std::chrono::minutes::zero()) {
      s -= m;

      // Extract the number of hours
      std::chrono::hours h = std::chrono::duration_cast<std::chrono::hours>(m);

      if (h > std::chrono::hours::zero()) {
        m -= h;
        // Display as hours:MM:SS.XXXX
        out << h.count() << ':';
        if (m < std::chrono::minutes(10))
          out.put('0'); // Need 2 digits
      }
      out << m.count() << ':';
      if (s < std::chrono::seconds(10))
        out.put('0'); // Need 2 digits
    }
  }
  typedef std::chrono::duration<long double> f_secs;
  f_secs ss = std::chrono::duration_cast<f_secs>(s);
  ss += d;
  return out << ss.count() << " s";
}

template <class Clock> class chronograph  {
public:
  typedef typename Clock::time_point time_point;
  typedef typename Clock::duration duration;

  chronograph(duration &dest) : m_start(Clock::now()), output(dest) {
    output = duration();
  }
  ~chronograph() { output = Clock::now() - m_start; }

private:
  time_point m_start;
  duration &output;

  chronograph() = delete;
  chronograph(chronograph const &) =delete;
};

} // namespace TREX::utils
