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
#pragma once
#include <chrono>

namespace TREX::utils {

/** @brief process cpu clock
 *
 * This class implements a chrono clock that measure the process
 * cpu time (both user and system) of the current process.
 *
 * It allows to uses such clock independently on whteher we use boost
 * or c++11 implementation (the later not providing such clock). In
 * addition it uses getrusage as a backend as opposed to boost using
 * times which is deprecated on some systems (at least recent version
 * of MacOS X).
 *
 * @author Frederic Py <fpy@mbari.org>
 */
class cpu_clock {
public:
  typedef std::chrono::nanoseconds duration; //*< Duration type
  typedef duration::rep rep;                 //*< internal representation type
  typedef duration::period period;           //*< period type

  typedef std::chrono::time_point<cpu_clock> time_point; //*< time point type
  /** @brief Steady information
   *
   * Indicates that this clock is steady.
   *
   * @note While the cpu clock being steady is open for debate; the clock
   * date cannot change to anterior date but can stilll stay at the same
   * value when the process do not run. Boost implementation claims its
   * implementation is steady. Therfore we just follow suit.
   */
  static const bool is_steady = true;

  /** @brief Current date
   *
   * Get the current date in term of proecess cpu time. The time corresponds
   * to the wall cpu time (ie system time+user time). And epoch is set to
   * process start time.
   *
   * @return process wall cpu time
   */
  static time_point now();
}; // TREX::utils::cpu_clock

} // namespace TREX::utils
