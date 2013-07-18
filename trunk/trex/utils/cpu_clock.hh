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
#ifndef H_trex_utils_cpu_clock
# define H_trex_utils_cpu_clock

# include "platform/chrono.hh"

namespace TREX {
  namespace utils {
    
    class cpu_clock {
    public:
      typedef CHRONO::nanoseconds duration;
      typedef duration::rep       rep;
      typedef duration::period    period;
      
      typedef CHRONO::time_point<cpu_clock> time_point;
      static const bool is_steady = true;
      
      static time_point now();
    }; // TREX::utils::cpu_clock

  }
}

# ifndef CPP11_HAS_CHRONO

namespace boost {
  namespace chrono {
    
    template<class CharT>
    struct clock_string<TREX::utils::cpu_clock, CharT> {
      /** @brief Clock name */
      static std::basic_string<CharT> name() {
        static const CharT u[] =
        { 'P', 'r', 'o', 'c', 'e', 's', 's', '_', 'c', 'p', 'u',  '_', 'c', 'l', 'o', 'c', 'k' };
        static const std::basic_string<CharT> str(u, u + sizeof(u) / sizeof(u[0]));
        return str;
      }
      /** @brief Clock epoch */
      static std::basic_string<CharT> since() {
        static const CharT u[] =
        { ' ', 's', 'i', 'n', 'c', 'e', ' ', 'p', 'r', 'o', 'c', 'e', 's', ' ', 's', 't', 'a', 'r', 't' };
        static const std::basic_string<CharT> str(u, u + sizeof(u) / sizeof(u[0]));
        return str;
      }
    }; // boost::chrono::clock_string<TREX::utils::cpu_clock, >
    
  }
}

# endif // CPP11_HAS_CHRONO

#endif // H_trex_utils_cpu_clock
