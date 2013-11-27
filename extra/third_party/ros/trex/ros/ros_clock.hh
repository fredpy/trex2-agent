/*********************************************************************
 * Software License Agreement (BSD License)
 * 
 *  Copyright (c) 2012, MBARI.
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
#ifndef H_trex_ros_ros_clock
# define H_trex_ros_ros_clock

# include <trex/agent/RealTimeClock.hh>
# include <ros/time.h>

namespace TREX {
  namespace ROS {

    /** @brief ROS based clock
     *
     * This class provides a trex clock definition based on ROS API 
     * for time access
     *
     * This cand be used to define a @c TREX::agent::rt_clock that will
     * access time using ROS libraries ensuring a common representaion 
     * with other ROS components.
     *
     * As for any rt_clock the basic frequency of the clock can be defined 
     * easily but we also provide a default clock running a 1000Hz defined 
     * as TREX::ROS::clock in the code.
     *
     * @sa TREX::ROS::clock
     *
     * @ingroup ros
     * @author Frederic Py <fpy@mbari.org>
     */
    class ros_clock {
    public:
      typedef CHRONO::nanoseconds           duration;
      typedef duration::rep                 rep;
      typedef duration::period              period;
      typedef CHRONO::time_point<ros_clock>  time_point;

      /** @brief steady clock flag
       *
       * This is the flag used in chrono (boost or std) to idndicate if this 
       * clock implementation is steady (monotonic progress) or not. For example 
       * the system clock in most unixes is not steady as one can reset the time
       *
       * @note There's no clear indication as to what type of clock ROS is using 
       * in its API. Therefore we assume it to be non steady (as it probably 
       * relies on standard posix calls such as gettimeofday).
       */
      static const bool is_steady = false;

      static time_point now();
    }; // TREX::ROS::ros_clock
  }

  namespace agent {

    template<>
    Clock::duration_type rt_clock<CHRONO_NS::milli, ROS::ros_clock>::doSleep() {
      Clock::duration_type delay = getSleepDelay();
      ::ros::Duration ros_dt;
      ros_dt.fromNSec(delay.count());
      // Now Compute the date in ROS term
      ::ros::Time::sleepUntil(::ros::Time::now()+ros_dt);
      return delay;
    }
  }

  namespace ROS {

    /** @brief default clock for ROS
     *
     * This type defines the default clock provided to interface with ROS time 
     * Its base frequency is 1000Hz. It can be loaded on a agent config file 
     * with the tag @c ROSClock. For example :
     * @code
     * <Agent name="ros_agent" finalTick="1000">
     *  <Plugin name="ros_pg">
     *    <ROSClock seconds="1"/>
     *    <!-- 
     *        [...]
     *     -->
     *  </Plugin>
     * </Agent>
     * @endcode
     * Will create the agent ros_agent with the ros clock runening with a 1Hz tick rate.
     *
     * @ingroup ros
     * @author Frederic Py <fpy@mbari.org>
     */
    typedef agent::rt_clock<CHRONO_NS::milli, ros_clock> clock;

  } // TREX::ROS
} // TREX

#ifndef CPP11_HAS_CHRONO
namespace boost {
  namespace chrono {
  
    /** @brief Information about TREX::LSTS::dune_posix 
     *
     * Textual information for the dune posix clock
     *
     * @relates TREX::LSTS::dune_posix
     *
     * @author Frederic Py 
     */
    template<class CharT>
    struct clock_string<TREX::ROS::ros_clock, CharT> {
      /** @brief Clock name */
      static std::basic_string<CharT> name() {
        static const CharT u[] =  
          { 'R', 'o', 's',  '_', 'c', 'l', 'o', 'c', 'k' };
        static const std::basic_string<CharT> str(u, u + sizeof(u) / sizeof(u[0]));
        return str;
      } 
      /** @brief Clock epoch */
      static std::basic_string<CharT> since() {
        static const CharT u[] =
          { ' ', 's', 'i', 'n', 'c', 'e', ' ', 'J', 'a', 'n', ' ', '1', ',', ' ', '1', '9', '7', '0' };
        static const std::basic_string<CharT> str(u, u + sizeof(u) / sizeof(u[0]));
        return str;
      }
    }; // boost::chrono::clock_string<TREX::LSTS::dune_posix_clock, >
  }
}
#endif // CPP11_HAS_CHRONO

#endif // H_trex_ros_ros_clock
