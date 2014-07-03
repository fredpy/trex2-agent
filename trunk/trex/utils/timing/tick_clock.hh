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
#ifndef H_TREX_utils_tick_clock
# define H_TREX_utils_tick_clock

# include <trex/config/chrono.hh>

namespace TREX {
  namespace utils {

    /** @brief tick based clock
     *
     * @tparam Period tick period
     * @tparam Clock  a clock
     *
     * This class implements a clock with a tick rate of @p Period 
     * based on the subjacent clock @p Clock. The epoch of the clock 
     * is set at construction time and cannot be changed allowing to
     * maintain the steadiness of the subjacent @p Clock.
     *
     * For example, one can implement a 1Hz clock as follow:
     * @code 
     *  typedef tick_clock<CHRONO::seconds> one_hz_clock;
     *
     *  one_hz_clock my_clock; // starts the clock
     *  one_hz_clock::time_point date = my_clock.now(); // get current second since started
     * @endcode
     *
     * @note While apparently similiar to a chrono clock, this class 
     * is not really one as its @c now() method is not static.
     *
     * @author Frederic Py <fpy@mbari.org>
     * @ingroup utils
     */
    template<typename Period, class Clock=CHRONO::high_resolution_clock>
    class tick_clock {
    public:
      /** @brief Subjacent clock type 
       *
       * The chrono clock type used to measure time
       */
      typedef Clock                                  base_clock;
      /** @brief Subjacent duration
       *
       * The duration used by the subjacent clock. This type is 
       * used to indicate the sub tick delays.
       */
      typedef typename base_clock::duration          base_duration;
      /** @brief Epoch time point type
       *
       * The type used to repsresent the epocjh of this clock
       */
      typedef typename base_clock::time_point        base_time_point;

      /** @brief Tick representation
       *
       * The type used tyo count the clock ticks
       */
      typedef typename base_clock::rep               rep;
      /** @brief Tick period 
       * 
       * The period of an atomic tick for this clock
       */
      typedef Period                                 period;
      /** @brief Duration type
       *
       * The type used to represent durations for this clock
       */
      typedef CHRONO::duration<rep, period>   duration;
      /** @brief time point type 
       *
       * The type used to represent time points for this clock
       */
      typedef CHRONO::time_point<tick_clock, duration>  time_point;

      static bool const is_steady = base_clock::is_steady;

      /** @brief Constructor 
       *
       * Create a new clock and set its epoch to the current time 
       * of the subjacent clock
       */
      tick_clock()
	:m_epoch(base_clock::now()) {}
      /** @brief Constructor 
       *
       * @param[in] epoch a time point
       *
       * Create a new clock with an epoch of @p epoch
       */
      explicit tick_clock(base_time_point const &epoch)
        :m_epoch(epoch) {}

      /** @brief Copy constructor 
       *
       * @tparam Period2 A period
       * @param[in] other Another clock
       *
       * Create a new clcock with the same epoch as @p other
       *
       * @note It is not required for @p other to have the same @p Period as 
       * long as it share the same @p Clock
       */
      template<typename Period2>
      tick_clock(tick_clock<Period2, Clock> const &other)
	:m_epoch(other.epoch()) {}
      /** @brief Destructor */
      ~tick_clock() {}

      /** @brief clock epoch
       *
       * @return the epoch date for this clock
       */
      base_time_point const &epoch() const {
	return m_epoch;
      }
      /** @brief Current date
       *
       * Identifies the current date for the clock using a unit tick 
       * of @p Period and aligned with the clock epoch
       *
       * @return the current date of this clock
       *
       * @sa epoch() const
       * @sa now(base_duration &) const
       * @sa now(duration const &, base_duration &) const
       */
      time_point now() const {
	base_duration delta = Clock::now()-epoch();                       // Get duration since epoch
	return time_point(CHRONO::duration_cast<duration>(delta)); // round to our Period
      }
      /** @brief Accurate current date
       *
       * @param[out] remain sub tick duration
       *
       * Identifies the current date for the clock using a unit tick 
       * of @p Period and aligned with the clock epoch. The extra time 
       * from the clock is stored in @p remain to give a finer grained 
       * view.
       *
       * @return the current date of this clock
       *
       * @sa epoch() const
       * @sa now() const
       * @sa now(duration const &, base_duration &) const
       */
      time_point now(base_duration &remain) const {
	remain = Clock::now()-epoch();                                     // Get duration since epoch
	duration n_ticks = CHRONO::duration_cast<duration>(remain); // round to our Period
	remain -= n_ticks;                                                 // get sub Period time
	return time_point(n_ticks);
      }
      /** @brief Accurate current date 
       *
       * @param[in] tick duration
       * @param[out] remain sub tick duration
       *
       * Identifies the current date for the clock using a unit @p tick 
       * of @p Period and aligned with the clock epoch. The extra time 
       * from the clock is stored in @p remain to give a finer grained 
       * view.
       *
       * @pre @p tick is a strictly positive duration
       *
       * @return the current date of this clock
       *
       * @note the returned value is not counted based on @p tick instead it returns 
       * a duration that is a multiple of @p tick. 
       *
       * @sa epoch() const
       * @sa now() const
       * @sa now(base_duration &) const
       */
      time_point now(duration const &tick, base_duration &remain) const {
	remain = Clock::now()-epoch();                                     // Get duration since epoch
	duration n_ticks = CHRONO::duration_cast<duration>(remain), // round to our Period
          extra = n_ticks%tick;                                            // get the module of tick
        n_ticks -= extra;                                                  // round to a multiple of tick
	remain -= n_ticks;                                                 // get sub tick time
	return time_point(n_ticks);
      }
      
      
      base_duration to_next(time_point &date, duration const &tick) const {
        base_duration ret = Clock::now()-epoch();
        duration extra(CHRONO::duration_cast<duration>(ret));
        time_point cur(extra);
        extra %= tick; 
        cur -= extra;
        if( cur > date ) {
          ret -= date.time_since_epoch();
          ret -= tick;
          date = cur;
          return ret;
        } else 
          return base_duration::zero();
      }
      
      
      
      /** @brief time left 
       * 
       * @param[in] target A date
       *
       * @return the duration until target
       */
      base_duration left(time_point const &target) const {
        duration t_dur = target.time_since_epoch();
        base_time_point 
           real_target = epoch()+CHRONO::duration_cast<base_duration>(t_dur);
        return real_target-Clock::now();
      }
    private:
      base_time_point const m_epoch;
      
    }; // TREX::utils::tick_clock 

  } // TREX::utils
} // TREX

#endif // H_TREX_utils_tick_clock 
