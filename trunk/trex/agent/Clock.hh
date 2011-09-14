/* -*- C++ -*- */
/** @file Clock.hh
 * @brief TREX Clock
 *
 * This class defines the basic utilities for time representation
 * and manipulation inside TREX.
 *
 * @author Conor McGann & Frederic Py <fpy@mbari.org>
 * @ingroup transaction
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
#ifndef H_Clock
#define H_Clock

# include <trex/transaction/Tick.hh>
# include <trex/utils/ErrnoExcept.hh>
# include <trex/utils/XmlFactory.hh>

namespace TREX {
  namespace agent {

    /** @brief TREX clock
     *
     * This class defines the abstract interface to the Clock that
     * manage the TICK advance.
     *
     * @author Conor McGann
     * @ingroup agent
     */
    class Clock {
    public:
      /** @brief Destructor */
      virtual ~Clock() {}

      /** @brief Start the clock
       *
       * This method is the public interface used to start
       * the clock.
       *
       * @sa void start()
       */
      void doStart();

      /** @brief get time in TICK
       *
       * This method is used to compute the current time.
       * This is where one specialization of clock will identify
       * whether the TICK value has advanced or not
       *
       * @return current tick value
       */
      virtual TREX::transaction::TICK getNextTick() =0;
      /** @brief Initial tick
       *
       * @return the value of the initial tick
       */
      virtual TREX::transaction::TICK initialTick() const {
	return 0;
      }
      /** @brief tick duration
       *
       * This methods return the TICK duration. It is expected to be in seconds
       *
       * @return tick duration
       */
      virtual double tickDuration() const {
	return 1.0;
      }
      virtual TREX::transaction::TICK timeToTick(time_t secs, 
						 suseconds_t usecs=0) const {
	return secs;
      }	
      virtual double tickToTime(TREX::transaction::TICK cur) const {
	return cur*tickDuration();
      }

      /** @brief Sleep until next tick
       *
       * This method check the time left before next tick and
       * put the process asleep for this duration.
       *
       * @throw ErrnoExcept An error occurred while trying to sleep
       *
       * @sa double getSleepDelay() const
       * @sa void sleep(double)
       */
      virtual void sleep() const;
      /** @brief Process sleep
       * @param sleepDuration number of seconds to sleep
       *
       * This methods make the calling process to sleep for
       * @a sleepDuration seconds
       *
       * @throw ErrnoExcept An error occurred while trying to sleep
       */
      static void sleep(double sleepDuration);

      /** @brief XML factory for clocks. */
      typedef TREX::utils::XmlFactory<Clock> xml_factory;
      
    protected:
      /** @brief Time left before next tick
       *
       * This method indicates how much time is left before the
       * next tick
       *
       * @return time left before next tick
       */
      virtual double getSleepDelay() const {
	return m_sleepSeconds;
      }
      /** @brief Constructor
       * @param sleepSeconds A default sleep delay
       * Create a new instance with the default sleeping delay set to
       * @a sleepSeconds
       */
      explicit Clock(double sleepSeconds)
	:m_sleepSeconds(sleepSeconds) {}

      /** @brief Internal start method
       *
       * This method is called internally by @c doStart to allow specialized
       * initialization before starting the clock.
       *
       * @sa void doStart()
       */
      virtual void start() {}
      /** @brief increment tick value
       * @param tick A tick variable
       *
       * increase the value of @a tick by 1
       *
       * @deprecated This method was used for computing extra stats.
       * We need to find a better approach. Specifically it would be
       * relevant to have the tick variable located on this class.
       */
      void advanceTick(TREX::transaction::TICK &tick);

    private:
      double const m_sleepSeconds;

    }; // TREX::agent::Clock

  } // TREX::agent   
} // TREX

#endif // H_Clock
