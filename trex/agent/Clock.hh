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

# include "bits/agent_graph.hh"

# include <trex/utils/platform/system_error.hh>
# include <trex/transaction/Tick.hh>
# include <trex/utils/asio_fstream.hh>

# include <trex/utils/timing/posix_utils.hh>

namespace TREX {
  namespace agent {

    using utils::log::null;
    using utils::log::info;
    using utils::log::warn;
    using utils::log::error;

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
      /** @brief Clock error
       *
       * The basic exception thrown by a clock when an error occurs
       * 
       * @author Frederic Py <fpy@mbari.org>
       * @relates Clock
       * @ingroup agent
       */
      class Error :public TREX::utils::Exception {
      public:
        /** @brief Constructor
         *
         * @param[in] msg An erro message
         *
         * Create a new instance  withe associated message @p msg
         */
        Error(std::string const &msg) throw()
          :TREX::utils::Exception(msg) {}
        /** @brief Destructor */
        ~Error() throw() {}
      }; // TREX::agent::Clock::Error
            
      typedef transaction::graph::duration_type          duration_type;
      typedef transaction::graph::date_type              date_type;
    
      /** @brief Destructor */
      virtual ~Clock();

      /** @brief Start the clock
       *
       * This method is the public interface used to start
       * the clock.
       *
       * @sa void start()
       */
      void doStart();

      TREX::transaction::TICK tick();
      virtual TREX::transaction::TICK max_tick() const {
        return std::numeric_limits<TREX::transaction::TICK>::max();
      }
      bool is_free() const;

      
      /** @brief Initial tick
       *
       * @return the value of the initial tick
       */
      virtual TREX::transaction::TICK initialTick() const {
	return 0;
      }
      
      virtual date_type epoch() const {
        return boost::posix_time::from_time_t(initialTick()); 
      }
      
      /** @brief tick duration
       *
       * This methods return the TICK duration. It is expected to be in seconds
       *
       * @return tick duration
       */
      virtual duration_type tickDuration() const {
	return CHRONO::seconds(1);
      }
      virtual TREX::transaction::TICK timeToTick(date_type const &date) const {
        return initialTick()+(date.since(epoch()).to_chrono<duration_type>().count()/tickDuration().count());
      }
      virtual date_type tickToTime(TREX::transaction::TICK cur) const {
        utils::rt_duration delta(tickDuration()*(cur-initialTick()));
        return epoch().add(delta);
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
      duration_type sleep();


      /** @brief Process sleep
       * @param sleepDuration number of seconds to sleep
       *
       * This methods make the calling process to sleep for
       * @a sleepDuration seconds
       *
       * @throw ErrnoExcept An error occurred while trying to sleep
       */
      static void sleep(duration_type const &sleepDuration);
      /** @brief Get tick string
       *
       * @param[in] tick A tick
       * 
       * Display @p tick in human readable format. 
       *
       * @return A string representation for @p tick
       */
      virtual std::string date_str(TREX::transaction::TICK const &tick) const;

      virtual std::string duration_str(TREX::transaction::TICK dur) const;

      /** @brief XML factory for clocks. */
      typedef TREX::utils::XmlFactory<Clock, SHARED_PTR<Clock> > xml_factory;
      
      /** @brief Clock basic information 
       *
       * Give basixc information of the clock used. This information 
       * will be displayed. when the clock will be started by the agent.
       *
       * @return A string describing this clock
       */
      virtual std::string info() const =0;
      
    protected:
       virtual duration_type doSleep();


      /** @brief Check if clock free
       *
       * Check if the clock is currently free. A free clock 
       * allow reactors to execute steps. If the clock is not 
       * free the agent will stop to attempt to insert new 
       * deliberation steps for this tick
       *
       * @retval true if free
       * @retval false otherwise
       */
      virtual bool free() const {
        return true;
      }
      /** @brief get time in TICK
       *
       * This method is used to compute the current time.
       * This is where one specialization of clock will identify
       * whether the TICK value has advanced or not
       *
       * @return current tick value
       */
      virtual TREX::transaction::TICK getNextTick() =0;
 
      /** @brief Time left before next tick
       *
       * This method indicates how much time is left before the
       * next tick
       *
       * @return time left before next tick
       */
      virtual duration_type getSleepDelay() const {
	return m_sleep;
      }
      /** @brief Constructor
       * @param sleepSeconds A default sleep delay
       * Create a new instance with the default sleeping delay set to
       * @a sleepSeconds
       */
      explicit Clock(duration_type const &sleep)
      :m_sleep(sleep), m_started(false), m_data(m_log->service()) {}

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
      
      utils::log::stream syslog(utils::log::id_type const &kind=utils::log::null) 
	const;

    private:
      duration_type const m_sleep;
      utils::singleton::use<utils::log_manager> m_log;
      
      void log_tick() const ;
      
      // Logging related attributes
      mutable bool   m_first;
      mutable bool   m_started;
      mutable transaction::TICK  m_last;
      mutable bool               m_free;
      mutable size_t             m_free_count, m_count;
      mutable utils::async_ofstream m_data;
    }; // TREX::agent::Clock

  } // TREX::agent   
} // TREX

#endif // H_Clock
