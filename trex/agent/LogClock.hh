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
#ifndef H_trex_LogClock
# define H_trex_LogClock

# include "Clock.hh"

namespace TREX {
  namespace agent {
  
    /** @brief Log replaying clock
     *
     * A pseudo clock that reproduces the behavior of a previously ran clock as 
     * indicated in a log file. 
     *
     * This cock allow to accurately replay a mission by giving as many atomic 
     * call steps for each tcj as it really occured during the replayed mission.
     *
     * @author Frederic Py <fpy@mbari.org>
     * @ingroup agent
     */
    class LogClock :public Clock {
    public:
      LogClock(boost::property_tree::ptree::value_type &node);
      ~LogClock() {}
      
      date_type epoch() const {
        return m_epoch;
      }
      duration_type tickDuration() const {
        return m_period;
      }

      std::string info() const;
     
    private:
      //duration_type doSleep();
      TREX::transaction::TICK getNextTick();
      bool free() const;      
      
      struct tick_info {
        tick_info(boost::property_tree::ptree::value_type const &node);
        ~tick_info() {}
        
        TREX::transaction::TICK const date;
        size_t const count;
        size_t const free_count;
      }; // TREX::agent::LogClock::tick_info
      
      date_type     m_epoch;
      duration_type m_period;
      std::list<tick_info> m_ticks;
      // size_t m_counter;
      
      // TREX::transaction::TICK m_last;
    }; // TREX::agent::LogClock

  } // TREX::agent
} // TREX

#endif // H_trex_LogClock
