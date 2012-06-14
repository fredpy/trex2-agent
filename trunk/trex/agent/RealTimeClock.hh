/* -*- C++ -*- */
/** @file "RealTimeClock.hh"
 * @brief definition of a system based real-time clock for TREX
 *
 * This files defines a real time clock which is the default clock
 * for the TREX agent
 *
 * @author Conor McGann & Frederic Py <fpy@mbari.org>
 * @ingroup agent
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
#ifndef H_RealTimeClock
# define H_RealTimeClock

# include <boost/thread/recursive_mutex.hpp>

# include "Clock.hh"
# include <trex/utils/TimeUtils.hh>
# include <trex/utils/LogManager.hh>
# include <trex/utils/StringExtract.hh>

# include <trex/utils/tick_clock.hh>

namespace TREX {
  namespace agent {
  
    template<class Period, class Clk = boost::chrono::high_resolution_clock>
    struct rt_clock :public Clock {      
    public:
      typedef TREX::utils::tick_clock<Period, Clk> clock_type;
      typedef typename clock_type::duration        tick_rate;
      typedef typename clock_type::rep             rep;
    
      explicit rt_clock(rep const &period)
        :Clock(0.0), m_period(period) {
        check_tick();
      }
      
      explicit rt_clock(tick_rate const &period)
        :Clock(0.0), m_period(period) {
        check_tick();
      }
      
      explicit rt_clock(boost::property_tree::ptree::value_type &node) 
        :Clock(0.0), 
         m_period(utils::parse_attr<rep>(node, "tick")) {
        check_tick();
      }
      
      ~rt_clock() {}
      
      transaction::TICK getNextTick() {
        typename mutex_type::scoped_lock guard(m_lock);
        if( NULL!=m_clock.get() ) {
          typename clock_type::base_duration how_late = m_clock->to_next(m_tick, m_period);
          if( how_late > clock_type::base_duration::zero() ) {
            // TODO diaply a warning
          }
          return m_tick.time_since_epoch().count()/m_period.count();
        } else 
          return 0;
      }
      
      bool free() const {
        return true;
      }
      
      double tickDuration() const {
        return boost::chrono::duration_cast< boost::chrono::duration<double> >(m_period).count();
      }
      
      transaction::TICK timeToTick(time_t secs, suseconds_t usecs=0) const {
        // TODO implement this
        return Clock::timeToTick(secs, usecs);
      }
      double tickToTime(TREX::transaction::TICK cur) const {
        // TODO implement this
        return Clock::tickToTime(cur);
      }
      std::string date_str(TREX::transaction::TICK &tick) const {
        // TODO implement this
        return Clock::date_str(tick);
      }
      
    private:
      void start() {
        typename mutex_type::scoped_lock guard(m_lock);
        m_clock.reset(new clock_type);
        m_tick -= m_tick.time_since_epoch();
      }
      double getSleepDelay() const {
        typename mutex_type::scoped_lock guard(m_lock);
        if( NULL!=m_clock.get() ) {
          typename clock_type::time_point target = m_tick + m_period;
          typename clock_type::base_duration left = m_clock->left(target);
          if( left >= clock_type::base_duration::zero() )
            return boost::chrono::duration_cast< boost::chrono::duration<double> >(left).count();
          else {
            // TODO warn that we ran late ...
            return 0.0;
          }
        } else 
          return Clock::getSleepDelay();
      }
    
      void check_tick() const {
        if( m_period <= tick_rate::zero() )
          throw TREX::utils::Exception("[clock] tick rate must be greater than 0");
      }
      
      typedef boost::recursive_mutex mutex_type;
      
      mutable mutex_type        m_lock;
      tick_rate                 m_period;
      std::auto_ptr<clock_type> m_clock;
      
      typename clock_type::time_point m_tick;
    }; 
    
    typedef rt_clock<boost::nano> RealTimeClock;
  
    
  } // TREX::agent 
} // TREX

#endif // H_RealTimeClock
