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


# include "Clock.hh"

# include <trex/utils/platform/memory.hh>

# include <trex/utils/TimeUtils.hh>
# include <trex/utils/LogManager.hh>
# include <trex/utils/StringExtract.hh>

# include <trex/utils/tick_clock.hh>
# include <trex/utils/chrono_helper.hh>

# include <boost/thread/recursive_mutex.hpp>
# include <boost/date_time/posix_time/posix_time_io.hpp>

# include <boost/math/common_factor_rt.hpp>

namespace TREX {
  namespace agent {
  
    /** @brief High accuracy tick clock
     *
     * @tparam Param period The tick period base type
     * @tparam Clk a clock definition based on CHRONO 
     *
     * This is the clock inmplementation that allows to implement a clock 
     * with precise tick frequency based on a clock. This clock will allow 
     * to implement any tick which is a multiple of @p Period and will rely on 
     * @p Clk to measure time 
     * 
     * It uses CHRONO library to measure time and allow to ensure that 
     * the tick value is always as accurate as @p Clk allow
     *
     * For exampel if one desire to  implement a clock running a &Hz or any of 
     * its multiple (3.5Hz, 7/3 Hz, ...) he just neads to decalre a clock using 
     * the following type:
     * @code 
     *  typedef rt_clock< boost::ratio<1,7> > seven_hz_clk;
     * @endcode
     *
     * @author Frederic Py
     * @ingroup agent
     */
    template<class Period, class Clk = CHRONO::high_resolution_clock>
    struct rt_clock :public Clock {      
    public:
      using typename Clock::duration_type;
      using typename Clock::date_type;
    
      /** @brief Subjacent clock type
       *
       * The clock used to measure time
       */
      typedef TREX::utils::tick_clock<Period, Clk> clock_type;
      /** @brief Basis tick representation
       *
       * The  duration represent using Period as a core tick value
       */
      typedef typename clock_type::duration        tick_rate;
      typedef typename clock_type::rep             rep;
    
      /** @brief Constructor
       *
       * @param[in] period The tick period
       *
       * Create a new clock with a tick duration of @p period where the unit 
       * of @p period is @t Period
       *
       * @{
       */
      explicit rt_clock(rep const &period, unsigned percent_use=100)
        :Clock(duration_type::zero()), m_period(period) {
        check_tick();
        if( percent_use<5 || percent_use>100 )
          throw utils::Exception("Only accept clock percent_use between 5 and 100%");
        m_sleep_watchdog = percent_use*CHRONO::duration_cast<typename clock_type::base_duration>(m_period);
        m_sleep_watchdog /= 100;
      }
      
      explicit rt_clock(tick_rate const &period, unsigned percent_use=100)
        :Clock(duration_type::zero()), m_period(period) {
        check_tick();
        if( percent_use<5 || percent_use>100 )
          throw utils::Exception("Only accept clock percent_use between 5 and 100%");
        m_sleep_watchdog = percent_use*CHRONO::duration_cast<typename clock_type::base_duration>(m_period);
        m_sleep_watchdog /= 100;
      }
      /** @} */
      /** @brief XML constrauctor
       *
       * @param[in] node An xml definition
       *
       * Create a new instance using the XML definition @p node
       *
       * The XML definition either defines a number of @p Period for the tick:
       * @code 
       *  <ClockName tick="2"/>
       * @endcode
       * Or can specify the tick in term of @c hours, @c minutes, @c seconds, 
       * @c millis, @c micros, @c nanos :
       * @code
       *  <ClockName minutes="2" seconds="20" micros="20000" /> 
       * @endcode
       * All attributes on either definition are expected to be integer.
       */  
      explicit rt_clock(boost::property_tree::ptree::value_type &node) 
        :Clock(duration_type::zero()) {
        boost::optional<rep> 
          tick = utils::parse_attr< boost::optional<rep> >(node, "tick");
        if( tick ) {
          m_period = tick_rate(*tick);
        } else {
          CHRONO::nanoseconds ns_tick = CHRONO::nanoseconds::zero();
          typedef boost::optional< typename CHRONO::nanoseconds::rep >
            value_type;
          value_type value;
          bool has_attr = false;

          // Get nanoseconds
          value = utils::parse_attr<value_type>(node, "nanos");
          if( value ) {
            has_attr = true;
            ns_tick += CHRONO::nanoseconds(*value);
          }
          
          // Get microseconds
          value = utils::parse_attr<value_type>(node, "micros");
          if( value ) {
            has_attr = true;
            ns_tick += CHRONO::microseconds(*value);
          }
          
          // Get milliseconds
          value = utils::parse_attr<value_type>(node, "millis");
          if( value ) {
            has_attr = true;
            ns_tick += CHRONO::milliseconds(*value);
          }
          
          // Get seconds
          value = utils::parse_attr<value_type>(node, "seconds");
          if( value ) {
            has_attr = true;
            ns_tick += CHRONO::seconds(*value);
          }
          
          // Get minutes
          value = utils::parse_attr<value_type>(node, "minutes");
          if( value ) {
            has_attr = true;
            ns_tick += CHRONO::minutes(*value);
          }
          
          // Get hours
          value = utils::parse_attr<value_type>(node, "hours");
          if( value ) {
            has_attr = true;
            ns_tick += CHRONO::hours(*value);
          }
          
          if( !has_attr ) {
            std::ostringstream oss;
            oss<<"No tick duration attribute found. Specify one or more of the following:\n"
            <<" - hours,minutes,seconds,millis,nanos";
            throw utils::XmlError(node, oss.str());
          }
          
          m_period = CHRONO::duration_cast<tick_rate>(ns_tick);
        } 
        check_tick();
        unsigned sleep_ratio = utils::parse_attr<unsigned>(100, node, "percent_use");
        if( sleep_ratio<5 || sleep_ratio>100 )
          throw utils::Exception("Only accept clock precent_use between 5 and 100%");
        m_sleep_watchdog = 
	  sleep_ratio*CHRONO::duration_cast<typename clock_type::base_duration>(m_period);
        m_sleep_watchdog /= 100;
      }
      /** @brief Destructor */
      ~rt_clock() {}
      
      void update_sleep() {
          m_sleep = m_clock->epoch();
          m_sleep += CHRONO::duration_cast<typename clock_type::base_duration>(m_tick.time_since_epoch());
          m_sleep += m_sleep_watchdog;
      }
      
      
      date_type epoch() const {
        typename mutex_type::scoped_lock guard(m_lock);
        if( NULL!=m_clock.get() )
          return m_epoch;
        else 
          return Clock::epoch();
      }
      
      virtual TREX::transaction::TICK max_tick() const {
        typename mutex_type::scoped_lock guard(m_lock);
        if( NULL!=m_clock.get() ) {
          return timeToTick(date_type(boost::posix_time::max_date_time));
        } else
          return Clock::max_tick();
      }


      duration_type tickDuration() const {
        return CHRONO::duration_cast<duration_type>(m_period);
      }
      
      transaction::TICK timeToTick(date_type const &date) const {
        typedef utils::chrono_posix_convert<tick_rate> convert;
    
        return initialTick()+(convert::to_chrono(date-epoch()).count()/m_period.count());
      }
      
      date_type tickToTime(TREX::transaction::TICK cur) const {
        typedef utils::chrono_posix_convert<tick_rate> convert;
        typedef typename convert::posix_duration conv_dur;
        static boost::posix_time::ptime const max_date(boost::posix_time::max_date_time);
        conv_dur delta /*, max_delta = max_date-epoch()*/;
        
        transaction::TICK max_t = max_tick();
        
        if( cur>max_t ) {
          syslog(warn)<<"Tick "<<cur<<" is larger than max delay until "
          <<max_date<<".\n\tReducing it to "<<max_t;
          cur = max_t;
        }
        
        rep const t_max = std::numeric_limits<rep>::max()/(2*m_period.count());
        
        if( t_max<=cur ) {          
          
          conv_dur const base = convert::to_posix(m_period*t_max);
          rep factor = cur/t_max, remains=cur%t_max;
          
          
          
          delta = base*factor;
          
          delta += convert::to_posix(m_period*remains);
        } else {
          // Do the same for the min just in case
          rep const t_min = std::numeric_limits<rep>::min()/(2*m_period.count());
          
          if( t_min>=cur ) {
            conv_dur const base = convert::to_posix(m_period*t_min);
            rep factor = cur/t_min, remains = cur%t_min;
            
            delta = base*factor;
            delta += convert::to_posix(m_period*remains);
          } else {
            delta = convert::to_posix(m_period*cur);
          }
        }
        
        return epoch()+delta;
      }
      std::string date_str(TREX::transaction::TICK const &tick) const {
        return boost::posix_time::to_iso_extended_string(tickToTime(tick));
      }
      std::string duration_str(TREX::transaction::TICK dur) const {
        duration_type dt = tickDuration()*dur;
        typedef TREX::utils::chrono_posix_convert<duration_type> cvt;
        
        typename cvt::posix_duration p_dur = cvt::to_posix(dt);
        std::ostringstream oss;
        oss<<p_dur;
        return oss.str();
      }      
      
      std::string info() const {
        std::ostringstream oss;
# ifndef CPP11_HAS_CHRONO
        oss<<"rt_clock based on "
          <<CHRONO::clock_string<Clk, char>::name();
# else 
	oss<<"rt_clock";
# endif 
        utils::display(oss<<"\n\ttick period: ", m_period);
        oss<<"\n\tfrequency: ";
        
        boost::math::gcd_evaluator<rep> gcdf;
        rep factor = gcdf(m_period.count()*Period::num, Period::den),
          num = (Period::num*m_period.count())/factor,
          den = Period::den/factor;
        if( 1==num ) 
          oss<<den<<"Hz";
        else {
          long double hz = den;
          hz /= num;
          oss<<hz<<"Hz ("<<den<<"/"<<num<<")";
        }
        utils::display(oss<<"\n\tsleep timer:", m_sleep_watchdog);
        return oss.str();
      }
      
    private:
      void start() {
        typename mutex_type::scoped_lock guard(m_lock);
        m_clock.reset(new clock_type);
        m_epoch = boost::posix_time::microsec_clock::universal_time();
        m_tick -= m_tick.time_since_epoch();
        update_sleep();
      }
      
      transaction::TICK getNextTick() {
        typename mutex_type::scoped_lock guard(m_lock);
        if( NULL!=m_clock.get() ) {
          typename clock_type::base_duration how_late = m_clock->to_next(m_tick, m_period);
          if( how_late >=clock_type::base_duration::zero() ) {
            double ratio = CHRONO::duration_cast< CHRONO::duration<double, Period> >(how_late).count();
            ratio /= m_period.count();
            if( ratio>=0.1 ) {
              // more than 10% of a tick late => display a warning
              std::ostringstream oss;
              utils::display(oss, how_late);
              syslog(warn)<<" clock is "<<oss.str()<<" late.";
            } 
            update_sleep();
          }
          return m_tick.time_since_epoch().count()/m_period.count();
        } else 
          return 0;
      }
      
      bool free() const {
        if( NULL!=m_clock.get() ) {
          typename clock_type::base_time_point t = clock_type::base_clock::now();
          if( t>=m_sleep ) {
            std::ostringstream oss;
            utils::display(oss, t-m_sleep);
            syslog(TREX::utils::log::info)<<"Sleep forced by clock ("
				     <<oss.str()<<" after watchdog)";
            return false;
          }
        }
        return true;
      }
      
      duration_type getSleepDelay() const {
        typename mutex_type::scoped_lock guard(m_lock);
        if( NULL!=m_clock.get() ) {
          typename clock_type::time_point target = m_tick + m_period;
          typename clock_type::base_duration left = m_clock->left(target);
          if( left >= clock_type::base_duration::zero() )
            return CHRONO::duration_cast<duration_type>(left);
          else {
            double ratio = CHRONO::duration_cast< CHRONO::duration<double, Period> >(-left).count();
            ratio /= m_period.count();
            if( ratio>=0.05 ) {
              // more than 5% of a tick late => display a warning
              std::ostringstream oss;
              utils::display(oss, -left);
              syslog(warn)<<" clock is "<<oss.str()<<" late before sleeping.";
            } 
            return duration_type(0);
          }
        } else 
          return Clock::getSleepDelay();
      }
    
      void check_tick() const {
        if( m_period <= tick_rate::zero() )
          throw TREX::utils::Exception("[clock] tick rate must be greater than 0");
      }
      
      typedef boost::recursive_mutex mutex_type;
      
      mutable mutex_type   m_lock;
      tick_rate            m_period;
      UNIQ_PTR<clock_type> m_clock;
      date_type            m_epoch;
      
      typename clock_type::base_duration    m_sleep_watchdog;
      typename clock_type::base_time_point m_sleep;
      
      
      
      typename clock_type::time_point m_tick;
    }; 
    
    typedef rt_clock<CHRONO_NS::milli> RealTimeClock;
    
    
  } // TREX::agent 
} // TREX

#endif // H_RealTimeClock
