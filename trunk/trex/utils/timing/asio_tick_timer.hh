/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, Frederic Py
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
#ifndef H_trex_utils_timing_asio_tick_timer
# define H_trex_utils_timing_asio_tick_timer

# include "tick_clock.hh"
# include "bits/asio_chrono_timer.hh"

namespace TREX {
  namespace utils {
    
    template<class Clock>
    class asio_tick_timer {
      typedef typename Clock::base_clock      base_clock;
      typedef typename Clock::base_duration   base_duration;
    public:
      typedef typename Clock::base_time_point base_time_point;
      
      typedef typename Clock::duration   duration_type;
      typedef typename Clock::time_point time_type;
      
      explicit asio_tick_timer(boost::asio::io_service &io)
      :m_timer(io) {}
      asio_tick_timer(boost::asio::io_service &io,
                      base_time_point const &epoch)
      :m_timer(io), m_clock(epoch) {}
      
      template<typename WaitHandler>
      void async_wait(WaitHandler handle) {
        m_timer.async_wait(handle);
      }
      std::size_t cancel() {
        m_timer.cancel();
      }
      std::size_t cancel(boost::system::error_code &ec) {
        m_timer.cancel(ec);
      }
      std::size_t cancel_one() {
        m_timer.cancel_one();
      }
      std::size_t cancel_one(boost::system::error_code &ec) {
        m_timer.cancel_one(ec);
      }
      time_type expires_at() const {
        base_duration delta = m_timer.expires_at()-m_clock.epoch();
        return time_type(CHRONO::duration_cast<duration_type>(delta));
      }
      std::size_t expires_at(time_type const &expiry_time) {
        duration_type ticks = expiry_time.time_since_epoch();
        return m_timer.expires_at(m_clock.epoch()+CHRONO::duration_cast<base_duration>(ticks));
      }
      std::size_t expires_at(time_type const &expiry_time,
                             boost::system::error_code &ec) {
        duration_type ticks = expiry_time.time_since_epoch();
        return m_timer.expires_at(m_clock.epoch()+CHRONO::duration_cast<base_duration>(ticks), ec);
      }
      duration_type expires_from_now() const {
        return CHRONO::duration_cast<duration_type>(m_timer.expires_from_now());
      }
      std::size_t expires_from_now(duration_type const &expiry_time) {
        return expires_at(m_clock.now()+expiry_time);
      }
      std::size_t expires_from_now(duration_type const &expiry_time,
                                   boost::system::error_code &ec) {
        return expires_at(m_clock.now()+expiry_time, ec);
      }
      boost::asio::io_service &get_io_service() {
        return m_timer.get_io_service();
      }
      void wait() {
        m_timer.wait();
      }
      void wait(boost::system::error_code &ec) {
        m_timer.wait(ec);
      }
      
      Clock const &clock() const {
        return m_clock;
      }
      
    private:
      typedef internals::tick_clock_traits<base_clock> base_traits;
      typedef boost::asio::basic_deadline_timer<base_clock, base_traits> timer;
      
      timer m_timer;
      Clock m_clock;
    };
    
  }
}


#endif // H_trex_utils_timing_asio_tick_timer
