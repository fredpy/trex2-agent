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
#ifndef H_trex_utils_bits_stranded_service
# define H_trex_utils_bits_stranded_service

# include <trex/config/memory.hh>
# include <trex/config/bits/asio_conf.hh>

# include <boost/asio/io_service.hpp>

namespace TREX {
  namespace utils {
    
    template<class Worker>
    class stranded_service :public boost::asio::io_service::service {
    public:
      static boost::asio::io_service::id id;
      typedef SHARED_PTR<boost::asio::io_service::work> work_ref;
      
      explicit stranded_service(boost::asio::io_service &io)
      :boost::asio::io_service::service(io),m_strand(io) {}
      
      ~stranded_service() {
        shutdown_service();
      }
      
      template<typename Handler>
      void post(Handler const &fn) {
        m_strand.post(fn);
      }
      template<typename Handler>
      void dispatch(Handler const &fn) {
        m_strand.dispatch(fn);
      }
      
      work_ref reserve() {
        work_ref ret = m_work.lock();
        if( !ret ) {
          boost::asio::io_service &io = get_io_service();
          ret = MAKE_SHARED<boost::asio::io_service::work>(boost::ref(io));
          m_work = ret;
        }
        return ret;
      }
      
      template<typename Handler>
      void async_reserve(Handler fn) {
        boost::function<void (work_ref)> handle(fn);
        post(boost::bind(&stranded_service::reserve_sync, this, handle));
      }
      
      void reserve_sync(boost::function<void (work_ref)> handle) {
        handle(reserve());
      }
      
    private:
      void shutdown_service() {}
      
      WEAK_PTR<boost::asio::io_service::work> m_work;
      boost::asio::io_service::strand m_strand;
    };
    
    template<class Worker>
    boost::asio::io_service::id stranded_service<Worker>::id;
    
  } // TREX::utils
} // TREX

#endif // H_trex_utils_bits_stranded_service