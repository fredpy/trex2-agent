/* -*- C++ -*- */
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
#ifndef H_trex_utils_log_sig
# define H_trex_utils_log_sig

# include "../entry.hh"

# include <boost/signals2.hpp>
# include <boost/asio.hpp>

namespace TREX {
  namespace utils {
    namespace log {
      namespace details {
        
        namespace bs2=boost::signals2;
    
        typedef bs2::signal<void (entry::pointer)>      log_signal; 
        typedef log_signal::extended_slot_function_type log_handle_fn;
        
        class local_protected_cb {
        public:
          typedef log_signal::result_type   result_type;
          typedef bs2::connection           first_argument_type;
          typedef log_signal::argument_type second_argument_type; 
          
          template<typename Handler>
          explicit local_protected_cb(Handler cb):m_fn(cb) {}
          ~local_protected_cb() {};
          
          
          void operator()(bs2::connection const &c,
                          entry::pointer e);
          
        private:
          boost::function<void (entry::pointer)> m_fn;
        };

        class async_cb_base {
        public:
          typedef local_protected_cb::result_type          result_type;
          typedef local_protected_cb::first_argument_type  first_argument_type;
          typedef local_protected_cb::second_argument_type second_argument_type;
          
          template<typename Handler>
          async_cb_base(Handler fn):m_callback(fn) {}
          ~async_cb_base() {}

        protected:
          boost::function<void ()> wrap(first_argument_type c,
                                        second_argument_type msg);
         
        private:
          local_protected_cb m_callback;
        };
        
        template<class Service, typename ServicePtr=Service *>
        class async_cb :public async_cb_base {
        public:
          typedef Service service_type;
          
          template<typename Handler>
          async_cb(service_type &s, Handler cb):async_cb_base(cb),
          m_service(&s) {}
          
          template<class ServicePointer, typename Handler>
          async_cb(ServicePointer s, Handler cb):async_cb_base(cb),
          m_service(s) {}
          
          
          void operator()(first_argument_type const &c,
                          second_argument_type const &msg) {
            service().post(wrap(c, msg));
          }
          
          service_type &service() {
            return *m_service; 
          }
          
        private:
          ServicePtr m_service;
        };
        
      } // TREX::utils::log::details
      
      namespace handler {
        
        template<typename Handler>
        details::log_handle_fn direct(Handler fn) {
          return details::local_protected_cb(fn);
        }
        
        template<class Service, typename Handler>
        details::log_handle_fn async(Service &async,Handler fn) {
          return details::async_cb<Service>(async, fn);
        }
        
        template<typename Handler>
        details::log_handle_fn stranded(boost::asio::io_service &io, Handler fn) {
          return details::async_cb<boost::asio::io_service::strand, 
          boost::shared_ptr<boost::asio::io_service::strand> >(new boost::asio::io_service::strand(io), fn);
        }
        
        
      } // TREX::utils::log::handler
    } // TREX::utils::log
  } // TREX::utils
} // TREX

#endif // H_trex_utils_log_sig
