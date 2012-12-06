/* -*- C++ -*- */
/** @file "trex/utils/log/text_log.hh" 
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
#ifndef H_trex_utils_log_text_log
# define H_trex_utils_log_text_log

# include "entry.hh"
# include "stream.hh"

# include "bits/log_sig.hh"
# include "../platform/cpp11_deleted.hh"

namespace TREX {
  namespace utils {
    namespace log {

      /** @brief Asynchronous text messages logging
       *
       * This class implements the logging mechanism used within trex to 
       * create log entries. entries are creted through a temporary stream 
       * which at its destruction signals its content that can then be 
       * handled asynchronously by the callbacks associated.
       *
       * This allow to have a generic way to redirect log messages created 
       * to any kind of consumer such as an output file, a graphic interface, 
       * a database, ...
       *
       * The handling is done asynchronously using @asio and, by default, 
       * connected callbacks are wrapped in a strand which ensure that this 
       * callback cannot be executed more than once at any time -- ensuring 
       * thread safetyness.
       *
       * @author Frederic Py <fpy@mbari.org>
       * @ingroup utils
       * @sa class TREX::utils::log::entry
       */
      class text_log :boost::noncopyable {
      public:
        typedef entry::date_type                        date_type;
        typedef entry::id_type                          id_type;
        typedef details::log_signal::extended_slot_type slot_type;
        
        /** @brief Constructor 
         *
         * @param[in] io A reference to an @asio service
         *
         * Create a new instance with @p io as the default service to execute 
         * its handler.
         */
        explicit text_log(boost::asio::io_service &io);
        /** @brief Destructor */
        ~text_log() {}
        
        /** @{
         * @brief New entry
         *
         * @param[in] when An (optional) date
         * @param[in] who  A symbol describing the source of the message
         * @param[in] kind A symbol describing the kind of message 
         *
         * Create a new local stream to build a new entry. The entry will be
         * with @p who as the source the entry, with the message type @p what 
         * and -- if provided -- dated as @p when. The returned stream will 
         * allow to create the entry content for this message.
         *
         * @return A stream used to build and dispatch on its destruction the 
         * newly created entry.
         *
         * For example one can create a new wraning message as follow:
         * @code
         * using namespace TREX::utils;
         * // ... 
         * log::text_log my_log(io)
         * // ...
         * my_log(0, "a fancy source tag", log::info)<<"This my message";
         * @endcode
         * At the end of this last line the temporary stream will be destoyed 
         * resulting on the new @e info log entry dated @c 0 from 
         * "a fancy source tag" with the content "This is my message" will be 
         * dispatched from a signal to all the handlers which will in turn be 
         * posted to execute and process this entry.       
         */
        stream msg(id_type const &who, id_type const &what=null);
        stream operator()(id_type const &who, id_type const &what=null) {
          return msg(who, what);
        }
        stream msg(date_type const &when, 
                   id_type const &who, id_type const &what=null);
        stream operator()(date_type const &when, 
                          id_type const &who, id_type const &what=null) {
          return msg(when, who, what); 
        }
        /** @} */
        
        /** @{ 
         * @brief Asynchronous log handler wrapping
         *
         * @tparam Handler A handling functor
         * @tparam Service A @asio "service"
         * 
         * @param[in] fn The handler to be called
         * @param[in] s The service used
         *
         * Wrap @p fn as an asynchronous slot using the service @p s.
         * These slot created, if connected will post the call of @p fn  
         * through @p s service whenever a new log entry is published by 
         * this class.
         *
         * If @p s is an asynchronous service (such as a io_service or a 
         * io_service::strand) the posting will just schedulle the handler 
         * execution in one of the threads executing this service and this 
         * call will return immediately 
         * 
         * @pre @p Service has a post function that accept functors with no 
         * argument
         * @pre @p Handler is copyable
         * @pre @p Handler accept a log::entry::pointer as argument and return 
         *      @c void 
         *
         * @return A slot that wraps the handler and will post its execution 
         *        through @p s when signaled of a new event to the text_log(s) 
         *        it is connected to
         *
         * @note when no service is specified this call create a strand attached 
         *       to the default @asio service associated to this instance
         * @sa text_log::post
         * @sa text_log::direct_wrap
         */
        template<typename Handler, class Service>
        slot_type wrap(Handler fn, Service &s) {
          return handler::async(s, fn);
        }
        template<typename Handler>
        slot_type wrap(Handler fn) {
          return handler::stranded(m_io, fn);
        }
        /** @} */
        /** @brief Direct log handler wrapping 
         *
         * @tparam Handler A handling functor
         * 
         * @param[in] fn The handler to be called
         *
         * Create a new log event slot for the handler @p fn. As opposed to warp 
         * this handler execution is not asynchronous which means that id will be 
         * direcly executed when a new log entry is porduced within the thread 
         * that created this event. Still this handler is protected in the sense 
         * that it connection is temporarilly disbaled during @p fn execution
         *
         * @pre @p Handler is copyable
         * @pre @p Handler accept a log::entry::pointer as argument and return 
         *      @c void 
         *
         * @return A slot that wraps the handler and will execute immediately 
         * 
         * @note As this handler is direct the execution of @p fn will not be 
         *       thread safe and it is the resposibilty of the implement of @p 
         *       fn to ensure its reentrancy.
         *
         * 
         */
        template<typename Handler>
        slot_type direct_wrap(Handler fn) {
          return handler::direct(fn);
        }
        
        boost::signals2::connection connect(slot_type const &slot);
        
        template<typename Handler, class Service>
        boost::signals2::connection connect(Handler fn, Service &s) {
          return connect(wrap(fn, s));
        }
        template<typename Handler>
        boost::signals2::connection connect(Handler fn) {
          return connect(wrap(fn));
        }
        template<typename Handler>
        boost::signals2::connection connect_direct(Handler fn) {
          return connect(wrap_direct(fn));
        }
        
      private:
        void post(entry::pointer const &msg);
        
        boost::asio::io_service &m_io;
        details::log_signal m_new_log; 
       
        friend class TREX::utils::log::details::entry_sink;
# ifndef DOXYGEN
        text_log() DELETED; // Non default constructible
# endif
      }; // class TREX::utils::log::text_log
      
    }
  }
}

#endif // H_trex_utils_log_log
