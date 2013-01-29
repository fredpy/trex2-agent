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
       * to any kind of consumer such as an output file, a graphic 
       * interface, a database, ...
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
        /** @brief Date type
         *
         * Type used to store the date of the message
         */
        typedef entry::date_type                        date_type;
        /** @brief Message Kind type
         *
         * Type used to store the kind of message produced
         */
        typedef entry::id_type                          id_type;
        
        typedef details::log_signal              log_signal;
        
        /** @brief Signal slot type
         *
         * The type of slot that can handle new log message signal events
         *
         * @sa class details::log_signal
         */
        typedef log_signal::slot_type          slot_type;
        typedef log_signal::extended_slot_type extended_slot_type;
        typedef log_signal::connection         connection;
       
        typedef log_signal::strand             strand;
        
        
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
         * with @p who as the source the entry, with the message type 
         * @p what and -- if provided -- dated as @p when. The returned 
         * stream will allow to create the entry content for this message.
         *
         * @return A stream used to build and dispatch on its destruction 
         * the newly created entry.
         *
         * For example one can create a new wraning message as follow:
         * @code
         * using namespace TREX::utils;
         * // ... 
         * log::text_log my_log(io)
         * // ...
         * my_log(0, "a fancy source tag", log::info)<<"This my message";
         * @endcode
         * At the end of this last line the temporary stream will be 
         * destoyed resulting on the new @e info log entry dated @c 0 from
         * "a fancy source tag" with the content "This is my message" will 
         * be dispatched from a signal to all the handlers which will in 
         * turn be posted to execute and process this entry.       
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
        
        strand new_strand() {
          return m_new_log.new_strand();
        }
        
         /** @brief Signal handler connection
         *
         * @param[in] slot A signal handler
         *
         * Connect @p slot to this instance. Whenever a new log message 
         * is produced @p slot will be called
         *
         * @return The connection beteween @p slot and this instance
         */
        connection connect(slot_type const &slot);
        connection connect(strand &s, slot_type const &slot);
        connection connect_extended(extended_slot_type const &slot);
        connection connect_extended(strand &s, extended_slot_type const &slot);

        
        connection async_connect(slot_type const &slot);
        connection async_connect_extended(extended_slot_type const &slot);
        
      private:
        log_signal m_new_log;
       
# ifndef DOXYGEN
        text_log() DELETED; // Non default constructible
# endif
      }; // class TREX::utils::log::text_log
      
    }
  }
}

#endif // H_trex_utils_log_log
