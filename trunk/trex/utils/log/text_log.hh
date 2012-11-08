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

namespace TREX {
  namespace utils {
    namespace log {

      class text_log :boost::noncopyable {
      public:
        typedef entry::date_type                        date_type;
        typedef entry::id_type                          id_type;
        typedef details::log_signal::extended_slot_type slot_type;
        
        explicit text_log(boost::asio::io_service &io);
        ~text_log() {}
        
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
        
        template<typename Handler>
        slot_type wrap(Handler fn) {
          return handler::stranded(m_io, fn);
        }
        template<typename Handler>
        slot_type wrap(Handler fn, boost::asio::io_service::strand &s) {
          return handler::async(s, fn);
        }
        
        boost::signals2::connection connect(slot_type const &slot);
        
        template<typename Handler>
        boost::signals2::connection connect(Handler fn) {
          return connect(wrap(fn));
        }
        template<typename Handler>
        boost::signals2::connection connect(Handler fn,
                                            boost::asio::io_service::strand &s) {
          return connect(wrap(fn, s));
        }
        
      private:
        void post(entry::pointer const &msg);
        
        boost::asio::io_service &m_io;
        details::log_signal m_new_log; 
       
        friend class TREX::utils::log::details::entry_sink;
# ifndef DOXYGEN
        text_log(); // Non default constructible
# endif
      }; // class TREX::utils::log::text_log
      
    }
  }
}

#endif // H_trex_utils_log_log