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

#include "text_log.hh"
#include "out_file.hh"
#include "bits/log_stream.hh" 

#include <boost/smart_ptr.hpp>
#include <boost/signals2/shared_connection_block.hpp>
#include <boost/signals2/signal.hpp>

namespace bs2=boost::signals2;

namespace TREX {
  namespace utils {
    namespace log {
      
      id_type const null;
      id_type const info("INFO");
      id_type const warn("WARNING");
      id_type const error("ERROR");
      
      namespace details {
       
        class sig_impl {
        public:
          typedef slot::signature_type signature_type;
          typedef bs2::signal<signature_type, bs2::optional_last_value<void>,
                              int, std::less<int>, slot::slot_function_type,
                              ext_slot::slot_function_type> signal_type;
          
          
          inline sig_impl() {}
          inline ~sig_impl() {}
          
          inline void emit(entry::pointer const &e) {
            m_signal(e);
          }
          
          inline connection connect(slot const &s) {
            return m_signal.connect(s);
          }
          inline connection ext_connect(ext_slot const &s) {
            return m_signal.connect_extended(s);
          }
          
        private:
          signal_type m_signal;
        };
        
        
      }
    }
  }
}

using namespace TREX::utils::log;

/*
 * class TREX::utils::log::details::entry_sink
 */

// structors

details::entry_sink::~entry_sink() {
  if( m_entry && m_entry->has_content() ) {
    // Get the pointer to the signal
    boost::shared_ptr<details::sig_impl> dest = m_log.lock();
  
    if( dest ) // if the signal still exists then trigger it
      dest->emit(m_entry);
  }
}

/*
 * class TREX::utils::log::text_log
 */

// structors 

text_log::text_log(boost::asio::io_service &io)
  :m_new_log(new details::sig_impl), m_service(io) {}

// manipulators

stream text_log::msg(id_type const &who, id_type const &what) {
  return stream(details::entry_sink(m_new_log, who, what));
}

stream text_log::msg(date_type const &when, 
                     id_type const &who, id_type const &what) {
  return stream(details::entry_sink(m_new_log, when, who, what));
}

text_log::connection text_log::direct_connect(text_log::slot_type const &cb) {
  return m_new_log->connect(cb);
}

text_log::connection text_log::direct_connect_extended(text_log::extended_slot_type const &cb) {
  return m_new_log->ext_connect(cb);
}


/*
 * class TREX::utils::log::out_file
 */

out_file::out_file(std::string const &name)
 :m_file(new std::ofstream(name.c_str())) {} // should use make_shared instead but
                                             // it conflicts when boost used with
                                             // gcc with c++11 support

void out_file::operator()(entry::pointer msg) {
  if( m_file && *m_file) {
    bool prefixed = false;
    
    if( msg->is_dated() ) {
      prefixed = true;
      (*m_file)<<'['<<msg->date()<<']';
    }
    if( !msg->source().empty() ) {
      prefixed = true;
      (*m_file)<<'['<<msg->source()<<']';
    }
    if( msg->kind()!=null && msg->kind()!=info ) 
      (*m_file)<<msg->kind()<<": ";
    else if( prefixed )
      m_file->put(' ');
    (*m_file)<<msg->content()<<std::endl;
  }
}

