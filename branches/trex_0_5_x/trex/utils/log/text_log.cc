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

#include "log_pipe.hh"
#include "out_file.hh"
#include "bits/log_stream.hh" 

#include "../platform/chrono.hh"

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
    SHARED_PTR<details::sig_impl> dest = m_log.lock();
  
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


void out_file::flush() {
  std::flush(*m_file);
}


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
    (*m_file)<<msg->content()<<std::endl; // change this to dt::endl if you awnt to flush
  }
}

/*
 * class TREX::util::log_pipe
 */

// structors 

log_pipe::log_pipe(text_log &log, std::ostream &os, 
		   TREX::utils::Symbol const &who,
                   TREX::utils::Symbol const &what)
  :m_who(who), m_what(what), m_log(log.m_new_log), m_dest(os), m_flush(log.service()) {
  m_initial = m_dest.rdbuf();
  m_me = new boost::iostreams::stream_buffer<pipe_sink>(*this);
  m_dest.rdbuf(m_me);
}


log_pipe::~log_pipe() {
  m_flush.cancel();
  m_dest.rdbuf(m_initial);
  delete m_me;
  if( !m_msg.empty() )
    send(m_msg);
}
	
std::streamsize log_pipe::pipe_sink::write(log_pipe::pipe_sink::char_type const *s,
                                          std::streamsize n) {
  m_master->m_flush.cancel();
  std::streamsize ret = m_master->m_initial->sputn(s, n);
  if( ret>0 ) {
    m_master->m_msg.append(s, ret);
    m_master->flush_msg();
  }
  return ret;
}

void log_pipe::flush_to() {
  std::string tmp;
  std::swap(tmp, m_msg);
  if( !tmp.empty() )
    send(tmp);
}


void log_pipe::flush_msg() {
  if( !m_msg.empty() ) {
    std::string::size_type last = m_msg.rfind('\n');
    if( last!=std::string::npos ) {
      if( last>0 )
        send(m_msg.substr(0, last));
      m_msg.erase(0, last+1);
    }
    if( !m_msg.empty() ) {
      m_flush.expires_from_now(boost::posix_time::milliseconds(100));
      m_flush.async_wait(boost::bind(&log_pipe::flush_to, this));
    }
  }
}

void log_pipe::send(std::string const &msg) {
  SHARED_PTR<details::sig_impl> dest = m_log.lock();
  if( dest ) {
    SHARED_PTR<entry> e(new entry(m_who, m_what));
    e->m_content = msg;
    dest->emit(e);
  }
}
