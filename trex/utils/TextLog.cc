/* -*- C++ -*- */
/** @file "TextLog.cc"
 * @brief TextLog implementation
 *
 * @author Frederic Py <fpy@mbari.org>
 * @ingroup utils
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
#include "TextLog.hh"

namespace TREX {
  namespace utils {

    Symbol const null;
    Symbol const info("INFO");
    Symbol const warn("WARN");
    Symbol const error("ERROR");

  }
}

using namespace TREX::utils;

/*
 * class TREX::utils::internals::entry
 */
// structors 

internals::entry::~entry() {
  // filter out empty messages
  if( !m_msg.empty() ) {
    // add trailing \n
    if( '\n'!=m_msg[m_msg.length()-1] )
      m_msg.push_back('\n');
    m_dest.send(m_date, m_source, m_kind, m_msg);
  }
}

/*
 * class TREX::utils::TextLog::handler
 */

TextLog::handler::~handler() {
  detach(true);
}

// manipulators

bool TextLog::handler::detach() {
  return detach(false);
}

bool TextLog::handler::detach(bool force) {
  if( NULL!=m_log ) {
    if( m_log->m_primary.get()!=this ) {
      TextLog::scoped_lock guard(m_log->m_lock);
      m_log->m_handlers.erase(this);
      m_log = NULL;
      return true;
    } else if( force ) {
      // Not thread safe
      m_log->m_primary.release();
      m_log = NULL;
      return true;
    }
  }
  return false;
}

/*
 * class TREX::utils::TextLog
 */

// structors

TextLog::~TextLog() {
  stop();
  if( NULL!=m_thread.get() ) {
    m_thread->join();
  }
  {
    scoped_lock guard(m_lock);
    while( !m_handlers.empty() ) {
      std::set<handler *>::iterator i = m_handlers.begin();
      std::auto_ptr<handler> tmp(*i);
      tmp->m_log = NULL;
      m_handlers.erase(i);
    }
  }
}

// manipulators 

void TextLog::send(boost::optional<TextLog::date_type> const &when,
		   TextLog::id_type const &who, TextLog::id_type const &type, 
		   TextLog::msg_type const &what) {
  // Not thread safe
  if( NULL!=m_primary.get() )
    m_primary->message(when, who, type, what);
  // end not thread safe
  if( is_running() ) {
    {
      queue_type::scoped_lock guard(m_queue);
      m_queue->push_back(packet(when, who, type, what));
    }
    m_have_message.notify_one();
  }
} 

/*
 *  class TREX::utils::TextLog::thread_proxy
 */

void TextLog::thread_proxy::operator()() {
  if( NULL!=m_log ) {
    {
      TextLog::scoped_lock guard(m_log->m_lock);
      m_log->m_running = true;
    }
    try {
      while( m_log->is_running() ) {
	boost::thread::yield(); 
	TextLog::packet msg;
	if( next(msg) ) {
	  TextLog::scoped_lock guard(m_log->m_lock);
	  if( m_log->m_handlers.empty() )
	    break; // do not run if no listeners
	  for(TextLog::handler_set::const_iterator i=m_log->m_handlers.begin();
	      m_log->m_handlers.end()!=i; ++i)
	    (*i)->message(msg.get<0>(), msg.get<1>(), 
			  msg.get<2>(), msg.get<3>()); 
	}/* else {
	  // sleep a little every 50ms is more than enough
	  boost::this_thread::sleep(boost::posix_time::milliseconds(50));
	  } */
      }
    } catch(...) {}
    m_log->stop();
  }
}

bool TextLog::thread_proxy::next(TextLog::packet &msg) {
  boost::unique_lock<queue_type> lock(m_log->m_queue);
  
  while( m_log->m_queue->empty() ) {
    m_log->m_have_message.wait(lock);
    if( !m_log->is_running() )
      return false;
  }
  msg = m_log->m_queue->front();
  m_log->m_queue->pop_front();
  return true;
}


