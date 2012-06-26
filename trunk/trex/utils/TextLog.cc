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
    m_dest.send(m_source, m_kind, m_msg);
  }
}

/*
 * class TREX::utils::TextLog::handler
 */

// manipulators

void TextLog::handler::detach() {
  if( NULL!=m_log ) {
    TextLog::scoped_lock lock(m_log->m_lock);
    m_log->m_handlers.erase(this);
    m_log = NULL;
  }
}

/*
 * class TREX::utils::TextLog
 */

// structors

TextLog::~TextLog() {
  scoped_lock lock(m_lock);
  for(std::set<handler *>::const_iterator i=m_handlers.begin();
      m_handlers.end()!=i; ++i) 
    (*i)->m_log = NULL;
  m_handlers.clear();
}

// manipulators 

bool TextLog::add_handler(TextLog::handler &handle) {
  if( NULL==handle.m_log ) {
    scoped_lock lock(m_lock);
    handle.m_log = this;
    m_handlers.insert(&handle);
    return true;
  }
  return false;
}

void TextLog::send(TextLog::id_type const &who, TextLog::id_type const &type, 
		   TextLog::msg_type const &what) {
  scoped_lock lock(m_lock);
  for(std::set<handler *>::const_iterator i=m_handlers.begin(); 
      m_handlers.end()!=i; ++i)
    (*i)->message(who, type, what);
}
