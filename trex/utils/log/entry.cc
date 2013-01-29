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
#include "bits/log_stream.hh"
#include "text_log.hh"

using namespace TREX::utils::log;

/*
 * class TREX::utils::log::details::entry_sink
 */

// structors

details::entry_sink::~entry_sink() {
  if( NULL!=m_log && m_entry && m_entry->has_content() )
    (*m_log)(m_entry);
}

// manipulators

std::streamsize details::entry_sink::write(details::entry_sink::char_type const *s, 
                                           std::streamsize n) {
  if( m_entry )
    return m_entry->write(s, n);
  return 0;
}

/*
 * class TREX::utils::log::stream
 */
// structors 

stream::stream(details::entry_sink const &dest)
:m_out(new details::stream_impl(dest)) {}

// manipulators 

std::ostream &stream::get_stream() {
  return *m_out;
}

/*
 * class TREX::utils::log::entry
 */

// modifiers

entry::size_type entry::write(entry::char_type const *s, entry::size_type n) {
  if( n>0 ) {
    if( m_pending_nl )
      m_content.push_back('\n');
    m_pending_nl = (s[n-1]=='\n');
    m_content.append(s, m_pending_nl?(n-1):n);
  }
  return n;
}
