/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, Frederic Py.
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
#include "external_handler.hh"

#include "../reactor_error.hh"
#include "../graph_error.hh"

using namespace TREX::transaction;
namespace utils=TREX::utils;
namespace tlog=utils::log;

using details::external_handler;
using details::date_type;
using utils::symbol;

namespace {
  inline bool is_less(boost::optional<details::date_type> const &a,
                      boost::optional<details::date_type> const &b) {
    return b && ( !a || (*a < *b));
  }
}


/*
 * class TREX::transaction::details::node_impl::external_handler
 */

// structors

external_handler::external_handler() {
}

external_handler::~external_handler() {
}

// observers

boost::optional<date_type> external_handler::initial() const {
  boost::shared_lock<boost::shared_mutex> read(m_mtx);
  return m_initial;
}


bool external_handler::should_synch() const {
  boost::shared_lock<boost::shared_mutex> read(m_mtx);
  return is_less(m_synch, m_graph) &&
    ( m_externals.empty() || is_less(m_synch, m_ext) );
}

bool external_handler::empty() const {
  boost::shared_lock<boost::shared_mutex> read(m_mtx);
  return m_externals.empty();
}

boost::optional<date_type> external_handler::synch_date() const {
  boost::shared_lock<boost::shared_mutex> read(m_mtx);
  return m_synch;
}

boost::optional<date_type> external_handler::next_synch() const {
  boost::shared_lock<boost::shared_mutex> read(m_mtx);
  if( !m_synch )
    return m_initial;
  else if( is_less(m_synch, m_graph) &&
           ( m_externals.empty() || is_less(m_synch, m_ext) ) )
    return 1+*m_synch;
  else
    return boost::optional<date_type>();
}

boost::optional<date_type> external_handler::synch_target() const {
  boost::shared_lock<boost::shared_mutex> read(m_mtx);
  if( m_externals.empty() || is_less(m_graph, m_ext) )
    return m_graph;
  else
    return m_ext;
}


boost::optional<date_type> external_handler::ext_date()   const {
  boost::shared_lock<boost::shared_mutex> read(m_mtx);
  return m_ext;
}

boost::optional<date_type> external_handler::graph_date() const {
  boost::shared_lock<boost::shared_mutex> read(m_mtx);
  return m_graph;
}


bool external_handler::external(symbol const &tl) const {
  boost::shared_lock<boost::shared_mutex> read(m_mtx);
  
  tl_map::const_iterator const endi = m_externals.end();
  tl_map::const_iterator i = m_externals.begin();
  
  for(; endi!=i && i->first->name()<tl; ++i);
  
  return endi!=i && i->first->name()==tl;
}

// modifiers

bool external_handler::insert(SHARED_PTR<external_impl> const &tl) {
  boost::optional<date_type> unknown;
  tl_map::value_type val(tl, unknown);
  bool inserted;
  tl_map::iterator pos;
  {
    boost::unique_lock<boost::shared_mutex> write(m_mtx);
    
    boost::tie(pos, inserted) = m_externals.insert(val);
    if( inserted )
      m_ext = unknown;
  }
  return inserted;
}

bool external_handler::update(symbol const &tl, date_type tick,
                              ERROR_CODE &ec) {
  bool update = true, un_held = false;
  ec = reactor_error_code(reactor_error::ok);
  
  // This algorithm assume that timeline ticks ar monotonically increasing
  {
    boost::upgrade_lock<boost::shared_mutex> test(m_mtx);
    
    tl_map::iterator const endi = m_externals.end();
    tl_map::iterator i = m_externals.begin();
    boost::optional<date_type> const tck = tick;
    
    // Search for the timleine and check if new tick is the smallest
    for(; endi!=i && i->first->name()<tl; ++i)
      if( is_less(i->second, tck) )
        update = false;
    
    if( endi==i || i->first->name()!=tl ) {
      ec = reactor_error_code(reactor_error::not_external);
      return false;
    } else {
      tl_map::iterator const pos = i;
      
      // Continue checking if tick is the new minimum
      for(++i; update && endi!=i; ++i)
        if( is_less(i->second, tck) )
          update = false;
      {
        boost::upgrade_to_unique_lock<boost::shared_mutex> write(test);
        pos->second = tck;
        if( update ) {
          un_held = is_less(m_synch, tck) && !is_less(m_synch, m_ext);
          m_ext = tck;
        }
      }
    }
  }  
  return un_held;
}


void external_handler::clear() {
  tl_map tmp;
  {
    boost::unique_lock<boost::shared_mutex> write(m_mtx);
    
    std::swap(tmp, m_externals);
    m_ext = m_graph;
  }
  tl_map::iterator i=tmp.begin();
  while( tmp.end()!=i ) {
    // TODO notify of the disconnect
    i = tmp.erase(i);
  }
}

bool external_handler::r_queue_tick(date_type date) {
  bool un_held = false;
  boost::optional<date_type> tck = date;
  {
    boost::upgrade_lock<boost::shared_mutex> test(m_mtx);
    
    if( m_externals.empty() )
      un_held = !is_less(m_synch, m_graph) && is_less(m_synch, tck);
    else
      un_held = is_less(m_ext, tck) && !is_less(m_synch, m_ext);
    {
      boost::upgrade_to_unique_lock<boost::shared_mutex> write(test);
      m_graph = tck;
      if( !m_initial )
        m_initial = tck;
    }
  }
  return un_held;
}

bool external_handler::r_queue_synchronized(date_type date, bool &changed) {
  bool re_post;
  changed = false;
  {
    boost::upgrade_lock<boost::shared_mutex> test(m_mtx);
    if( !m_synch || *m_synch<=date ) {
      boost::upgrade_to_unique_lock<boost::shared_mutex> write(test);
      m_synch = date;
      changed = true;
    }
    re_post = is_less(m_synch, m_graph);
    if( !m_externals.empty() && re_post )
      re_post = is_less(m_synch, m_ext);
  }
  return re_post;
}




