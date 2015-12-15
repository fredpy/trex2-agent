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
#include "node_impl.hh"
#include "graph_impl.hh"
#include "external_impl.hh"

#include <trex/utils/asio_runner.hh>

using namespace trex::transaction;
namespace utils=trex::utils;
namespace tlog=utils::log;

using utils::symbol;
using details::external_impl;

/*
 * class TREX::transaction::details::external_impl
 */

// structors

external_impl::external_impl(SHARED_PTR<details::node_impl> const &cli,
                             SHARED_PTR<details::internal_impl> const &tl,
                             details::transaction_flags gp)
:m_client(cli), m_timeline(tl), m_gp(gp) {}

external_impl::~external_impl() {
  m_client.reset();
}

// observers

bool external_impl::accept_goals() const {
  bool ret = m_timeline->accept_goals();
  if( ret ) {
    boost::shared_lock<boost::shared_mutex> read(m_mtx);
    ret = m_gp[0];
  }
  return ret;
}

bool external_impl::publish_plan() const {
  bool ret = m_timeline->publish_plan();
  if( ret ) {
    boost::shared_lock<boost::shared_mutex> read(m_mtx);
    ret = m_gp[1];
  }
  return ret;
}

// manipulators


tlog::stream external_impl::syslog(utils::symbol const &kind) const {
  SHARED_PTR<node_impl> r = m_client.lock();
  if( r )
    return r->syslog(name(), kind);
  else // this should never happen
    return m_timeline->syslog("!!"+kind.str());
}

token_ref external_impl::goal(symbol const &pred) const {
  token_ref tok = token::goal(name(), pred);
  boost::optional<date_type> cur = now();
  
  if( cur ) // goals necessarily end in the future
    tok->restrict_end(int_domain(1+*cur, int_domain::plus_inf));
  return tok;
}

// Graph strand events

void external_impl::g_strand_connect() {
  SHARED_PTR<node_impl> r = m_client.lock();

  if( r ) {
    internal_impl::synch_event::extended_slot_type
    slot(&external_impl::g_strand_obs, this, _1, _2, _3);
    slot.track_foreign(shared_from_this());
    m_timeline->on_synch().connect_extended(slot.track_foreign(m_client));
    bool fresh = false;
    date_type tick;
    token_id  obs;
    {
      boost::unique_lock<boost::shared_mutex> write(m_mtx);
      m_last_tick = m_timeline->last_synch(m_state);
      if( m_last_tick ) {
        fresh = true;
        tick = *m_last_tick;
        obs = m_state;
      }
    }
    if( fresh )
      r->g_strand_notify(tick, obs, fresh);
  }
}


void external_impl::g_strand_obs(boost::signals2::connection const &c,
                                 date_type tick, token_id obs) {
  SHARED_PTR<node_impl> r = m_client.lock();
  if( r ) {
    bool fresh = false;
    {
      boost::unique_lock<boost::shared_mutex> write(m_mtx);
      m_last_tick = tick;
      if( obs!=m_state ) {
        fresh = true;
        m_state = obs;
      }
    }
    r->g_strand_notify(tick, obs, fresh);
  } else
    c.disconnect();
}






