/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, MBARI.
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

using namespace TREX::transaction::details;
using namespace TREX;

namespace tlog=TREX::utils::log;
namespace bs2=boost::signals2;

using TREX::utils::symbol;

namespace {
  
  utils::singleton::use<utils::log_manager> s_log;

}

/*
 * class TREX::transaction::details::node_impl
 */

// structors

node_impl::node_impl(WEAK_PTR<graph_impl> const &g)
:m_graph(g) {}

node_impl::~node_impl() {}

// observers

symbol const &node_impl::name() const {
  return m_name;
}

SHARED_PTR<graph_impl> node_impl::graph() const {
  return m_graph.lock();
}


// modifiers

void node_impl::set_name(symbol const &n) {
  if( m_name.empty() )
    m_name = n;
}

void node_impl::reset() {
  // disconnect from the graph
  m_graph.reset();
}

// manipulators

tlog::stream node_impl::syslog(symbol const &ctx,
                               symbol const &kind) const {
  SHARED_PTR<graph_impl> g = m_graph.lock();
  symbol who = name();
  
  if( !ctx.empty() )
    who = who.str()+"."+ctx.str();
  
  if( g )
    return g->syslog(who, kind);
  else {
    // handle the situation where the node is no longer attached to a graph
    who = "(nil)."+who.str();
    return s_log->syslog(who, kind);
  }
}

void node_impl::notify(TICK date, utils::symbol const &tl,
                       boost::optional<Observation> const &o) {
  // TODO report to the node through a signal
}


// Executed by graph strand

void node_impl::tick(bs2::connection const &c, date_type const &date) {
  SHARED_PTR<graph_impl> g = m_graph.lock();
  if( g ) {
    syslog(tlog::null, tlog::info)<<"Tick("<<date<<")";
  } else
    c.disconnect();
}
