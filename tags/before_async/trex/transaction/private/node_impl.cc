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
#include "external_impl.hh"

using namespace TREX::transaction::details;
using namespace TREX;

namespace tlog=TREX::utils::log;
namespace bs2=boost::signals2;

using TREX::utils::symbol;
using TREX::transaction::TICK;

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

boost::optional<TICK> node_impl::external_date() const {
  boost::shared_lock<boost::shared_mutex> lock(m_mutex);
  return m_external_date;
}


boost::optional<TICK> node_impl::internal_date() const {
  boost::shared_lock<boost::shared_mutex> lock(m_mutex);
  return m_internal_date;
  
}

boost::optional<TICK> node_impl::current_tick() const {
  boost::shared_lock<boost::shared_mutex> lock(m_mutex);
  return m_execution_frontier;
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

void node_impl::synchronized(TICK date) {
  // I probably shoudl make this occur in the graph strand
  bool updated = false;
  {
    boost::upgrade_lock<boost::shared_mutex> test(m_mutex);
  
    // Not ideally I should enforce that
    // m_internal_date < date <= m_external_date <= m_execution_frontier
    
    if( !m_internal_date || *m_internal_date<date ) {
      boost::upgrade_to_unique_lock<boost::shared_mutex> lock(test);
      m_internal_date = date;
    }
  }
  if( updated ) {
    // TODO notify all the internal timelines of their update
    
  }
}


bool node_impl::use(symbol const &tl, transaction_flags fl) {
  SHARED_PTR<graph_impl> g = m_graph.lock();
  
  if( g ) {
    SHARED_PTR<internal_impl> obj = g->get_tl(tl).lock();
    if( obj ) {
      SHARED_PTR<external_impl>
        ext = MAKE_SHARED<external_impl>(shared_from_this(),
                                         obj, fl);
      // TODO: attach it to the node
      return bool(ext);
    }
  }
  return false;
}


bool node_impl::provide(symbol const &tl, transaction_flags fl) {
  SHARED_PTR<graph_impl> g = m_graph.lock();
  
  if( g ) {
    // TODO: declare_tl need to associate this timeline to this node
    WEAK_PTR<internal_impl> obj = g->declare_tl(tl, shared_from_this(), fl);
    return bool(obj.lock());
  }
  return false;
}


// manipulators

utils::log_manager &node_impl::manager() const {
  SHARED_PTR<graph_impl> g = m_graph.lock();

  if( g )
    return g->manager();
  else
    return *s_log;
}

tlog::stream node_impl::syslog(symbol const &ctx,
                               symbol const &kind) const {
  SHARED_PTR<graph_impl> g = m_graph.lock();
  symbol who = name();
  
  
  if( !ctx.empty() )
    who = who.str()+"."+ctx.str();
  
  boost::optional<TICK> now = internal_date();
  
  if( g ) {
    if( now && *now<(*g->date(true))) {
      std::ostringstream oss;
      oss<<who<<"]["<<*now;
      who = oss.str();
    }
    
    return g->syslog(who, kind);
  } else {
    // handle the situation where the node is no longer attached to a graph
    who = "(nil)."+who.str();
    if( now )
      return s_log->syslog(*now, who, kind);
    else
      return s_log->syslog(who, kind);
  }
}

void node_impl::notify(TICK date, utils::symbol const &tl,
                       token_id const &o) {
  SHARED_PTR<graph_impl> g = m_graph.lock();
  if( g ) {
    // TODO update this timeline observation, notify the reactor,
    // and check if m_external_date has been updated
  } else {
    // TODO disconnect
  }
}


// Executed by graph strand

void node_impl::tick(bs2::connection const &c, date_type const &date) {
  SHARED_PTR<graph_impl> g = m_graph.lock();
  if( g ) {
    boost::unique_lock<boost::shared_mutex> lock(m_mutex);
    m_execution_frontier = date;
  } else
    c.disconnect();
}
