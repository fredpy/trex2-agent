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

#include <trex/utils/asio_runner.hh>

using namespace TREX::transaction;
namespace utils=TREX::utils;
namespace tlog=utils::log;

using utils::symbol;

namespace {
  // A reference to the LogManager singleton. So I can silently handle cases where a node
  // is not connected to a graph
  utils::singleton::use<utils::log_manager> s_log;
}

/*
 * class TREX::transaction::details::node_impl
 */

// structors

details::node_impl::node_impl(WEAK_PTR<details::graph_impl> const &g):m_graph(g) {
}

details::node_impl::~node_impl() {
}

// observers

bool details::node_impl::internal(symbol const &tl) const {
  SHARED_PTR<graph_impl> g = graph();
  if( g ) {
    boost::function<bool ()> fn(boost::bind(&node_impl::check_internal_sync,
                                            this, tl));
    return utils::strand_run(g->strand(), fn);
  }
  return false;
}

bool details::node_impl::external(symbol const &tl) const {
  SHARED_PTR<graph_impl> g = graph();
  if( g ) {
    boost::function<bool ()> fn(boost::bind(&node_impl::check_external_sync,
                                            this, tl));
    return utils::strand_run(g->strand(), fn);
  }
  return false;
}


// public manipulators

tlog::stream details::node_impl::syslog(symbol const &ctx, symbol const &kind) const {
  symbol source = name();
  if( !ctx.empty() )
    source = source.str()+"."+ctx.str();
  SHARED_PTR<graph_impl> g = graph();
  
  if( g )
    return g->syslog(source, kind);
  else {
    // In case someone produce a message on a disconnected node
    
    source = "<nil>."+source.str();
    return s_log->syslog(source, kind);
  }
}

utils::log_manager &details::node_impl::manager() const {
  SHARED_PTR<graph_impl> g = graph();

  // Following code can appear redundant but it is future proof in the sendse it
  // handle possible case where one graph do not use the global singleton as
  // LogManager.
  if( g )
    return g->manager();
  else
    return *s_log;
}

void details::node_impl::provide(symbol const &tl, bool read_only, bool publish_plan) {
  SHARED_PTR<graph_impl> g = graph();
  
  if( g ) {
    transaction_flags flags;
    flags.set(0, !read_only);
    flags.set(1, publish_plan);
    g->declare(shared_from_this(), tl, flags);
  }
}

void details::node_impl::unprovide(symbol const &tl) {
  SHARED_PTR<graph_impl> g = graph();

  if( g )
    g->strand().dispatch(boost::bind(&node_impl::unprovide_sync,
                                     shared_from_this(), g, tl));
}


void details::node_impl::use(symbol const &tl, bool read_only, bool listen_plan) {
  SHARED_PTR<graph_impl> g = graph();
  
  if( g ) {
    transaction_flags flags;
    flags.set(0, !read_only);
    flags.set(1, listen_plan);
    g->subscribe(shared_from_this(), tl, flags);
  }
}

void details::node_impl::unuse(symbol const &tl) {
  SHARED_PTR<graph_impl> g = graph();
  
  if( g )
    g->strand().dispatch(boost::bind(&node_impl::unuse_sync,
                                     shared_from_this(), tl));
}


void details::node_impl::notify(TICK date, symbol tl,
                                boost::optional<Observation> o) {
  if( o )
    syslog(tlog::null, tlog::info)<<'['<<date<<"] "<<o;
  else
    syslog(tlog::null, tlog::info)<<'['<<date<<"] synchronisation event for timeline "<<tl;
}

// strand protected calls

bool details::node_impl::check_internal_sync(symbol tl) const {
  return graph() && m_internals.find(tl)!=m_internals.end();
}

bool details::node_impl::check_external_sync(symbol tl) const {
  return graph() && m_externals.find(tl)!=m_externals.end();
}

void details::node_impl::assigned_sync(details::tl_ref const &tl) {
  utils::symbol tl_name = tl->name();
  
  syslog(tlog::null, tlog::info)<<"declared \""<<tl_name<<"\" as Internal";
  m_internals.insert(internal_set::value_type(tl_name, tl));
}

void details::node_impl::subscribed_sync(details::ext_ref const &tl) {
  utils::symbol tl_name = tl->name();
  syslog(tlog::null, tlog::info)<<"subscribed to "<<tl_name<<" as External";
  m_externals.insert(external_set::value_type(tl_name, tl));
  tl->connect();
}


void details::node_impl::unprovide_sync(SHARED_PTR<details::graph_impl> g,
                                        symbol tl) {
  internal_set::iterator i = m_internals.find(tl);
  if( m_internals.end()!=i ) {
    tl_ref timeline = i->second;
    m_internals.erase(i);
    g->undecl_sync(shared_from_this(), timeline);
    syslog(tlog::null, tlog::info)<<"Undeclared timeline "<<tl;
  }
}

void details::node_impl::unuse_sync(symbol tl) {
  if( m_externals.erase(tl) )
    syslog(tlog::null, tlog::info)<<"Unsubscribed from timline "<<tl;
}

void details::node_impl::isolate(SHARED_PTR<details::graph_impl> const &g) {
  // Disable all connections
}
