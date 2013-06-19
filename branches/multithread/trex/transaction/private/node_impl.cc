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

using namespace TREX::transaction;
namespace utils=TREX::utils;
namespace tlog=utils::log;

using utils::Symbol;

namespace {
  // A reference to the LogManager singleton. So I can silently handle cases where a node
  // is not connected to a graph
  utils::SingletonUse<utils::LogManager> s_log;
}

/*
 * class TREX::transaction::details::node_impl
 */

// structors

details::node_impl::node_impl(boost::weak_ptr<details::graph_impl> const &g):m_graph(g) {
}

details::node_impl::~node_impl() {
}

// public observers

bool details::node_impl::internal(Symbol const &tl) const {
  boost::shared_ptr<graph_impl> g = graph();
  
  if( g ) {
    boost::function<bool ()> fn(boost::bind(&node_impl::check_internal_sync, this, tl));
    return utils::strand_run(g->strand(), fn);
  }
  return false;
}

bool details::node_impl::external(Symbol const &tl) const {
  boost::shared_ptr<graph_impl> g = graph();
  
  if( g ) {
    boost::function<bool ()> fn(boost::bind(&node_impl::check_external_sync, this, tl));
    return utils::strand_run(g->strand(), fn);
  }
  return false;
}


// public manipulators

tlog::stream details::node_impl::syslog(Symbol const &ctx, Symbol const &kind) const {
  Symbol source = name();
  if( !ctx.empty() )
    source = source.str()+"."+ctx.str();
  boost::shared_ptr<graph_impl> g = graph();
  
  if( g )
    return g->syslog(source, kind);
  else {
    // In case someone produce a message on a disconnected node
    
    source = "<nil>."+source.str();
    return s_log->syslog(source, kind);
  }
}

utils::LogManager &details::node_impl::manager() const {
  boost::shared_ptr<graph_impl> g = graph();

  // Following code can appear redundant but it is future proof in the sense it
  // handle possible case where one graph do not use the global singleton as
  // LogManager.
  if( g )
    return g->manager();
  else
    return *s_log;
}

void details::node_impl::provide(Symbol const &tl, bool read_only, bool publish_plan) {
  boost::shared_ptr<graph_impl> g = graph();
  
  if( g ) {
    transaction_flags flags;
    flags.set(0, !read_only);
    flags.set(1, publish_plan);
    g->declare(shared_from_this(), tl, flags);
  }
}

void details::node_impl::unprovide(Symbol const &tl) {
  boost::shared_ptr<graph_impl> g = graph();
  if( g )
    g->strand().dispatch(boost::bind(&node_impl::unprovide_sync, shared_from_this(), g, tl));
}


void details::node_impl::use(Symbol const &tl, bool read_only, bool listen_plan) {
  boost::shared_ptr<graph_impl> g = graph();
  
  if( g ) {
    transaction_flags flags;
    flags.set(0, !read_only);
    flags.set(1, listen_plan);
    g->subscribe(shared_from_this(), tl, flags);
  }
}

void details::node_impl::unuse(Symbol const &tl) {
  boost::shared_ptr<graph_impl> g = graph();
  if( g )
    g->strand().dispatch(boost::bind(&node_impl::unuse_sync, shared_from_this(), tl));
}

// strand protected calls

void details::node_impl::unprovide_sync(boost::shared_ptr<details::graph_impl> g, Symbol tl) {
  internal_map::iterator i = m_internals.find(tl);
  
  if( m_internals.end()!=i ) {
    m_internals.erase(i);
    g->undeclare(shared_from_this(), i->second);
  }
}

void details::node_impl::unuse_sync(Symbol tl) {
  m_externals.erase(tl);
}

void details::node_impl::assigned(details::tl_ref tl) {
  m_internals.insert(std::make_pair(tl->name(), tl));
}

void details::node_impl::subscribed(details::ext_ref tl) {
  m_externals.insert(std::make_pair(tl->name(), tl));
}

bool details::node_impl::check_internal_sync(Symbol name) const {
  return graph() && m_internals.end()!=m_internals.find(name);
}

bool details::node_impl::check_external_sync(Symbol name) const {
  return graph() && m_externals.end()!=m_externals.find(name);
}


void details::node_impl::isolate(boost::shared_ptr<details::graph_impl> const &g) {
  boost::shared_ptr<node_impl> me = shared_from_this();
  
  for(internal_map::iterator i=m_internals.begin(); m_internals.end()!=i; ++i)
    g->undeclare(me, i->second);
  m_internals.clear();
  m_externals.clear();
}
