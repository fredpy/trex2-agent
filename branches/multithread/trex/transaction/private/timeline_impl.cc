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
#include "timeline_impl.hh"
#include "graph_impl.hh"

using namespace TREX::transaction;
namespace utils=TREX::utils;
namespace tlog=utils::log;
namespace asio=boost::asio;

using utils::Symbol;

/*
 * class TREX::transaction::details::timeline_impl
 */

// structors

details::timeline_impl::timeline_impl(Symbol const &name, boost::weak_ptr<details::graph_impl> const &g)
:m_name(name), m_graph(g), m_flags(0) {}


details::timeline_impl::~timeline_impl() {}

// public observers

bool details::timeline_impl::accept_goals() const {
  boost::shared_ptr<graph_impl> g = m_graph.lock();
  
  if( g ) {
    boost::function<bool ()> fn(boost::bind(&details::transaction_flags::test, &m_flags, 0));
    return utils::strand_run(g->strand(), fn);
  }
  return false;
}

bool details::timeline_impl::publish_plan() const {
  boost::shared_ptr<graph_impl> g = m_graph.lock();
  
  if( g ) {
    boost::function<bool ()> fn(boost::bind(&details::transaction_flags::test, &m_flags, 1));
    return utils::strand_run(g->strand(), fn);
  }
  return false;  
}

details::node_id details::timeline_impl::owner() const {
  boost::shared_ptr<graph_impl> g = m_graph.lock();

  if( g ) {
    boost::function<node_id ()> fn(boost::bind(&timeline_impl::owner_sync, this));
    return utils::strand_run(g->strand(), fn);
  } else {
    node_id empty;
    return empty;
  }
}

// strand protected calls

details::node_id details::timeline_impl::owner_sync() const {
  return m_owner;
}

bool details::timeline_impl::reset_sync() {
  if( m_owner.lock() ) {
    m_owner.reset();
    m_flags.reset();
    // TODO set myself as faile
    return true;
  }
  return false;
}

bool details::timeline_impl::set_sync(boost::shared_ptr<details::node_impl> const &n,
                                      details::transaction_flags const &fl) {
  boost::shared_ptr<node_impl> cur = m_owner.lock();
  
  if( cur ) {
    if( n==cur && m_flags!=fl ) {
      m_flags = fl;
      return true;
    }
    return false;
  } else {
    m_owner = n;
    m_flags = fl;
    return true;
  }
}




