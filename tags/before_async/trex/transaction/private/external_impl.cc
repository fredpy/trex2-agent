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

using namespace TREX::transaction;
namespace utils=TREX::utils;
namespace tlog=utils::log;

/*
 * class TREX::transaction::details::external_impl
 */

// structors

details::external_impl::external_impl(SHARED_PTR<details::node_impl> cli,
                                      SHARED_PTR<details::internal_impl> tl,
                                      details::transaction_flags const  &fl)
:m_timeline(tl), m_client(cli), m_flags(fl) {}

// observers

SHARED_PTR<details::graph_impl> details::external_impl::graph() const {
  SHARED_PTR<graph_impl> ret;
  SHARED_PTR<node_impl> node = m_client.lock();
  
  if( node )
    ret = node->graph();
  return ret;
}

bool details::external_impl::accept_goals() const {
  SHARED_PTR<graph_impl> g = graph();
  
  if( g ) {
    boost::function<bool ()> fn(boost::bind(&details::transaction_flags::test,
                                            &m_flags, 0));
    if( utils::strand_run(g->strand(), fn) )
      return m_timeline->accept_goals();
  }
  return false;
}

bool details::external_impl::publish_plan() const {
  SHARED_PTR<graph_impl> g = graph();
  
  if( g ) {
    boost::function<bool ()> fn(boost::bind(&details::transaction_flags::test,
                                            &m_flags, 1));
    if( utils::strand_run(g->strand(), fn) )
      return m_timeline->publish_plan();
  }
  return false;
}

void details::external_impl::on_synch(TICK date, token_id o) {
  SHARED_PTR<node_impl> node = m_client.lock();
  if( node )
    node->notify(date, name(), o);
}

// modifiers

void details::external_impl::reset() {
  m_client.reset();
}


