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
#include "graph.hh"
#include "reactor.hh"
#include "private/graph_impl.hh"


using namespace TREX::transaction;

namespace utils=TREX::utils;
namespace tlog=utils::log;
namespace asio=boost::asio;
namespace bpt=boost::property_tree;

using utils::symbol;

/*
 * class TREX::transaction::graph
 */

// structors

graph::graph(utils::symbol const &n) {
  m_impl = MAKE_SHARED<details::graph_impl>(n);
}

graph::graph(bpt::ptree &xml) {
  symbol n = utils::parse_attr<symbol>(xml, "name");
  m_impl = MAKE_SHARED<details::graph_impl>(n);
  
  size_t count = add_reactors(xml);
  syslog(tlog::info)<<"Created "<<count<<" reactors.";
}

graph::~graph() {}

// observers

symbol const &graph::name() const {
  return m_impl->name();
}

utils::log_manager &graph::manager() const {
  return m_impl->manager();
}

asio::io_service &graph::service() const {
  return manager().service();
}

size_t graph::count_reactors() const {
  return m_impl->reactors_size();
}

// modifiers

bool graph::add_reactor(bpt::ptree::value_type &xml) {
  details::reactor_factory::argument_type arg = details::reactor_factory::arg_traits::build(xml, m_impl);
  
  SHARED_PTR<reactor> tmp(m_factory->produce(arg));
  return add_reactor(tmp);
}

bool graph::add_reactor(SHARED_PTR<reactor> r) {
  return m_impl->add_reactor(r);
}

// manipulators

tlog::stream graph::syslog(symbol const &kind) const {
  return m_impl->syslog(tlog::null, kind);
}

void graph::tick(TICK date) {
  m_impl->tick(date);
}

