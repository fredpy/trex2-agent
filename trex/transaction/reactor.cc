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
#include "reactor.hh"
#include "private/node_impl.hh"
#include "private/graph_impl.hh"

using namespace trex;
using namespace trex::transaction;

namespace bpt=boost::property_tree;
using utils::symbol;

/*
 * class TREX::transaction::reactor
 */

// statics

bpt::ptree::value_type &reactor::xml(reactor::xml_arg_type const &arg) {
  return *(arg.get<0>());
}

// structors

reactor::reactor(reactor::xml_arg_type &cfg) {
  SHARED_PTR<details::graph_impl> tmp = cfg.get<1>();
  m_impl = tmp->new_node(utils::parse_attr<symbol>(xml(cfg), "name"),
                         cfg.get<2>());
  m_impl->set_latency(utils::parse_attr<unsigned>(xml(cfg), "latency"));
  m_impl->set_lookahead(utils::parse_attr<unsigned>(xml(cfg), "lookahead"));

  // Do I want/need to parse potential timelines ?
  // Do I want to alter cfg to embed external config ?
  // anything else ?
}

reactor::~reactor() {
  isolate();
}

// observers

symbol reactor::name() const {
  return m_impl->name();
}

TICK reactor::latency() const {
  return m_impl->latency();
}

TICK reactor::lookahead() const {
  return m_impl->lookahead();
}

bool reactor::internal(symbol const &name) const {
  return m_impl->is_internal(name);
}

bool reactor::external(symbol const &name) const {
  return m_impl->is_external(name);
}



// manipulators

void reactor::provide(utils::symbol const &name, bool g, bool p) {
  m_impl->provide(name, g, p);
}

void reactor::use(symbol const &name, bool g, bool p) {
  m_impl->use(name, g, p);
}


void reactor::attached() {
  m_impl->attach(shared_from_this());
}

void reactor::isolate() {
  m_impl->isolate();
}

utils::log::stream reactor::syslog(utils::symbol const &kind) const {
  return m_impl->syslog(utils::log::null, kind);
}

token_ref reactor::create_obs(utils::symbol const &tl,
                              utils::symbol const &pred) {
  return m_impl->obs(tl, pred);
}

void reactor::post(token_ref obs) {
  m_impl->post(obs);
}

// callbacks

void reactor::disowned(symbol tl) {
  syslog(utils::log::error)<<"Killed due to "<<tl<<" not being Internal";
  isolate();
}

void reactor::unsubscribed(symbol tl) {
  syslog(utils::log::error)<<"Killed due to "<<tl<<" not being External";
  isolate();
}




