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
#include "graph_impl.hh"
#include "node_impl.hh"
#include "internal_impl.hh"

#include "trex/utils/asio_runner.hh"

using namespace TREX::transaction::details;
namespace asio=boost::asio;
using namespace TREX;

using TREX::utils::symbol;

/*
 * class TREX::transaction::details::graph_impl
 */

graph_impl::graph_impl() {
  m_strand.reset(new asio::strand(m_mgr->service()));
}

graph_impl::~graph_impl() {}

// Observers

symbol const &graph_impl::name() const {
  boost::shared_lock<mutex_type> lock(m_mutex);
  return m_name;
}

boost::optional<date_type> graph_impl::date(bool fast) const {
  if( fast ) {
    boost::shared_lock<mutex_type> lock(m_mutex);
    return m_date;
  } else {
    // Non fast approach ensure sequencing with set_date in case it is posted
    boost::function<boost::optional<date_type> ()>
      fn(boost::bind(&graph_impl::date, this, true));
    return utils::strand_run(strand(), fn);
  }
}

// Modifiers

symbol const &graph_impl::set_name(symbol const &n) {
  boost::upgrade_lock<mutex_type> lock(m_mutex);
  if( m_name.empty() ) {
    boost::upgrade_to_unique_lock<mutex_type> w_lock(lock);
    m_name = n;
  }
  return m_name;
}

void graph_impl::set_date(date_type const &d) {
  strand().dispatch(boost::bind(&graph_impl::set_date_sync,
                                 shared_from_this(), d));
}

WEAK_PTR<node_impl> graph_impl::add_node(symbol const &desired_name) {
  SHARED_PTR<graph_impl> me = shared_from_this();
  SHARED_PTR<node_impl> ret = MAKE_SHARED<node_impl>(me);

  strand().dispatch(boost::bind(&graph_impl::add_node_sync,
                                 me, ret, desired_name));
  return ret;
}

void graph_impl::rm_node(WEAK_PTR<node_impl> n) {
  SHARED_PTR<node_impl> node = n.lock();
  if( node ) {
    node->reset();
    strand().dispatch(boost::bind(&graph_impl::rm_node_sync,
                                   shared_from_this(), node));
  }
}

WEAK_PTR<internal_impl> graph_impl::declare_tl(symbol const &tl,
                                               SHARED_PTR<node_impl> const &n,
                                               transaction_flags const &fl) {
  boost::function<SHARED_PTR<internal_impl> ()>
  fn(boost::bind(&graph_impl::decl_tl_sync, this, tl, n, fl));
  return utils::strand_run(strand(), fn);
}


// Manipulators

utils::log_manager &graph_impl::manager() const {
  return *m_mgr;
}

utils::log::stream graph_impl::syslog(symbol const &ctx,
                                      symbol const &kind) const {
  symbol who = name();
 
  if( !ctx.empty() )
    who = who.str()+"."+ctx.str();

  boost::optional<date_type> when = date(true);
  if( when )
    return m_mgr->syslog(*when, who, kind);
  else
    return m_mgr->syslog(who, kind);
}

void graph_impl::detached(SHARED_PTR<internal_impl> const &tl) {
  tl_map::iterator i = m_timelines.find(tl->name());

  if( m_timelines.end()!=i ) {
    // do something to notify its failure

    if( m_failed.insert(*i).second ) {
      syslog(utils::log::null, utils::log::warn)<<"Failed timeline "<<i->first;
    }
  }
}


// Thread protected operations

void graph_impl::set_date_sync(date_type d) {
  bool updated = false;
  {
    boost::upgrade_lock<mutex_type> lock(m_mutex);
    if( !m_date || d>*m_date ) {
      boost::upgrade_to_unique_lock<mutex_type> w_lock(lock);
      m_date = d;
      updated = true;
    }
  }
  if( updated )
    m_tick(d); // announce the tick update
}

void graph_impl::add_node_sync(SHARED_PTR<node_impl> node,
                               symbol desired_name) {
  // For now I do not worry about mutiple reactors with
  // the same name
  node->set_name(desired_name);
  
  tick_sig::extended_slot_type slot(&node_impl::tick,
                                    node.get(), _1, _2);
  m_tick.connect_extended(slot.track_foreign(node));
  m_nodes.insert(node);
}

void graph_impl::rm_node_sync(SHARED_PTR<node_impl> node) {
  std::set< SHARED_PTR<node_impl> >::iterator pos = m_nodes.find(node);
  
  if( m_nodes.end()!=pos ) {
    syslog(utils::log::null,
           utils::log::info)<<"Removing reactor \""<<node->name()
                            <<"\" from the graph.";
    m_nodes.erase(pos);
    node->reset();
  }
}

SHARED_PTR<internal_impl> graph_impl::decl_tl_sync(symbol tl,
                                                   SHARED_PTR<node_impl> n,
                                                   transaction_flags fl) {
  tl_map::iterator pos = m_timelines.lower_bound(tl);
  
  if( m_timelines.end()==pos || pos->first!=tl ) {
    SHARED_PTR<internal_impl> obj = MAKE_SHARED<internal_impl>(tl, shared_from_this());
    pos = m_timelines.insert(pos, tl_map::value_type(tl, obj));
    // TODO handle the case where this timeline is not assigned yet
  }
  // TODO I also need to check for cycles
  if( n && !pos->second->set_sync(n, fl) ) {
    // throw an exception here but for now just return NULL
    return SHARED_PTR<internal_impl>();
  }
  return pos->second;
}

