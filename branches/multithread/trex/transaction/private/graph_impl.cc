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

using namespace TREX::transaction;
namespace utils=TREX::utils;
namespace tlog=utils::log;
namespace asio=boost::asio;

using utils::Symbol;

/*
 * class TREX::transaction::details::graph_impl
 */

// structors

details::graph_impl::graph_impl()
:m_strand(new asio::strand(m_log->service())) {}

details::graph_impl::graph_impl(Symbol const &name)
:m_name(name), m_strand(new asio::strand(m_log->service())) {}

details::graph_impl::~graph_impl() {
}

// public  manipulators

void details::graph_impl::set_date(details::graph_impl::date_type const &d) {
  m_strand->dispatch(boost::bind(boost::mem_fn(&graph_impl::set_date_sync), 
				 shared_from_this(), 
				 d));
}

boost::optional<details::graph_impl::date_type> details::graph_impl::get_date(bool fast) const {
  if( fast ) {
    // if fast just directly access the mutex protected value
    utils::SharedVar< boost::optional<date_type> >::scoped_lock lock(m_date);
    return *m_date;
  } else {
    // If not fast post the call through the starnd to ensure that any queued set_date have been processed
    boost::function<boost::optional<date_type> ()> fn(boost::bind(&graph_impl::get_date, this, true));
    return utils::strand_run(strand(), fn);
  }
}

tlog::stream details::graph_impl::syslog(Symbol const &ctx,
                                         Symbol const &kind) const {
  // Access quickly to the date : I'd rather have an innacurate date in the logs
  // than being blocked by pending events in the graph strand 
  boost::optional<date_type> cur = get_date(true);
  Symbol who = m_name;
  if( !ctx.empty() )
    who = who.str()+"."+ctx.str();
  if( cur )
    return m_log->syslog(*cur, who, kind);
  else
    return m_log->syslog(who, kind);
}


details::node_id details::graph_impl::create_node() {
  SHARED_PTR<graph_impl> me = shared_from_this();
  SHARED_PTR<node_impl> ret(new node_impl(me));

  strand().dispatch(boost::bind(&graph_impl::add_node_sync, 
				me, ret));
  return ret;
}

bool details::graph_impl::remove_node(details::node_id const &n) {
  SHARED_PTR<graph_impl> me = shared_from_this();
  SHARED_PTR<node_impl> node = n.lock();

  if( node && me==node->graph() ) {
    node->m_graph.reset();
    strand().dispatch(boost::bind(&graph_impl::rm_node_sync, 
				  me, node));
    return true;
  }
  return false;
}

// private calls

void details::graph_impl::declare(SHARED_PTR<details::node_impl> n,
                                  Symbol const &name,
                                  details::transaction_flags flag) {
  SHARED_PTR<graph_impl> me = shared_from_this();
  strand().dispatch(boost::bind(&graph_impl::decl_sync, 
				me, 
				n, name, flag));
}

void details::graph_impl::subscribe(SHARED_PTR<details::node_impl> n,
                                    Symbol const &name,
                                    details::transaction_flags flag) {
  SHARED_PTR<graph_impl> me = shared_from_this();
  strand().dispatch(boost::bind(&graph_impl::use_sync, 
				me, 
				n, name, flag));
}

void details::graph_impl::synchronize(TICK date) {
  SHARED_PTR<graph_impl> me = shared_from_this();
  strand().dispatch(boost::bind(&graph_impl::synch_sync, me, date));
}


// strand protected calls

void details::graph_impl::set_date_sync(details::graph_impl::date_type date) {
  // This may not look thread safe but it is : there's a mutex lock happening
  // behind the curtain thanks to SharedVar
  m_date = date;
}


void details::graph_impl::add_node_sync(SHARED_PTR<details::node_impl> n) {
  m_nodes.insert(n);
}

void details::graph_impl::rm_node_sync(SHARED_PTR<details::node_impl> n) {
  n->isolate(shared_from_this());
  m_nodes.erase(n);
}


details::tl_ref details::graph_impl::get_timeline_sync(Symbol const &name) {
  tl_map::iterator i = m_timelines.lower_bound(name);

  if( m_timelines.end()==i || name!=i->first ) {
    tl_ref tl = MAKE_SHARED<internal_impl>(name, shared_from_this());
    
    i = m_timelines.insert(i, tl_map::value_type(name, tl));
    m_failed.insert(*i); // Add this timeline to the failed set
    // schedule event for later
    strand().post(boost::bind(boost::mem_fn(&graph_impl::notify_new), 
			      shared_from_this(), 
			      tl));
  }
  return i->second;
}

void details::graph_impl::synch_sync(TICK date) {
  for(tl_map::iterator i=m_failed.begin(); m_failed.end()!=i; ++i) {
    tl_ref tl = i->second;
    if( !tl->m_last_obs ) {
      tl->m_next_obs = tl->obs("Failed");
    } else if( tl->m_last_obs->predicate()!="Failed" ) {
      TICK fail_date = tl->m_last_synch + 1;
      tl->m_next_obs = tl->obs("Failed");
      if( fail_date<date )
        tl->synchronize(fail_date); // post the Failed observation
    }
    tl->synchronize(date); // Update timeline to current tick
  }
}


void details::graph_impl::decl_sync(SHARED_PTR<details::node_impl> n,
                                    Symbol name, details::transaction_flags flag) {
  SHARED_PTR<graph_impl> owned = n->graph();
  if( shared_from_this()==owned ) {
    tl_ref tl = get_timeline_sync(name);

    if( tl->set_sync(n, flag) ) {
      m_failed.erase(name);
      n->assigned(tl);
    }
  } else
    syslog(tlog::null, tlog::warn)<<"Ignoring creation request of timeline \""<<name
    <<"\" as it was requested by a reactor that is no longer part of this graph.";
}

void details::graph_impl::use_sync(SHARED_PTR<details::node_impl> n,
                                   Symbol name, details::transaction_flags flag) {
  
  SHARED_PTR<graph_impl> owned = n->graph();
  if( shared_from_this()==owned ) {
    SHARED_PTR<external_impl>
    ext = MAKE_SHARED<external_impl>(n, get_timeline_sync(name), flag);
    
    n->subscribed(ext);
  } else
    syslog(tlog::null, tlog::warn)<<"Ignoring subscription request to timeline \""<<name
    <<"\" as it was requested by a reactor that is no longer part of this graph.";
}

void details::graph_impl::undeclare(SHARED_PTR<details::node_impl> n, details::tl_ref tl) {
  if( tl->owner().lock()==n ) {
    m_failed.insert(tl_map::value_type(tl->name(), tl));
    tl->reset_sync();
  }
}

void details::graph_impl::notify_new(details::tl_ref tl) {
  syslog(tlog::null, tlog::info)<<"New timeline \""<<tl->name()<<"\" created.";
  m_new_tl(tl);
}
