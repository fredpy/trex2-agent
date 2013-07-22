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
 * class TREX::transaction::details::internal_impl
 */

// structors

details::internal_impl::internal_impl(Symbol const &name, WEAK_PTR<details::graph_impl> const &g)
:m_name(name), m_graph(g), m_flags(0) {}


details::internal_impl::~internal_impl() {}

// public observers

bool details::internal_impl::accept_goals() const {
  SHARED_PTR<graph_impl> g = m_graph.lock();
  
  if( g ) {
    boost::function<bool ()> fn(boost::bind(&details::transaction_flags::test, &m_flags, 0));
    return utils::strand_run(g->strand(), fn);
  }
  return false;
}

bool details::internal_impl::publish_plan() const {
  SHARED_PTR<graph_impl> g = m_graph.lock();
  
  if( g ) {
    boost::function<bool ()> fn(boost::bind(&details::transaction_flags::test, &m_flags, 1));
    return utils::strand_run(g->strand(), fn);
  }
  return false;  
}

details::node_id details::internal_impl::owner() const {
  SHARED_PTR<graph_impl> g = m_graph.lock();

  if( g ) {
    boost::function<node_id ()> fn(boost::bind(&internal_impl::owner_sync, this));
    return utils::strand_run(g->strand(), fn);
  } else {
    node_id empty;
    return empty;
  }
}

Observation details::internal_impl::obs(utils::Symbol const &pred) {
  return Observation(name(), pred);
}


// manipulators

void details::internal_impl::post_observation(Observation const &obs,
                                              bool echo) {
  SHARED_PTR<graph_impl> g = m_graph.lock();
  
  if( g ) {
    SHARED_PTR<node_impl> node = m_owner.lock();
    if( node ) {
      g->strand().post(boost::bind(&internal_impl::post_obs_sync,
                                   shared_from_this(), node, obs, echo));
    }
  }
}

void details::internal_impl::synchronize(TICK date) {
  SHARED_PTR<graph_impl> g = m_graph.lock();
  if( g ) {
    g->strand().post(boost::bind(&internal_impl::notify_sync,
                                 shared_from_this(), date));
  }
}

void details::internal_impl::connect(details::ext_ref tl) {
  SHARED_PTR<graph_impl> g = graph();
  
  if( g ) {
    synch_event::slot_type s(&details::external_impl::on_synch, tl.get(),
                             _1, _2);
    m_synch.connect(s.track(tl));
  
    if( m_last_obs ) {
      g->strand().post(boost::bind(&details::external_impl::on_synch,
                                   tl, m_obs_date, m_last_obs));
      if( m_obs_date<m_last_synch )
        g->strand().post(boost::bind(&details::external_impl::on_synch,
                                     tl, m_last_synch,
                                     boost::optional<Observation>()));
    }
  }
}


// strand protected calls

details::node_id details::internal_impl::owner_sync() const {
  return m_owner;
}

bool details::internal_impl::reset_sync() {
  if( m_owner.lock() ) {
    m_owner.reset();
    m_flags.reset();
    m_next_obs = obs("Failed"); // need to schedule a synchronize ...
    return true;
  }
  return false;
}

bool details::internal_impl::set_sync(SHARED_PTR<details::node_impl> const &n,
                                      details::transaction_flags const &fl) {
  SHARED_PTR<node_impl> cur = m_owner.lock();
  
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

void details::internal_impl::post_obs_sync(SHARED_PTR<node_impl> n,
                                           Observation o, bool echo) {
  SHARED_PTR<node_impl> owner = m_owner.lock();
  
  if( n==owner ) {
    if( m_next_obs ) {
      owner->syslog(name(), tlog::warn)<<"post observation overrule the previous pending.\n\t - lost observation is "<<o;
    }
    m_next_obs = o;
    m_echo = echo;
  } else if( n ) {
    n->syslog(tlog::null, tlog::error)<<"Cannot post observation on non owned timeline "<<name()<<".\n\t- observation was :"<<o;
  } else {
    owner->syslog(name(), tlog::warn)<<"Ignored post observation from a null reactor.\n\t- observation was: "<<o;
  }
}

void details::internal_impl::notify_sync(TICK date) {
  SHARED_PTR<graph_impl> g = m_graph.lock();
  
  if( g ) {
    boost::optional<Observation> o(m_next_obs);
    if( m_next_obs ) {
      m_last_obs = m_next_obs;
      m_next_obs.reset(); // remove cached observation
      m_obs_date = date;
      if( m_echo ) {
        g->syslog(name(), "ASSERT")<<"["<<date<<"] "<<(*m_last_obs);
        m_echo = false;
      }
    }
    m_last_synch = date;
    m_synch(date, o);
  }
}


/*
 * class TREX::transaction::details::external_impl
 */

// structors

details::external_impl::external_impl(SHARED_PTR<details::node_impl> cli,
                                      details::tl_ref tl,
                                      details::transaction_flags const &fl)
:m_timeline(tl), m_client(cli), m_flags(fl) {}

details::external_impl::~external_impl() {}

// public observers

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
    boost::function<bool ()> fn(boost::bind(&details::transaction_flags::test, &m_flags, 0));
    if( utils::strand_run(g->strand(), fn) )
      return m_timeline->accept_goals();
  }
  return false;
}

bool  details::external_impl::publish_plan() const {
  SHARED_PTR<graph_impl> g = graph();
  
  if( g ) {
    boost::function<bool ()> fn(boost::bind(&details::transaction_flags::test, &m_flags, 1));
    if( utils::strand_run(g->strand(), fn) )
      return m_timeline->publish_plan();
  }
  return false; 
}

// Manipulators

void details::external_impl::on_synch(TICK date, boost::optional<Observation> o) {
  SHARED_PTR<node_impl> n = m_client.lock();
  if( n ) {
    if( o )
      n->syslog(name(), tlog::info)<<'['<<date<<"]obs: "<<o;
    else  
      n->syslog(name(), tlog::info)<<"Synch event ["<<date<<"]";
  }
}








