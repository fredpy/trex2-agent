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
#include "internal_impl.hh"
#include "external_impl.hh"

#include <trex/utils/asio_runner.hh>

using namespace TREX::transaction;
namespace utils=TREX::utils;
namespace tlog=utils::log;


/*
 * class TREX::transaction::details::internal_impl
 */

// statics

utils::symbol const details::internal_impl::s_assert("ASSERT");
utils::symbol const details::internal_impl::s_failed("Failed");

// structors

details::internal_impl::internal_impl(utils::symbol const &tl_name,
                                      WEAK_PTR<details::graph_impl> const &g)
:m_name(tl_name), m_graph(g), m_access(0) {}

details::internal_impl::~internal_impl() {}

// observers

WEAK_PTR<details::node_impl> details::internal_impl::owner() const {
  SHARED_PTR<graph_impl> g = graph();
  
  if( g ) {
    boost::function<WEAK_PTR<node_impl> ()> fn(boost::bind(&internal_impl::owner_sync,
                                               this));
    return utils::strand_run(g->strand(), fn);
  } else {
    WEAK_PTR<node_impl> empty;
    return empty;
  }
}

bool details::internal_impl::accept_goals() const {
  SHARED_PTR<graph_impl> g = graph();

  if( g ) {
    boost::function<bool ()> fn(boost::bind(&transaction_flags::test,
                                            &m_access, 0));
    return utils::strand_run(g->strand(), fn);
  }
  return false;
}

bool details::internal_impl::publish_plan() const {
  SHARED_PTR<graph_impl> g = graph();
  
  if( g ) {
    boost::function<bool ()> fn(boost::bind(&transaction_flags::test,
                                            &m_access, 1));
    return utils::strand_run(g->strand(), fn);
  }
  return false;
}

boost::optional<TICK> details::internal_impl::synch_date() const {
  SHARED_PTR<graph_impl> g = graph();
  boost::optional<TICK> ret;
  
  if( g ) {
    boost::function<boost::optional<TICK> ()>
    fn(boost::bind(&internal_impl::last_update_sync, shared_from_this()));
    
    ret = utils::strand_run(g->strand(), fn);
  }
  return ret;
}

// manipulators

Observation details::internal_impl::create_obs(utils::symbol const &pred) {
  return Observation(name(), pred);
}

void details::internal_impl::connect(SHARED_PTR<details::external_impl> client) {
  SHARED_PTR<graph_impl> g = graph();

  if( g )
    g->strand().post(boost::bind(&internal_impl::connect_sync,
                                 shared_from_this(), client));
}

// modifiers

void details::internal_impl::post_observation(Observation const &obs,
                                              bool echo) {
  SHARED_PTR<graph_impl> g = graph();
  
  if( g ) {
    SHARED_PTR<node_impl> node = m_owner.lock();
    if( node ) {
      g->strand().post(boost::bind(&internal_impl::post_obs_sync,
                                   shared_from_this(),
                                   node, obs, echo));
    }
  }
}

void details::internal_impl::synchronize(TICK date) {
  SHARED_PTR<graph_impl> g = graph();
  
  if( g ) {
    g->strand().post(boost::bind(&internal_impl::notify_sync,
                                 shared_from_this(), date));
  }
}

// Strand protected manipulators

WEAK_PTR<details::node_impl> details::internal_impl::owner_sync() const {
  return m_owner;
}

bool details::internal_impl::reset_sync() {
  if( m_owner.lock() ) {
    m_owner.reset();
    m_access.reset();
    m_next_obs = create_obs(s_failed);
    return true;
  }
  return false;
}

bool details::internal_impl::set_sync(SHARED_PTR<details::node_impl> node,
                                      details::transaction_flags const &fl) {
  SHARED_PTR<node_impl> cur = m_owner.lock();
  
  if( cur ) {
    if( node==cur && m_access!=fl ){
      m_access = fl;
      return true;
    }
    return false;
  } else {
    m_owner = node;
    m_access = fl;
    return true;
  }
}


boost::optional<TICK> details::internal_impl::last_update_sync() const {
  boost::optional<TICK> ret;
  
  if( m_last_obs )
    ret = m_last_synch;
  return ret;
}

void details::internal_impl::connect_sync(SHARED_PTR<details::external_impl> client) {
  SHARED_PTR<graph_impl> g = graph();
  
  if( g ) {
    synch_event::slot_type s(&external_impl::on_synch,
                           client.get(), _1, _2);
    if( m_last_obs ) {
      // post the last observation
      g->strand().post(boost::bind(&external_impl::on_synch,
                                   client, m_obs_date, m_last_obs));
      if( m_obs_date<m_last_synch ) {
        boost::optional<Observation> empty;
        // update to last tick date
        g->strand().post(boost::bind(&external_impl::on_synch,
                                     client, m_last_synch, empty));
      }
    }
  }
}


void details::internal_impl::post_obs_sync(SHARED_PTR<details::node_impl> node,
                                           Observation obs, bool echo) {
  SHARED_PTR<node_impl> owner = m_owner.lock();
  
  if( node==owner ) {
    // the owner of this timeline did not chage since we called
    if( m_next_obs ) {
      // An observation was pending and will be lost: log this
      owner->syslog(name(), tlog::warn)<<"New posted observation overrule"
        " previously cached observation.\n\t- Lost observation: "
        <<(*m_next_obs);
    }
    m_next_obs = obs;
    m_echo = echo;
  } else {
    // owner is not the same anymore
    if( node ) {
      // notify that node no longer owns this timeline
      node->syslog(tlog::null, tlog::error)<<"This reactor no longer own"
        " timeline \""<<name()<<"\" and the observation requested"
        " cannot be posted.\n\t- observation was: "<<obs;
    } else {
      // this should never happen but you never know
      owner->syslog(name(), tlog::error)<<"Ignore observation from an invalid"
      " (null) reactor.\n\t- observation was: "<<obs;
    }
  }
}

void details::internal_impl::notify_sync(TICK date) {
  SHARED_PTR<graph_impl> g = graph();
  
  // NOTE: ideally I should double check that date is grater
  //       than last update but we will assume that the caller
  //       is not stupid
  
  if( g ) {
    boost::optional<Observation> obs(m_next_obs);
    
    if( m_next_obs ) {
      // if m_next_obs is not empty then it is a nerw observation
      // for this tick
      m_last_obs = m_next_obs;
      m_next_obs.reset();
      m_obs_date = date;
      if( m_echo ) {
        g->syslog(name(), s_assert)<<'['<<date<<"] "<<(*m_last_obs);
        m_echo = false;
      }
    }
    m_last_synch = date;
    // publish event that this timeline is synchronized for tick date
    m_synch(date, obs);
  }
}





