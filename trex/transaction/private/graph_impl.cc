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
#include "../reactor.hh"
#include "../graph_error.hh"

#include "trex/utils/asio_runner.hh"

using namespace trex::transaction;
namespace utils = trex::utils;
namespace asio = boost::asio;
namespace tlog = utils::log;

using details::graph_impl;
using utils::symbol;

asio::io_service &trex::transaction::details::service_of(SHARED_PTR<graph_impl> const &g) {
  return g->manager().service();
}

/*
 * class TREX::transaction::details::graph_impl
 */

// structors

graph_impl::graph_impl(symbol const &n):m_name(n) {
  m_strand.reset(new asio::strand(manager().service()));
}

graph_impl::~graph_impl() {
  // TODO cleanup the graph
  reactor_set tmp;
  syslog(tlog::null, tlog::info)<<"Shutting down the graph";
  {
    boost::unique_lock<boost::shared_mutex> lock;
    std::swap(tmp, m_reactors);
  }
  while( !tmp.empty() ) {
    reactor_set::iterator i = tmp.begin();
    SHARED_PTR<reactor> r = *i;
    tmp.erase(i);
    if( r )
      r->isolate();
  }
  
  
  for(reactor_set::iterator i=tmp.begin(); tmp.end()!=i; ++i) {
    
  }
}

// observers

boost::optional<details::date_type> graph_impl::now() const {
  boost::shared_lock<boost::shared_mutex> lock(m_date_mtx);
  return m_date;
}

size_t graph_impl::reactors_size() const {
  boost::shared_lock<boost::shared_mutex> lock(m_mtx);
  return m_reactors.size();
}

// manipulators

tlog::stream graph_impl::syslog(symbol const &who,
                                symbol const &kind) const {
  symbol source = name();
  boost::optional<date_type> date = now();
  
  if( !who.empty() )
    source = source.str()+"."+who.str();

  if( date )
    return m_log->syslog(*date, source, kind);
  else
    return m_log->syslog(source, kind);
}

SHARED_PTR<details::node_impl> graph_impl::new_node
(symbol const &n, details::exec_ref const &queue) {
  WEAK_PTR<graph_impl> me = shared_from_this();
  // Create the new node
  SHARED_PTR<node_impl> ret = MAKE_SHARED<node_impl>(me, n, queue);  
  return ret;
}

bool graph_impl::add_reactor(SHARED_PTR<reactor> r) {
  if( r ) {
    r->attached(); // Notify the reactor that it is attached
    return true;
  }
  return false;
}


// graph strand calls

void graph_impl::g_strand_tick(details::date_type date) {
  bool updated = false;
  boost::optional<date_type> prev;
  
  {
    boost::upgrade_lock<boost::shared_mutex> test(m_date_mtx);
    
    prev = m_date;
    if( !m_date || (*m_date)<date ) {
      boost::upgrade_to_unique_lock<boost::shared_mutex> lock(test);
      
      m_date =  date;
      updated = true;
    }
  }
  
  if( updated ) {
    if( prev ) {
      date_type delta = date - *prev;
      if( delta>1 )
        m_log->syslog(date, name(),
                      tlog::warn)<<"Clock skipped "<<(delta-1)<<" tick"
                                 <<(delta==2?"":"s");
    }
    m_tick(date);
  } else if( date==*prev )
    m_log->syslog(date, name(),
                  tlog::warn)<<"Ignoring redundant tick event"
                             <<" for current date";
  else
    m_log->syslog(date, name(),
                  tlog::error)<<"Ignoring erroneous past tick event ("
                              <<date<<")";
}

ERROR_CODE graph_impl::g_strand_add(SHARED_PTR<reactor> const &r) {
  bool inserted = false;
  ERROR_CODE ec = graph_error_code(graph_error::ok);
  {
    boost::upgrade_lock<boost::shared_mutex> test(m_mtx);
    reactor_set::iterator pos = m_reactors.lower_bound(r);
    
    if( m_reactors.end()==pos || r!=*pos ) {
      boost::upgrade_to_unique_lock<boost::shared_mutex> lock(test);
      
      pos = m_reactors.insert(pos, r);
      inserted = true;
    }
  }
  if( inserted ) {
    // Insertion succeeded => connect this reactor to the clock
    SHARED_PTR<details::node_impl> impl = r->impl();
    tick_sig::extended_slot_type slot(&details::node_impl::g_strand_tick,
                                      impl.get(), _1, _2);
    m_tick.connect_extended(slot.track_foreign(impl));
  } else {
    syslog(tlog::null, tlog::warn)<<"Attempted to insert reactor \""
                                  <<r->name()<<"\" multiple times.";
    ec = graph_error_code(graph_error::reactor_already_exist);
  }
  return ec;
}

void graph_impl::g_strand_rm(SHARED_PTR<reactor> r) {
  bool removed = false;
  {
    boost::unique_lock<boost::shared_mutex> lock(m_mtx);
    removed = (0!=m_reactors.erase(r));
  }
  if( !removed )
    syslog(tlog::null, tlog::error)<<"Attempted to remove reactor \""
    <<r->name()<<"\" which was ot listed for this graph.";
}


SHARED_PTR<details::internal_impl>
graph_impl::g_strand_decl(SHARED_PTR<details::node_impl> const &r,
                          symbol const &tl, details::transaction_flags gp,
                          ERROR_CODE &ec) {
  bool fresh = false;
  SHARED_PTR<internal_impl> ret;
  SHARED_PTR<graph_impl> me = shared_from_this();
  ec = graph_error_code(graph_error::ok);
  {
    boost::upgrade_lock<boost::shared_mutex> test(m_mtx);
    internal_set::iterator const endi = m_internals.end();
    internal_set::iterator i = name_lower_bound(m_internals.begin(), endi, tl);
  
    if( endi!=i && (*i)->name()==tl ) {
      ret = *i;
      if( r ) {
        boost::upgrade_to_unique_lock<boost::shared_mutex> lock(test);
        ec = ret->g_strand_set_owner(r, gp);
        if( ec ) {
          ret.reset();
          return ret;
        }
      }
    } else {
      fresh = true;
      ret = MAKE_SHARED<internal_impl>(me, tl);
      boost::upgrade_to_unique_lock<boost::shared_mutex> lock(test);
      if( r ) {
        ec = ret->g_strand_set_owner(r, gp);
        if( ec ) {
          ret.reset();
          return ret;
        }
      }
      m_internals.insert(i, ret);
    }
  }
  if( fresh ) {
    tick_sig::extended_slot_type slot(&internal_impl::g_strand_tick,
                                      ret.get(), _1, _2);
    m_tick.connect_extended(slot.track_foreign(ret).track_foreign(me));
  }
  return ret;
}
