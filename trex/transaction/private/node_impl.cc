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
#include "external_impl.hh"

#include "../reactor.hh"
#include "../reactor_error.hh"

#include <trex/utils/asio_runner.hh>

using namespace trex::transaction::details;
using namespace trex::transaction;
using namespace trex;

namespace tlog=trex::utils::log;
namespace bs2=boost::signals2;

using trex::utils::symbol;

/*
 * class TREX::transaction::details::node_impl
 */

// structors

node_impl::node_impl(WEAK_PTR<graph_impl> const &g,
                     symbol const &n,
                     exec_ref const &e)
:m_graph(g), m_name(n),
 m_exec(e), m_dead(false), m_latency(0), m_lookahead(0) {
  if( m_exec->is_active() ) {
    syslog(tlog::null, tlog::warn)<<"Suspending exec queue given for "
      <<"new reactor \""<<n<<"\"";
    m_exec->stop();
  }
}

node_impl::~node_impl() {
  m_exec->clear();
}

// observers

bool node_impl::root() const {
  boost::shared_lock<boost::shared_mutex> read(m_mtx);
  return m_externals.empty();
}

bool node_impl::dead() const {
  boost::shared_lock<boost::shared_mutex> read(m_mtx);
  return m_dead;
}

date_type node_impl::latency() const {
  boost::shared_lock<boost::shared_mutex> read(m_mtx);
  return m_latency;
}

date_type node_impl::lookahead() const {
  boost::shared_lock<boost::shared_mutex> read(m_mtx);
  return m_lookahead;
}

boost::optional<date_type> node_impl::now() const {
  return m_externals.graph_date();
}

boost::optional<date_type> node_impl::last_synch() const {
  return m_externals.synch_date();
}

bool node_impl::is_internal(symbol const &tl) const {
  boost::shared_lock<boost::shared_mutex> read(m_mtx);
  internal_set::const_iterator const endi = m_internals.end();
  internal_set::const_iterator const i = name_lower_bound(m_internals.begin(),
                                                          endi, tl);
  return endi!=i && (*i)->name()==tl;
}

bool node_impl::is_external(symbol const &tl) const {
  return m_externals.external(tl);
}


token_ref node_impl::obs(symbol const &tl, symbol const &pred) const {
  token_ref nil;
  
  {
    internal_set::const_iterator const endi = m_internals.end();
    internal_set::const_iterator const i = name_lower_bound(m_internals.begin(),
                                                            endi, tl);
    if( endi!=i && (*i)->name()==tl )
      return (*i)->obs(pred);
  }
  syslog(tlog::null, tlog::error)<<"Cannot create obs "<<tl<<"."<<pred
    <<" as \""<<tl<<"\" is not provided by this reactor";
  
  return nil;
}


// manipulators

void node_impl::kill() {
  {
    boost::unique_lock<boost::shared_mutex> write(m_mtx);
    m_dead = true;
  }
  m_exec->stop();
}

bool node_impl::start() {
  if( !dead() ) {
    m_exec->start();
    return true;
  }
  return false;
}


tlog::stream node_impl::syslog(symbol const &who,
                               symbol const &kind) const {
  std::ostringstream oss;
  boost::optional<date_type> cur = now(), last = last_synch(),
    boundary = m_externals.synch_target();
  SHARED_PTR<graph_impl> g = m_graph.lock();
  
  oss<<name();

  if( !who.empty() )
    oss<<"."<<who;
  
  if( cur ) {
    oss<<"][";
    if( !last )
      oss<<"?, ";
    else
      oss<<*last<<", ";
    if( boundary )
      oss<<*boundary<<", ";
    else
      oss<<"?, ";
    oss<<*cur;
  }
  
  if( g )
    return g->syslog(oss.str(), kind);
  else // special case where the node is no longer attached to a graph
    return m_log->syslog("<nil>."+oss.str(), kind);
}

// modifiers

void node_impl::set_latency(date_type const &val) {
  SHARED_PTR<graph_impl> g = m_graph.lock();
  if( g )
    g->strand().dispatch(boost::bind(&node_impl::g_strand_latency,
                                     shared_from_this(), val));
}

void node_impl::set_lookahead(date_type const &val) {
  SHARED_PTR<graph_impl> g = m_graph.lock();
  if( g )
    g->strand().dispatch(boost::bind(&node_impl::g_strand_lookahead,
                                     shared_from_this(), val));
}

void node_impl::attach(SHARED_PTR<reactor> r) {
  SHARED_PTR<graph_impl> g = m_graph.lock();
  if( g )
    g->strand().post(boost::bind(&node_impl::g_strand_attach,
                                     shared_from_this(), r));
}

void node_impl::provide(symbol const &tl, bool g, bool p) {
  SHARED_PTR<graph_impl> gr = m_graph.lock();
  
  if( gr ) {
    transaction_flags gp;
    if( g )
      gp.set(0);
    if( p )
      gp.set(1);
    gr->strand().post(boost::bind(&node_impl::g_strand_declare,
                                 shared_from_this(), tl, gp));
  }
}

void node_impl::use(symbol const &tl, bool g, bool p) {
  SHARED_PTR<graph_impl> gr = m_graph.lock();
  if( gr ) {
    transaction_flags gp;
    if( g )
      gp.set(0);
    if( p )
      gp.set(1);
    gr->strand().post(boost::bind(&node_impl::g_strand_use,
                                  shared_from_this(), tl, gp));
  }
}

void node_impl::post(token_ref obs) {
  if( obs ) {
    SHARED_PTR<graph_impl> g = m_graph.lock();
    if( g ) {
      g->strand().dispatch(boost::bind(&node_impl::g_strand_post,
                                       shared_from_this(), obs));
    }
  }
}


void node_impl::isolate() {
  syslog(tlog::null, tlog::warn)<<"Killed";
  SHARED_PTR<graph_impl> g = m_graph.lock();
  kill();
  if( g )
    g->strand().dispatch(boost::bind(&node_impl::g_strand_isolate,
                                     shared_from_this()));
}

// graph strand calls

void node_impl::g_strand_latency(date_type val) {
  SHARED_PTR<graph_impl> g = m_graph.lock();
  if( g ) {
    int delta = 0;
    {
      boost::upgrade_lock<boost::shared_mutex> test(m_mtx);
    
      if( m_latency!=val ) {
        boost::upgrade_to_unique_lock<boost::shared_mutex> lock(test);
      
        delta = val;
        delta -= m_latency;
        m_latency = val;
      }
    }
    if( delta!=0 ) {
      // TODO notify the timelines
    
      syslog(tlog::null, tlog::info)<<"Latency updated from "<<(val-delta)
      <<" to "<<val;
    }
  }
}

void node_impl::g_strand_lookahead(date_type val) {
  SHARED_PTR<graph_impl> g = m_graph.lock();
  if( g ) {
    int delta = 0;
    {
      boost::upgrade_lock<boost::shared_mutex> test(m_mtx);
    
      if( m_latency!=val ) {
        boost::upgrade_to_unique_lock<boost::shared_mutex> lock(test);
      
        delta = val;
        delta -= m_lookahead;
        m_lookahead = val;
      }
    }
    if( delta!=0 ) {
      // TODO notify the timelines
    
      syslog(tlog::null, tlog::info)<<"Lookahead updated from "<<(val-delta)
      <<" to "<<val;
    }
  }
}

void node_impl::g_strand_attach(SHARED_PTR<reactor> r) {
  SHARED_PTR<graph_impl> g = m_graph.lock();
  if( g ) {
    ERROR_CODE ec = g->g_strand_add(r);
    if( ec )
      syslog(tlog::null, tlog::error)<<"Error during insertion in graph:\n\t"
      <<ec;
    else {
      m_reactor = r;
      if( !dead() )
        m_exec->strand().dispatch(boost::bind(&node_impl::r_queue_init,
                                              shared_from_this()));
    }
  }
}

void node_impl::g_strand_declare(symbol tl,
                                 transaction_flags gp) {
  SHARED_PTR<graph_impl> g = m_graph.lock();
  SHARED_PTR<reactor> r = m_reactor.lock();
  
  if( g ) {
    if( is_external(tl) ) {
      syslog(tlog::null, tlog::error)<<"Cannot provide a timeline I already use ("<<tl<<")";
      if( r )
        m_exec->strand().dispatch(boost::bind(&reactor::disowned,
                                              r, tl));
   } else {
      ERROR_CODE ec;
      SHARED_PTR<internal_impl> i = g->g_strand_decl(shared_from_this(), tl, gp,
                                                   ec);
     if( ec ) {
        syslog(tlog::null, tlog::error)<<"Failed to declare timeline \""<<tl
        <<"\":\n\t- ("<<ec<<") "<<ec.message();
       if( r )
         m_exec->strand().dispatch(boost::bind(&reactor::disowned,
                                               r, tl));
       else
         syslog(tlog::null, tlog::error)<<"Not attached to a reactor ...";
     } else {
        boost::unique_lock<boost::shared_mutex> lock(m_mtx);
        m_internals.insert(i);
      }
    }
  }
}

void node_impl::g_strand_use(symbol tl,
                             transaction_flags gp) {
  SHARED_PTR<graph_impl> g = m_graph.lock();
  SHARED_PTR<reactor> r = m_reactor.lock();

  if( g ) {
    if( is_internal(tl) ) {
      syslog(tlog::null, tlog::error)<<"Cannot use a timeline I already provide ("<<tl<<")";
      if( r )
        m_exec->strand().dispatch(boost::bind(&reactor::unsubscribed,
                                              r, tl));
    } else {
      ERROR_CODE ec;
      SHARED_PTR<node_impl> nil;
      SHARED_PTR<internal_impl> i = g->g_strand_decl(nil, tl,
                                                     gp, ec);
      if( ec ) {
        syslog(tlog::null, tlog::error)<<"Failed to locate timeline \""<<tl
        <<"\":\n\t("<<ec<<") "<<ec.message();
        if( r )
          m_exec->strand().dispatch(boost::bind(&reactor::unsubscribed,
                                                r, tl));
      } else {
        SHARED_PTR<external_impl>
          e = MAKE_SHARED<external_impl>(shared_from_this(), i, gp);
        bool inserted = m_externals.insert(e);
        if( inserted ) {
          syslog(tlog::null, tlog::info)<<"Use \""<<tl<<"\"";
          e->g_strand_connect();
        }
      }
    }
  }
}

void node_impl::g_strand_synchronized(date_type date) {
  SHARED_PTR<reactor> r = m_reactor.lock();
  SHARED_PTR<graph_impl> g = m_graph.lock();
  if( g && r ) {
    boost::unique_lock<boost::shared_mutex> read(m_mtx);
    
    if( !m_dead ) {
      for(internal_set::iterator i=m_internals.begin();
          m_internals.end()!=i; ++i)
        (*i)->synchronized(date);
    }
  }
}


void node_impl::g_strand_isolate() {
  SHARED_PTR<reactor> r = m_reactor.lock();
  SHARED_PTR<node_impl> me = shared_from_this();
  SHARED_PTR<graph_impl> g = m_graph.lock();
  m_graph.reset();
  kill();
  
  if( g ) {
    internal_set tmp;
    {
      boost::unique_lock<boost::shared_mutex> lock(m_mtx);
      std::swap(tmp, m_internals);
    }
    internal_set::iterator i = tmp.begin();
    while( tmp.end()!=i ) {
      (*i)->g_strand_reset(me);
      i = tmp.erase(i);
    }
    m_externals.clear();
    
    if( r )
      g->g_strand_rm(r);
    g->syslog(tlog::null, tlog::warn)<<"Isolated reactor \""<<name()<<"\"";
  }
}

void node_impl::g_strand_lost(SHARED_PTR<internal_impl> tl) {
  SHARED_PTR<graph_impl> g = m_graph.lock();
  SHARED_PTR<reactor> r = m_reactor.lock();
  bool erased = false;
  if( g ) {
    boost::unique_lock<boost::shared_mutex> lock(m_mtx);
    erased = m_internals.erase(tl);
  }
  if( erased ) {
    syslog(tlog::null, tlog::warn)<<"lost ownership of "<<tl->name();
    if( r )
      m_exec->strand().dispatch(boost::bind(&reactor::disowned,
                                            r, tl->name()));
  }
}

void node_impl::g_strand_post(token_ref obs) {
  SHARED_PTR<reactor> r = m_reactor.lock();
  SHARED_PTR<graph_impl> g = m_graph.lock();
  SHARED_PTR<internal_impl> tl;

  if( g && r ) {
    boost::shared_lock<boost::shared_mutex> read(m_mtx);
    if( m_dead ) {
      syslog(tlog::null, tlog::error)<<"Zombie post: "<<(*obs);
      return;
    }
    internal_set::const_iterator const endi = m_internals.end();
    internal_set::const_iterator const i = name_lower_bound(m_internals.begin(),
                                                            endi, obs->object());
    if( endi!=i && (*i)->name()==obs->object() )
      tl = *i;
  }
  if( tl ) {
    ERROR_CODE ec = tl->g_strand_post(obs);
    if( ec ) {
      syslog(tl->name(), tlog::error)<<"Failed to post observation:"
      <<"\n\t- "<<(*obs)
      <<"\n\t- error: ("<<ec<<") "<<ec.message();
    }
  } else {
    syslog(tlog::null, tlog::error)<<"Failed to post observation:"
      <<"\n\t- "<<(*obs)
      <<"\n\t- error: "<<obs->object()<<" not owned by this reactor";
  }
}

void node_impl::g_strand_tick(bs2::connection const &c,
                              date_type tick) {
  SHARED_PTR<graph_impl> g = m_graph.lock();
  if( !g || dead() )
    c.disconnect();
  else if( m_reactor.lock() ) {
    m_exec->send(boost::bind(&node_impl::r_queue_tick,
                             shared_from_this(), tick),
                 tick_event);
  }
}

void node_impl::g_strand_notify(details::date_type tick,
                                token_id const &obs,
                                bool fresh) {
  SHARED_PTR<graph_impl> g = m_graph.lock();
  if( g && !dead() ) {
    ERROR_CODE ec;
    bool synch = m_externals.update(obs->object(), tick, ec);
    
    if( ec ) {
      syslog(tlog::null, tlog::error)<<"Error on observation handling:"
      <<"\n\t- obs: "<<(*obs)
      <<"\n\t- ("<<ec<<") "<<ec.message();
      return;
    } else {
      m_exec->send(boost::bind(&node_impl::r_queue_notify,
                             shared_from_this(),
                             tick, obs, fresh),
                   obs_event);
      if( synch )
        m_exec->send(boost::bind(&node_impl::r_queue_synch,
                                 shared_from_this()),
                     synch_event);
    }
  }
}


// reactor strand protected calls

void node_impl::r_queue_init() {
  SHARED_PTR<reactor> r = m_reactor.lock();
  SHARED_PTR<graph_impl> g = m_graph.lock();
  if( g && r ) {
    if( m_exec->is_active() ) {
      syslog(tlog::null, tlog::error)<<"Reactor running before being inited";
    }
    try {
      r->init_complete();
      if( !start() )
        syslog(tlog::null, tlog::warn)<<"Reactor is dead after init";
      return;
    } catch(std::exception const &e) {
      syslog(tlog::null, tlog::error)<<"Exception caught during init:\n\t"
                                     <<e.what();
    } catch(...) {
      syslog(tlog::null, tlog::error)<<"Unknown exception caught during init";
    }
    isolate();
  }
}

void node_impl::r_queue_tick(date_type val) {
  SHARED_PTR<reactor> r = m_reactor.lock();
  if( r && !dead() ) {
    bool do_synch = m_externals.r_queue_tick(val);
    
    try {
      r->new_tick(val);
      if( do_synch )
        m_exec->send(boost::bind(&node_impl::r_queue_synch,
                                 shared_from_this()),
                     synch_event);
      return;
    } catch(std::exception const &e) {
      syslog(tlog::null, tlog::error)<<"Exception during new_tick:\n\t"
                                     <<e.what();
    } catch(...) {
      syslog(tlog::null, tlog::error)<<"Unknown exception during new_tick";
    }
    isolate();
  }
}

void node_impl::r_queue_notify(date_type val, token_id obs, bool fresh) {
  SHARED_PTR<reactor> r = m_reactor.lock();
  if( r ) {
    if( fresh ) {
      try {
        r->notify(obs);
      } catch(std::exception const &e) {
        syslog(tlog::null, tlog::error)<<"exception during notification:"
        <<"\n\t- obs "<<(*obs)
        <<"\n\t- error "<<e.what();
      } catch(...) {
        syslog(tlog::null, tlog::error)<<"unknown exception during notification:"
        <<"\n\t- obs "<<(*obs);
      }
    }
  }
}

void node_impl::r_queue_synch() {
  SHARED_PTR<reactor> r = m_reactor.lock();
  
  if( r && !dead() && m_externals.should_synch() ) {
    ERROR_CODE ec = reactor_error_code(reactor_error::ok);
    boost::optional<date_type> new_synch;
    date_type nxt_synch = *m_externals.next_synch(),
      max_synch = *m_externals.synch_target(),
      delta = max_synch-nxt_synch;
    
    if( delta>=1 )
      syslog(tlog::null, tlog::warn)<<"Synchronization is "<<delta
        <<" tick"<<(delta>1?"s":"")<<" late";
    
    try {
      new_synch = r->synchronize(nxt_synch, max_synch, ec);
    } catch(SYSTEM_ERROR const &se) {
      ec = se.code();
    } catch(std::exception const &e) {
      syslog(tlog::null, tlog::error)<<"Exception during synchronize:\n\t"
      <<e.what();
      ec = reactor_error_code(reactor_error::failed_to_synchronize);
    } catch(...) {
      syslog(tlog::null, tlog::error)<<"Unknown exception during synchronize";
      ec = reactor_error_code(reactor_error::failed_to_synchronize);
    }
    
    if( ec ) {
      syslog(tlog::null, tlog::error)<<"Synchronization error (target=["
      <<nxt_synch<<", "<<max_synch<<"]):\n\t("<<ec<<") "<<ec.message();
      isolate();
    } else {
      bool do_synch, changed;
      
      if( new_synch ) {
        do_synch = m_externals.r_queue_synchronized(*new_synch, changed);
        if( changed ) {
          boost::unique_lock<boost::shared_mutex> lock(m_mtx);
          internal_set::const_iterator i;
          for(i=m_internals.begin(); m_internals.end()!=i; ++i)
            (*i)->synchronized(*new_synch);
        }
      } else
        do_synch = true;
      if( do_synch )
        m_exec->send(boost::bind(&node_impl::r_queue_synch,
                                 shared_from_this()),
                     synch_event);
    }
  }
}

