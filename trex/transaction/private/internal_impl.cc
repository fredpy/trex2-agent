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
#include "../graph_error.hh"

#include <trex/utils/asio_runner.hh>

using namespace TREX::transaction;
namespace utils=TREX::utils;
namespace tlog=utils::log;
namespace bs2=boost::signals2;

using utils::symbol;
using details::internal_impl;

std::string TREX::transaction::details::access_str
(details::transaction_flags const &gp) {
  std::string ret("--");
  if( gp[0] )
    ret[0] = 'g';
  if( gp[1] )
    ret[1] = 'p';
  return ret;
}


/*
 * class TREX::transaction::details::tl_cmp
 */

bool details::tl_cmp::operator()(SHARED_PTR<internal_impl> const &a,
                                 SHARED_PTR<internal_impl> const &b) const {
  return a->name()<b->name();
}

/*
 * class TREX::transaction::details::internal_impl
 */

// statics

symbol const internal_impl::s_failed("Failed");

// structors

internal_impl::internal_impl(SHARED_PTR<details::graph_impl> const &g,
                             symbol const &n)
:m_graph(g), m_name(n) {}

internal_impl::~internal_impl() {}

// observers

bool internal_impl::accept_goals() const {
  boost::shared_lock<boost::shared_mutex> read(m_mtx);
  SHARED_PTR<node_impl> r = m_owner.lock();
  return bool(r) && m_gp[0] && r->lookahead()>0;
}

bool internal_impl::publish_plan() const {
  boost::shared_lock<boost::shared_mutex> read(m_mtx);
  SHARED_PTR<node_impl> r = m_owner.lock();
  return bool(r) && m_gp[1];
}

details::date_type internal_impl::latency() const {
  boost::shared_lock<boost::shared_mutex> read(m_mtx);
  SHARED_PTR<node_impl> r = m_owner.lock();
  if( r )
    return r->latency();
  return 0;
}


details::date_type internal_impl::lookahead() const {
  boost::shared_lock<boost::shared_mutex> read(m_mtx);
  SHARED_PTR<node_impl> r = m_owner.lock();
  if( r && m_gp[0] )
    return r->lookahead();
  return 0;
}

boost::optional<details::date_type> internal_impl::now() const {
  boost::optional<date_type> ret;
  SHARED_PTR<graph_impl> g = m_graph.lock();
  if( g )
    ret = g->now();
  return ret;
}

boost::optional<details::date_type> internal_impl::last_synch(token_id &obs) const {
  boost::shared_lock<boost::shared_mutex> read(m_mtx);
  obs = m_last_obs;
  return m_synch_date;
}

std::string internal_impl::access_str() const {
  boost::shared_lock<boost::shared_mutex> read(m_mtx);
  return details::access_str(m_gp);
}

// manipulators

tlog::stream internal_impl::syslog(symbol const &kind) const {
  SHARED_PTR<node_impl> r = owner();
  if( r )
    return r->syslog(name(), kind);
  else // this could produce an error if I log when m_graph is destroyed
    return m_graph.lock()->syslog(name(), kind);
}

token_ref internal_impl::obs(symbol const &pred) const {
  token_ref ret = token::obs(name(), pred);
  boost::optional<date_type> cur = last_synch();
  
  if( cur )
    ret->restrict_start(int_domain(1+*cur, int_domain::plus_inf));
  return ret;
}

// modifiers

void internal_impl::synchronized(details::date_type tick) {
  SHARED_PTR<graph_impl> g = m_graph.lock();
  if( g )
    g->strand().post(boost::bind(&internal_impl::g_strand_synch,
                                 shared_from_this(), tick));
}

// Graph strand functions

ERROR_CODE internal_impl::g_strand_set_owner
(SHARED_PTR<details::node_impl> const &r,
 details::transaction_flags const &gp) {
  SHARED_PTR<graph_impl> g = m_graph.lock();
  if( g ) {
    boost::upgrade_lock<boost::shared_mutex> test(m_mtx);
    SHARED_PTR<node_impl> o = m_owner.lock();

    if( !o ) {
      boost::upgrade_to_unique_lock<boost::shared_mutex> lock(test);
      m_owner = r;
      m_gp = gp;
      r->syslog(tlog::null, tlog::info)<<" provide "<<name()
      <<" with access ["<<details::access_str(m_gp)<<"]";
    } else if( o==r ) {
      if( m_gp!=gp ) {
        boost::upgrade_to_unique_lock<boost::shared_mutex> lock(test);
        m_gp = gp;
        syslog(tlog::info)<<"timeline access changed to ["
                          <<details::access_str(m_gp)<<"]";
      }
    } else
      return graph_error_code(graph_error::already_owned);
    return graph_error_code(graph_error::ok);
  } else
    return graph_error_code(graph_error::invalid_graph);
}

ERROR_CODE internal_impl::g_strand_reset(SHARED_PTR<node_impl> const &r) {
  SHARED_PTR<graph_impl> g = m_graph.lock();
  if( g ) {
    boost::upgrade_lock<boost::shared_mutex> test(m_mtx);
    SHARED_PTR<node_impl> o = m_owner.lock();
    if( o==r ) {
      if( r ) {
        boost::upgrade_to_unique_lock<boost::shared_mutex> lock(test);
        m_owner.reset();
        m_gp.reset();
        m_next_obs.reset();
        r->g_strand_lost(shared_from_this());
        r->syslog(tlog::null, tlog::info)<<"Unprovided timeline "<<name();
      }
      return graph_error_code(graph_error::ok);
    } else
      return graph_error_code(graph_error::already_owned);
  } else
    return graph_error_code(graph_error::invalid_graph);
}



void internal_impl::g_strand_tick(bs2::connection const &c,
                                  details::date_type tick) {
  SHARED_PTR<graph_impl> g = m_graph.lock();
  if( !g )
    c.disconnect();
  else if( !owned() ) {
    if( m_last_obs ) {
      if( m_last_obs->predicate()==s_failed )
        g_strand_post(obs(s_failed));
      g_strand_synch(tick);
    } else
      syslog(tlog::warn)<<"Still undefined at tick "<<tick;
  }
}

ERROR_CODE internal_impl::g_strand_post(token_ref next) {
  if( !next || next->object()!=name() )
    return graph_error_code(graph_error::invalid_post_object);
  else {
    boost::optional<date_type> last = last_synch();
    SHARED_PTR<graph_impl> g = m_graph.lock();
    ERROR_CODE ec = graph_error_code(graph_error::ok);
  
    if( g ) {
      if( last ) {
        next->restrict_start(int_domain(1+*last, int_domain::plus_inf), ec);
        if( ec ) {
          syslog(tlog::error)<<"Failed to restrict new obs start to ["
                             <<(1+*last)<<",+inf]:\n\t"<<(*next);
          return ec;
        }
      }
      std::swap(m_next_obs, next);
      if( next ) {
        syslog(tlog::warn)<<"New obs overwrite previous pending one:"
        <<"\n\t- old: "<<(*next);
      }
      return ec;
    }
    return graph_error_code(graph_error::invalid_graph);
  }
}

void internal_impl::g_strand_synch(details::date_type date) {
  SHARED_PTR<graph_impl> g = m_graph.lock();
  if( g ) {
    boost::optional<date_type> last = last_synch(), cur = now();
    if( !cur )
      syslog(tlog::error)<<"Cannot synchronize to "<<date
                         <<" before initial tick.";
    else if( date>*cur )
      syslog(tlog::error)<<"Cannot synchronize in the future ("<<date<<")";
    else if( last && date<=*last )
      syslog(tlog::error)<<"Cannot synchronize in the past ("<<date<<"<="
        <<(*last)<<")";
    else {
      bool constrained = false;
      if( m_next_obs ) {
        int_domain::bound lo = m_next_obs->start().lower_bound();
        if( lo>date )
          syslog(tlog::warn)<<"Have a pending observation in the future:"
          <<"\n\tsynch date: "<<date
          <<"\n\tnex obs: "<<(*m_next_obs);
        else {
          if( m_last_obs ) {
            int_domain::bound last_lo = m_last_obs->end().lower_bound();
            if( last_lo>lo )
              lo = last_lo;
          }
          if( lo>date ) {
            syslog(tlog::error)<<"Attempted to start a new observation at "
            <<date<<" while last observation is still ends after this tick:\n\t"
            <<(*m_last_obs);
          } else {
            if( last && lo<*last )
              lo = *last;
            ERROR_CODE ec;
            m_next_obs->restrict_time(int_domain(lo, date),
                                      token::s_duration_full,
                                      int_domain(date+1, int_domain::plus_inf),
                                      ec);
            if( ec ) {
              syslog(tlog::error)<<"Failed to restrict temporal scope of "
              <<" next obs to contain tick "<<date<<":\n\t"<<(*m_next_obs);
              g_strand_reset(m_owner.lock());
              return; // TODO need to fail by detaching from the reactor
            }
            constrained = true;
            if( m_last_obs ) {
              m_last_obs->restrict_end(int_domain(lo, m_next_obs->start().upper_bound()), ec);
              if( ec ) {
                syslog(tlog::error)<<"Failed to constrain previous state "
                <<"to end before new obs:\n\t- prev: "<<(*m_last_obs)
                <<"\n\t- new: "<<(*m_next_obs);
                g_strand_reset(m_owner.lock());
                return;
              }
            }
            m_last_obs = m_next_obs;
            m_next_obs.reset();
            syslog(tlog::info)<<(*m_last_obs)<<" {start="<<m_last_obs->start()
              <<", end="<<m_last_obs->end()<<"}";
          }
        }
      }
      if( !constrained ) {
        if( !m_last_obs ) {
          syslog(tlog::error)<<"Attempted to synchronize with no observation ("
          <<date<<").";
          g_strand_reset(m_owner.lock());
          return;
        } else {
          ERROR_CODE ec;
          m_last_obs->restrict_end(int_domain(date+1, int_domain::plus_inf),
                                   ec);
          if( ec ) {
            syslog(tlog::error)<<"Failed to extend current obs to current tick ("
            <<date<<")\n\t-"<<(*m_last_obs);
            g_strand_reset(m_owner.lock());
            return;
          }
        }
      }
      if( m_last_obs->end().lower_bound()>date+1 )
        syslog(tlog::warn)<<"Current state cannot end before "
        <<m_last_obs->end().lower_bound().value();
      {
        boost::unique_lock<boost::shared_mutex> lock(m_mtx);
        m_synch_date = date;
      }
      m_synch(date, m_last_obs);
    }
  }
}



//// modifiers
//// manipulators
//
//token_ref internal_impl::create_obs(symbol const &pred) const {
//  // TOODO protect this in a strand
//  boost::optional<date_type> cur = last_synch();
//  token_ref ret = token::obs(name(), pred);
//  if( cur ) {
//    int_domain future(1+*cur, int_domain::plus_inf);
//    ret->restrict_start(future);
//  }
//  return ret;
//}
//
//// graph strand protected
//
//void internal_impl::update_synch(date_type d) {
//  boost::optional<date_type> prev = last_synch(), cur = now();
//  
//  if( !cur ) {
//    syslog(tlog::error)<<"Attempt to synchronize for tick "<<d<<" while "
//    "clock has no initial date";
//  } else if( d>*cur )
//    syslog(tlog::error)<<"Cannot synchronize in the future ("<<d<<")";
//  else if( prev && d<=*prev )
//    syslog(tlog::error)<<"Cannot synchronize in the past ("<<d<<"<="<<*prev<<")";
//  else if( m_next_obs ) {
//    int_domain::bound lo = int_domain::minus_inf;
//    if( prev )
//      lo = *prev;
//    if( m_last_obs && m_last_obs->end().lower_bound()>lo )
//      lo = m_last_obs->end().lower_bound();
//    
//    if( lo>d ) {
//      syslog(tlog::error)<<"Attempted to start a new observation at tick "
//      <<d<<" while previous observation is not done yet:\n\t"
//      <<*m_last_obs;
//      return;
//    }
//    
//    ERROR_CODE ec;
//    m_next_obs->restrict_time(int_domain(lo, d),
//                              token::s_duration_full,
//                              int_domain(d+1, int_domain::plus_inf),
//                              ec);
//    if( ec ) {
//      syslog(tlog::error)<<"Failed to restrict temporal scope of next"
//      <<" observation to contain tick "<<d<<"\n\t-"<<(*m_next_obs);
//      return;
//    }
//    if( m_last_obs ) {
//      m_last_obs->restrict_end(int_domain(lo, m_next_obs->end().upper_bound()),
//                               ec);
//      if( ec ) {
//        syslog(tlog::error)<<"Failed to constrain previous state to finish "
//        <<"before new obs:\n\t-prev: "<<(*m_last_obs)
//        <<"\n\t-new: "<<(*m_next_obs);
//        return;
//      }
//    }
//    m_last_obs = m_next_obs;
//    m_next_obs.reset();
//    syslog(tlog::info)<<"["<<d<<"] "<<(*m_last_obs)<<"\n\t"
//    <<m_last_obs->start();
//    m_synch_date = d;
//    m_synch(d, m_last_obs);
//  } else if( !m_last_obs ) {
//    syslog(tlog::error)<<"Attempted to synchronize to tick ("<<d<<" with no observation";
//  } else {
//    ERROR_CODE ec;
//    m_last_obs->restrict_end(int_domain(d+1, int_domain::plus_inf), ec);
//    if( ec ) {
//      syslog(tlog::error)<<"Failed to extend current obs to tick ("<<d
//      <<"):\n\t- "<<(*m_last_obs);
//      // TODO I may deduce an alternate tick here
//    }
//    m_synch_date = d;
//    m_synch(d, m_last_obs);
//  }
//}
//
///*
// * struct TREX::transaction::details::tl_cmp
// */
//
//bool details::tl_cmp::operator()(SHARED_PTR<internal_impl> const &a,
//                                 SHARED_PTR<internal_impl> const &b) const {
//  return a->name()<b->name();
//}
//
//
//
//
//
//
