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
  m_strand->dispatch(boost::bind(&graph_impl::set_date_sync, shared_from_this(), d));
}

boost::optional<details::graph_impl::date_type> details::graph_impl::get_date(bool fast) const {
  if( fast ) {
    utils::SharedVar< boost::optional<date_type> >::scoped_lock lock(m_date);
    return *m_date;
  } else {
    boost::function<boost::optional<date_type> ()> fn(boost::bind(&graph_impl::get_date, this, true));
    return utils::strand_run(strand(), fn);
  }
}

tlog::stream details::graph_impl::syslog(Symbol const &ctx,
                                         Symbol const &kind) const {
  boost::optional<date_type> cur = get_date(true);
  Symbol who = m_name;
  if( !ctx.empty() )
    who = who.str()+"."+ctx.str();
  if( cur )
    return m_log->syslog(*cur, who, kind);
  else
    return m_log->syslog(who, kind);
}


// strand protected calls

void details::graph_impl::set_date_sync(details::graph_impl::date_type date) {
  m_date = date;
}