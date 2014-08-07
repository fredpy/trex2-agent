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
#include "clock_impl.hh"

using namespace TREX::transaction;
namespace asio=boost::asio;

/*
 * class TREX::transaction::details::clock
 */

// structors

details::clock::clock(asio::io_service &io)
:m_strand(io), m_started(false) {}

details::clock::~clock() {}

// observers

bool details::clock::started() const {
  boost::shared_lock<mutex_type> lock(m_mutex);
  return m_started;
}

boost::optional<details::clock::date_type> details::clock::read_date() const {
  boost::shared_lock<mutex_type> lock(m_mutex);
  return m_date;
}

// modifiers

void details::clock::set_date(details::clock::date_type const &val) {
  boost::function<void ()> fn(boost::bind(&clock::set_date_sync,
                                          shared_from_this(), val));
  
  utils::strand_run(m_strand, fn);
}

void details::clock::set_started(bool flag) {
  // TODO: this may be overkill and could present a risk on
  //       thread starvation. It may be better to just do the
  //       update call directly instread of transiting through
  //       a strand
  m_strand.dispatch(boost::bind(&clock::set_start_sync,
                                shared_from_this(), flag));
}

// strand protected methods

void details::clock::set_date_sync(details::clock::date_type date) {
  boost::upgrade_lock<mutex_type> lock(m_mutex);
  
  // update date only if date is fresher than previous update
  // TODO: I may want to give some wraning when the date was
  //       not updated due to the above
  if( !m_date || *m_date<date ) {
    // get unique access
    boost::upgrade_to_unique_lock<mutex_type> unique(lock);
    m_date = date;
  }
}

void details::clock::set_start_sync(bool flag) {
  boost::upgrade_lock<mutex_type> lock(m_mutex);
  
  if( flag!=m_started ) {
    // get unique access to change the value
    boost::upgrade_to_unique_lock<mutex_type> unique(lock);
    m_started = flag;
  }
}


