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
#include "priority_strand_impl.hh"

using namespace TREX::utils;
namespace asio=boost::asio;

/*
 * class TREX::utils::priority_strand::pimpl
 */

// structors

priority_strand::pimpl::pimpl(std::shared_ptr<strand_type> const &s)
:m_strand(s), m_active(false), m_running(false) {}

priority_strand::pimpl::~pimpl() {
  clear();
}

// observers

bool priority_strand::pimpl::active() const {
  boost::shared_lock<boost::shared_mutex> lock(m_mutex);
  return m_active;
}

bool priority_strand::pimpl::completed() const {
  boost::shared_lock<boost::shared_mutex> lock(m_mutex);
  return !( m_active || m_running );
}


size_t priority_strand::pimpl::tasks() const {
  boost::shared_lock<boost::shared_mutex> lock(m_mutex);
  return m_queue.size();
}

bool priority_strand::pimpl::empty() const {
  boost::shared_lock<boost::shared_mutex> lock(m_mutex);
  return m_queue.empty();
}

// modifiers

void priority_strand::pimpl::enqueue(priority_strand::pimpl::task_ref tsk) {
  std::shared_ptr<pimpl> me = shared_from_this();
  bool should_post = false;
  
  {
    boost::unique_lock<boost::shared_mutex> lock(m_mutex);
    should_post = m_queue.empty() && m_active;
    m_queue.push(tsk);
  }
  if( should_post )
    m_strand->post(boost::bind(&priority_strand::pimpl::dequeue_sync,
                               me));
}

void priority_strand::pimpl::clear() {
  task_queue tmp;
  {
    boost::unique_lock<boost::shared_mutex> lock(m_mutex);
    std::swap(tmp, m_queue);
  }
  while( !tmp.empty() ) {
    task_ref nxt = tmp.top();
    tmp.pop();
    delete nxt;
  }
}

void priority_strand::pimpl::start() {
  std::shared_ptr<pimpl> me = shared_from_this();
  bool was_active = true, queued_tasks;
  {
    boost::upgrade_lock<boost::shared_mutex> test(m_mutex);
    queued_tasks = !m_queue.empty();
    {
      boost::upgrade_to_unique_lock<boost::shared_mutex> lock(test);
      std::swap(m_active, was_active);
    }
  }
  if( queued_tasks && !was_active )
    m_strand->post(boost::bind(&priority_strand::pimpl::dequeue_sync,
                               me));
}

void priority_strand::pimpl::stop() {
  boost::unique_lock<boost::shared_mutex> lock(m_mutex);
  m_active = false;
}

// strand protected functions

void priority_strand::pimpl::dequeue_sync() {
  std::shared_ptr<pimpl> me = shared_from_this();
  task_ref nxt;
  bool should_post = false;
  
  {
    boost::upgrade_lock<boost::shared_mutex> test(m_mutex);
    
    if( m_queue.empty() || !m_active ) {
      return; // nothing to do here ... lets stop
    } else {
      boost::upgrade_to_unique_lock<boost::shared_mutex> lock(test);
      m_running = true;
      nxt = m_queue.top();
      m_queue.pop();
    }
    should_post = !m_queue.empty();
  }
  // Execute this task
  try {
    nxt->execute();
    delete nxt;
  } catch(...) {
    // just skip the exceptions
  }
  {
    boost::unique_lock<boost::shared_mutex> lock(m_mutex);
    m_running = false;
  }
  if( should_post )
    m_strand->post(boost::bind(&priority_strand::pimpl::dequeue_sync,
                               me));
}

/*
 * struct priority_strand::pimpl::task_cmp
 */

bool priority_strand::pimpl::task_cmp::operator()
(priority_strand::pimpl::task_ref a,
 priority_strand::pimpl::task_ref b) const {
  return a->operator<(*b);
}



