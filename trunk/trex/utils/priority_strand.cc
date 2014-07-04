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
#include "priority_strand.hh"

using namespace TREX::utils;
namespace asio=boost::asio;

/*
 * class TREX::utils::priority_strand::task
 */

bool priority_strand::task::operator< (priority_strand::task const &other) const {
  return other.m_level && ( !m_level || (*other.m_level)<(*m_level) );
}

/*
 * struct TREX::utils::priority_strand::task_cmp
 */

bool priority_strand::tsk_cmp::operator()(priority_strand::task *a,
                                          priority_strand::task *b) const {
  return a->operator<(*b);
}

/*
 * class TREX::utils::priority_strand
 */

// structors

priority_strand::priority_strand(asio::io_service &io, bool active)
:m_strand(io), m_active(false) {
  if( active )
    start();
}

priority_strand::~priority_strand() {
  clear();
  stop();
}

// observers

size_t priority_strand::tasks() const {
  boost::shared_lock<boost::shared_mutex> lock(m_mutex);
  return m_tasks.size();
}

bool priority_strand::empty() const {
  boost::shared_lock<boost::shared_mutex> lock(m_mutex);
  return m_tasks.empty();
}

bool priority_strand::is_active() const {
  boost::shared_lock<boost::shared_mutex> lock(m_mutex);
  return m_active;
}

// modifiers

void priority_strand::start() {
  bool was_active = true, queued_tasks;
  {
    boost::upgrade_lock<boost::shared_mutex> test(m_mutex);
    queued_tasks = !m_tasks.empty();
    {
      boost::upgrade_to_unique_lock<boost::shared_mutex> lock(test);
      std::swap(m_active, was_active);
    }
  }
  if( queued_tasks && !was_active )
    m_strand.post(boost::bind(&priority_strand::dequeue_sync, this));
}

void priority_strand::stop() {
  boost::unique_lock<boost::shared_mutex> lock(m_mutex);
  m_active = false;
}



// manipulators

void priority_strand::enqueue(priority_strand::task *t) {
  bool should_post = false;
  
  {
    boost::unique_lock<boost::shared_mutex> lock(m_mutex);
    should_post = m_tasks.empty() && m_active;
    m_tasks.push(t);
  }
  if( should_post )
    m_strand.post(boost::bind(&priority_strand::dequeue_sync, this));
}

void priority_strand::dequeue_sync() {
  task *nxt;
  bool should_post = false;
  {
    boost::upgrade_lock<boost::shared_mutex> test(m_mutex);
    
    if( m_tasks.empty() || !m_active )
      return;
    else {
      boost::upgrade_to_unique_lock<boost::shared_mutex> lock(test);
      nxt = m_tasks.top();
      m_tasks.pop();
    }
    should_post = !m_tasks.empty();
  }
  nxt->execute();
  delete nxt;
  if( should_post )
    m_strand.post(boost::bind(&priority_strand::dequeue_sync, this));
}

void priority_strand::clear() {
  task_queue tmp;
  {
    boost::unique_lock<boost::shared_mutex> lock(m_mutex);
    std::swap(tmp, m_tasks);
  }
  while( !tmp.empty() ) {
    task *nxt = tmp.top();
    tmp.pop();
    delete nxt;
  }
}

