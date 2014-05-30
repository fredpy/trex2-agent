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

priority_strand::priority_strand(asio::io_service &io)
:m_strand(io) {}

priority_strand::~priority_strand() {}

// observers

size_t priority_strand::tasks() const {
  boost::shared_lock<boost::shared_mutex> lock(m_mutex);
  return m_tasks.size();
}

bool priority_strand::empty() const {
  boost::shared_lock<boost::shared_mutex> lock(m_mutex);
  return m_tasks.empty();
}

// manipulators

void priority_strand::enqueue(priority_strand::task *t) {
  {
    boost::unique_lock<boost::shared_mutex> lock(m_mutex);
    m_tasks.push(t);
  }
  m_strand.post(boost::bind(&priority_strand::dequeue_sync, this));
}

void priority_strand::dequeue_sync() {
  task *nxt;
  {
    boost::upgrade_lock<boost::shared_mutex> test(m_mutex);
    
    if( m_tasks.empty() )
      return;
    else {
      boost::upgrade_to_unique_lock<boost::shared_mutex> lock(test);
      nxt = m_tasks.top();
      m_tasks.pop();
    }
  }
  nxt->execute();
  delete nxt;
}
