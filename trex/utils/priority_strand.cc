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
#include "private/priority_strand_impl.hh"

using namespace TREX::utils;
namespace asio=boost::asio;

/*
 * class TREX::utils::priority_strand::task
 */

bool priority_strand::task::operator< (priority_strand::task const &other) const {
  return other.m_level && ( !m_level || (*other.m_level)<(*m_level) );
}

/*
 * class TREX::utils::priority_strand
 */

// structors

priority_strand::priority_strand(asio::io_service &io, bool active)
:m_impl(MAKE_SHARED<pimpl>(MAKE_SHARED<asio::strand>(boost::ref(io)))) {
  if( active )
    m_impl->start();
}

priority_strand::priority_strand(SHARED_PTR<asio::strand> const &s,
                                 bool active)
:m_impl(MAKE_SHARED<pimpl>(s)) {
  if( active )
    m_impl->start();
}


priority_strand::~priority_strand() {}

// observers

size_t priority_strand::tasks() const {
  return m_impl->tasks();
}

bool priority_strand::empty() const {
  return m_impl->empty();
}

bool priority_strand::is_active() const {
  return m_impl->active();
}

// modifiers

void priority_strand::start() {
  m_impl->start();
}

void priority_strand::stop() {
  m_impl->stop();
}

// manipulators

boost::asio::strand &priority_strand::strand() {
  return m_impl->strand();
}


void priority_strand::enqueue(priority_strand::task *t) {
  m_impl->enqueue(t);
}

void priority_strand::clear() {
  m_impl->clear();
}

