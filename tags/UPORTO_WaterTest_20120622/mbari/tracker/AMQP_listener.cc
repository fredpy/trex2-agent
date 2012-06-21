/*********************************************************************
 * Software License Agreement (BSD License)
 * 
 *  Copyright (c) 2011, MBARI.
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
#include "AMQP_listener.hh"

using namespace mbari::amqp;
using namespace TREX::utils;

listener::listener(queue_ref const &q, listener::handler &handle) 
  :m_running(false), m_queue(q), m_handler(&handle) {}

listener::listener(listener const &other) 
  :m_running(false), m_queue(other.m_queue), m_handler(other.m_handler) {}

listener::~listener() {
  stop();
}

bool listener::is_running() const {
  SharedVar<bool>::scoped_lock lock(m_running);
  return *m_running;
}

void listener::run() {
  m_running = true;
  try {
    while( is_running() ) {
      // std::cerr<<"Listening to queue ... "<<std::endl;
      m_handler->handle(m_queue->consume());
      boost::thread::yield();
    }
  } catch(...) {
    m_running = false;
    throw;
  }
  // std::cerr<<"EOT"<<std::endl;
}

bool msg_buffer::empty() const {
  SharedVar<queue_type>::scoped_lock lock(m_queue);
  return m_queue->empty();
}

boost::shared_ptr<queue::message> msg_buffer::pop() {
  boost::shared_ptr<queue::message> result;

  {
    SharedVar<queue_type>::scoped_lock lock(m_queue);
    if( !m_queue->empty() ) {
      result = m_queue->front();
      m_queue->pop_front();
    }
  }
  return result;
}

void msg_buffer::handle(boost::shared_ptr<queue::message> const &msg) {
    SharedVar<queue_type>::scoped_lock lock(m_queue);
    m_queue->push_back(msg);
}
