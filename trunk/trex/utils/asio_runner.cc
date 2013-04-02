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
#include "asio_runner.hh"

using namespace TREX::utils;
using namespace boost::asio;

/*
 * class TREX::utils::asio_runner
 */

// structors 

asio_runner::asio_runner() {
  // Create a work for maintaining our service 
  m_active.reset(new io_service::work(m_io));
  // spawn a first thread that manages our service
  //spawn(1);
}

asio_runner::~asio_runner() {
  // complete our work so threads can complete
  m_active.reset();
  // wait for our threads to join:
  //  can take time if they still have tasks to do
  m_threads.join_all();
}

// manipulators

size_t asio_runner::thread_count(size_t n, bool override) {  
  // correct the number of threads
  if( !override ) {
    // Max thread is number of cores minus the main thread
    size_t max_count = boost::thread::hardware_concurrency()-1;
    
    if( n > max_count )
      n=max_count;
  }

  size_t cur = thread_count();
  if( cur < n ) {
    // spawn the number of threads needed to reach n
    spawn(n-cur);
    return thread_count();
  }
  return cur;
}

void asio_runner::spawn(size_t n) {
  for(size_t i=0; i<n; ++i) 
    m_threads.create_thread(boost::bind(&io_service::run, boost::ref(m_io)));
}


