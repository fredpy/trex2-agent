/** @file "SingletonDummy.cc"
 * @brief SingletonDummy internal class implmentation
 *
 * @author Frederic Py <fpy@mbari.org>
 * @ingroup utils
 */
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
#include "private/SingletonServer.hh"

using namespace TREX::utils::internal;

// statics 

SingletonDummy *SingletonDummy::attach(std::string const &name, 
                                       sdummy_factory const &factory) {
  SingletonServer::lock_type locker(SingletonServer::sing_mtx());
  return SingletonServer::instance().attach(name, factory);
}

void SingletonDummy::detach(std::string const &name) {
  SingletonServer::lock_type locker(SingletonServer::sing_mtx());
  if( SingletonServer::instance().detach(name) ) {
    delete SingletonServer::s_instance;
  }
}


// structors 

SingletonDummy::SingletonDummy() 
  :ref_counter(0ul) {}

SingletonDummy::~SingletonDummy() {}

// modifers

void SingletonDummy::incr_ref() const {
  ++ref_counter;
}

bool SingletonDummy::decr_ref() const {
  return (ref_counter--)<=1;
}


