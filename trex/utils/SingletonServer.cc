/** @file "SingletonServer.cc"
 * @brief SingletonServer internal class implmentation
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
#include "bits/SingletonServer.hh"

# include <iostream>
# include <cxxabi.h> 

using namespace TREX::utils::internal;

// statics 

SingletonServer *SingletonServer::s_instance = 0x0;


SingletonServer &SingletonServer::instance() {
  if( 0x0==s_instance ) 
    new SingletonServer;
  return *s_instance;
}

SingletonServer::mutex_type &SingletonServer::sing_mtx() {
  static SingletonServer::mutex_type mtx;
  return mtx;
}

// structors 

SingletonServer::SingletonServer() {
  s_instance = this;
}

SingletonServer::~SingletonServer() {
  s_instance = 0x0;
}

// Modifiers :

SingletonDummy *SingletonServer::attach(std::string const &name,
                                        sdummy_factory const &factory) {
  single_map::iterator i = m_singletons.find(name);

  if( m_singletons.end()==i ) {
    // std::cerr<<name<<" = new Ty()\n";
    i = m_singletons.insert(single_map::value_type(name,
						   factory.create())).first;
  }
  i->second->incr_ref();
  return i->second;
}

bool SingletonServer::detach(std::string const &name) {
  single_map::iterator i = m_singletons.find(name);

  if( m_singletons.end()!=i ) {
    SingletonDummy *dummy = i->second;
    bool del = dummy->decr_ref();
    
    if( del ) {
      m_singletons.erase(i);

      // std::cerr<<name<<" : delete Ty\n";
      delete dummy;
      return m_singletons.empty();
    }
  }
  return false;
}
