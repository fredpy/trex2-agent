/* -*- C++ -*- */
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
#include <trex/wt/server.hh>

using namespace TREX::wt;

/*
 * class TREX::Wt::server
 */

// structors

server::server() {}

server::~server() {
  m_server.reset();
}

// observer

bool server::is_inited() const {
  return NULL!=m_server.get();
}

bool server::is_running() const {
  return is_inited() && m_server->isRunning();
}

::Wt::WServer &server::impl() const {
  return *m_server;
}

// modifiers

void server::init(size_t argc, char *argv[], boost::optional<std::string> const &cfg) {
  if( is_inited() )
    throw std::runtime_error("server already inited");
  if( cfg )
    m_server.reset(new ::Wt::WServer("", *cfg));
  else
    m_server.reset(new ::Wt::WServer(""));
  m_server->setServerConfiguration(argc, argv);
}

bool server::start() {
  if( is_inited() ) {
    if( m_server->isRunning() )
      return m_server->start();
  }
  return false;
}
