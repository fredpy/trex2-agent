/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, Frederic Py.
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
#include "roscpp_inject.hh"

using namespace TREX::ROS;
using namespace TREX::utils;

// structors

roscpp_initializer::roscpp_initializer() {
//  start();
}

roscpp_initializer::~roscpp_initializer() {
  if( ok() ) {
    stop();
  }
}

// observers

bool roscpp_initializer::ok() const {
  return ::ros::ok();
}

bool roscpp_initializer::active() const {
  return ok() && ::ros::isInitialized() && !::ros::isShuttingDown();
}


::ros::NodeHandle const &roscpp_initializer::handle() const {
  return *m_handle;
}

// Manipulators

void roscpp_initializer::start() {
  
  if( !::ros::isInitialized() ) {
    int argc = 0;
    char **argv = NULL;
  
    m_log->syslog("ros", log::info)<<"Initialize ROS connection as trex2";
    ::ros::init(argc, argv, "trex2", ::ros::init_options::AnonymousName);
    m_handle.reset(new ::ros::NodeHandle);
  }
}

void roscpp_initializer::stop() {
  if( ::ros::isInitialized() && !::ros::isShuttingDown() ) {
    m_log->syslog("ros", log::info)<<"Shutting down ROS connection";
    m_handle.reset();
  }
}



