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
#include "ros_client.hh"

using namespace TREX::ROS;

/*
 * class TREX::ROS::ros_client
 */

// structors

ros_client::ros_client():m_active(false), m_freq(m_log->service()) {
  int argc = 0;
  char **argv = NULL;
  m_log->syslog("ros", TREX::utils::log::info)<<"Initialize ros connection";
  ros::init(argc, argv, "trex2", ros::init_options::AnonymousName);
  // Create an extra thread for when the service will be started
  m_log->thread_count(m_log->thread_count()+1, true);
}

ros_client::~ros_client() {
  if(ros::ok() ) {
    m_log->syslog("ros", TREX::utils::log::info)<<"Shutting down ros connection";
    stop();
  }
}

// accessors

bool ros_client::ok() const {
  return ::ros::ok();
}

bool ros_client::started() const {
  TREX::utils::SharedVar<bool>::scoped_lock l(m_active);
  return *m_active;
}

::ros::NodeHandle ros_client::handle() const {
  ros::NodeHandle handle;
  return handle;
}

// modifiers

void ros_client::start() {
  bool should_start = false;
  {
    TREX::utils::SharedVar<bool>::scoped_lock l(m_active);
    if( !*m_active && ::ros::ok() ) {
      should_start = true;
      *m_active = true;
    }
  }
  if( should_start )
    m_log->service().post(boost::bind(&ros_client::spin_cb, this));
}

void ros_client::spin_cb() {
  if( started() ) {
    if( ::ros::ok() ) {
      // Set my timer
      m_freq.expire_from_now(boost::posix_time::milliseconds(100)); // 10Hz is more than enough
      // manage things from ros
      ::ros::spinOnce();
      
      m_freq.async_wait(boost::bind(&ros_client::spin_cb, this));
    } else
      stop();
  }
}




