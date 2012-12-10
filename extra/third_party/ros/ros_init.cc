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
#include "ros_reactor.hh"
#include "ros_client.hh"

#include <trex/utils/Plugin.hh>
#include <trex/utils/LogManager.hh>

#include <geometry_msgs/Point.h>

using namespace TREX::utils;
using namespace TREX::transaction;
using namespace TREX::ROS;

/*
 * example for position update based on Point
 */ 

template<>
void TREX::ROS::ros_subscriber<geometry_msgs::Point>::message(geometry_msgs::Point::ConstPtr const &msg) {
  TREX::transaction::Observation obs(name(), "Hold");
  obs.restrictAttribute("x", TREX::transaction::FloatDomain(msg->x));
  obs.restrictAttribute("y", TREX::transaction::FloatDomain(msg->y));
  obs.restrictAttribute("z", TREX::transaction::FloatDomain(msg->z));
  notify(obs);
}

namespace {
  SingletonUse<LogManager> s_log;
  SingletonUse<ros_client> s_ros; // initate connection to ros

  TeleoReactor::xml_factory::declare<ros_reactor> decl("ROSReactor");
  // declare the Point subscriber
  details::ros_timeline::xml_factory::declare< ros_subscriber<geometry_msgs::Point> > pt_decl("Point");
}


namespace TREX {

  void initPlugin() {
    s_log->syslog("ros.plugin", log::info)<<"ROS initialized";
  }

}
