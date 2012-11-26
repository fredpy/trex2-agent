/*********************************************************************
 * Software License Agreement (BSD License)
 * 
 *  Copyright (c) 2012, MBARI.
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
 
using namespace TREX::ROS;
using namespace TREX::transaction;
using namespace TREX::utils; 

/*
 * class TREX::ROS::details::ros_timeline
 */

::ros::NodeHandle &::TREX::ROS::details::ros_timeline::node() {
  return m_reactor.m_ros;
}

/*
 * class TREX::ROS::ros_reactor
 */

ros_reactor::ros_reactor(TeleoReactor::xml_arg_type arg)
  :TeleoReactor(arg, false) {
  // Need to parse the timelines
  boost::property_tree::ptree::value_type &node(xml_factory::node(arg));
  // embeds external configuration
  ext_xml(node.second, "config");
 
  for(boost::property_tree::ptree::iterator i=node.second.begin();
      node.second.end()!=i; ++i) {
    if( is_tag(*i, "Observe") ) {
      // Do something to get the data
    } else if( is_tag(*i, "Service") ) {
      // Do something to both allow requests and collect feedback
    } else
      syslog(log::warn)<<"Ignoring XML tag "<<i->first;
  }
}
    
ros_reactor::~ros_reactor() {
}

void ros_reactor::handleInit() { 
}

bool ros_reactor::synchronize() {
  return ::ros::ok();
}
      
void ros_reactor::handleRequest(goal_id const &g) {
}

void ros_reactor::handleRecall(goal_id const &g) {
}
