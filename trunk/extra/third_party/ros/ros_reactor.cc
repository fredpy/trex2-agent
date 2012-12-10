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
 * class TREX::ROS::ros_reactor
 */

ros_reactor::ros_reactor(TeleoReactor::xml_arg_type arg)
  :TeleoReactor(arg, false) {
  // Need to parse the timelines
  boost::property_tree::ptree::value_type &node(xml_factory::node(arg));
  // embeds external configuration
  ext_xml(node.second, "config");
  add_timelines(node.second.begin(), node.second.end());
 
}
    
ros_reactor::~ros_reactor() {
}

void ros_reactor::handleInit() { 
  m_cli->start();
}

bool ros_reactor::synchronize() {
  // fail synchronization when unable to connect to ros
  return m_cli->ok();
}
      
void ros_reactor::handleRequest(goal_id const &g) {
  // Look for the timeline
  tl_map::const_iterator i = m_tl_conn.find(g->object());
  if( m_tl_conn.end()!=i ) {
    if( i->second->request(g) ) {
      syslog("ros", TREX::utils::log::info)<<"Sent goal "<<g;
      m_goals.insert(g);
    }
  }
}

void ros_reactor::handleRecall(goal_id const &g) {
  std::set<goal_id>::iterator pos = m_goals.find(g);

  if( m_goals.end()!=pos ) {
    m_goals.erase(pos);
    tl_map::const_iterator i = m_tl_conn.find(g->object());
    if( m_tl_conn.end()!=i ) {
      i->second->recall(g);
    }
  }
}
