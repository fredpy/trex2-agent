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
#include "ros_action.hh"

#include <trex/utils/Plugin.hh>
#include <trex/utils/LogManager.hh>

#include <trex/domain/FloatDomain.hh>

#include <geometry_msgs/Point.h>

namespace TREX {
  namespace ROS {

    template<>
    struct ros_convert_traits<geometry_msgs::Point> {
      typedef geometry_msgs::Point    message;
      typedef message::ConstPtr  message_ptr;
      enum {
	accept_goals = true
      };
      
      static transaction::observation_id ros_to_trex(utils::Symbol const &timeline,
						     message_ptr const &msg);

      static message_ptr trex_to_ros(transaction::goal_id g);
    };

    transaction::observation_id ros_convert_traits<geometry_msgs::Point>::ros_to_trex(utils::Symbol const &timeline,
										      ros_convert_traits<geometry_msgs::Point>::message_ptr const &msg) {
      transaction::observation_id obs = MAKE_SHARED<transaction::Observation>(timeline, utils::Symbol("Hold"));
      
      obs->restrictAttribute("x", transaction::FloatDomain(msg->x));
      obs->restrictAttribute("y", transaction::FloatDomain(msg->y));
      obs->restrictAttribute("z", transaction::FloatDomain(msg->z));
      return obs;
    } 

    
    ros_convert_traits<geometry_msgs::Point>::message_ptr ros_convert_traits<geometry_msgs::Point>::trex_to_ros(transaction::goal_id g) {
      geometry_msgs::Point::Ptr msg;

      if( g->predicate()=="Hold" ) { 
	msg.reset(new geometry_msgs::Point);
	if( g->hasAttribute("x") )
	  msg->x = g->getDomain<transaction::FloatDomain>("x").closestTo(0.0);
	else 
	  msg->x = 0.0;
	if( g->hasAttribute("y") )
	  msg->y = g->getDomain<transaction::FloatDomain>("y").closestTo(0.0);
	else 
	  msg->y = 0.0;
	if( g->hasAttribute("z") )
	  msg->z = g->getDomain<transaction::FloatDomain>("z").closestTo(0.0);
	else 
	  msg->z = 0.0;	
      }
      return msg;
    }

  }
}

using namespace TREX::utils;
using namespace TREX::transaction;
using namespace TREX::ROS;

namespace {
  SingletonUse<LogManager> s_log;
  SingletonUse<ros_client> s_ros; // initate connection to ros

  TeleoReactor::xml_factory::declare<ros_reactor> decl("ROSReactor");
  // declare the Point subscriber
  ros_factory::declare< ros_subscriber<geometry_msgs::Point> > pt_decl("Point");
}


namespace TREX {

  void initPlugin() {
    s_log->syslog("ros.plugin", log::info)<<"ROS initialized";
  }

}
