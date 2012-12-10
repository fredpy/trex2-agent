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
#ifndef H_trex_ros_ros_action
# define H_trex_ros_ros_action

# include <actionlib/client/simple_action_client.h>
# include <actionlib/client/simple_goal_state.h>
# include <actionlib/client/simple_client_goal_state.h>

# include "bits/ros_timeline.hh"

namespace TREX {
  namespace ROS {
    
    template<typename Action>
    class ros_action :public details::ros_timeline {
    public:
      typedef typename Action::_action_goal_type::_goal_type         goal_type;

      typedef typename Action::_action_feedback_type::_feedback_type feedback_type;
      typedef typename feedback_type::ConstPtr                       feedback_ptr;

      typedef typename Action::_action_result_type::_result_type     result_type;
      typedef typename result_type::ConstPtr                         result_ptr;

      ros_action(details::ros_timeline::xml_arg arg):details::ros_timeline(arg, true) {
        boost::property_tree::ptree::value_type &xml(details::ros_timeline::xml_factory::node(arg));

	m_service = TREX::utils::parse_attr<TREX::utils::Symbol>(xml, "ros_service");
	
	try {
	  // create my action client 
	  m_client.reset(new actionlib::SimpleActionClient<Action>(m_service.str()));
	  
	  // wait for server 
	  if( !m_client->waitForServer(::ros::Duration(60.0)) ) {
	    std::ostringstream oss;
	    oss<<"timed out while connecting to action server \""<<m_service<<"\"";
	    throw ros_error(oss.str());
	  }
	} catch(::ros::Exception const &e) {
	  std::ostringstream oss;
	  oss<<"Exception while trying to connect action client to \""<<m_service<<"\": "<<e.what();
	  throw ros_error(oss.str());
	}
	notify(new_obs("Inactive")); // Set it inactive by default
      }

      virtual ~ros_action() {
	m_client.reset();
      }

    protected:
      void handle_result(actionlib::SimpleClientGoalState const &state,
			 result_ptr const &msg);
      void handle_active();
      void handle_feedback(feedback_ptr const &msg);
      

    private:
      TREX::utils::Symbol m_service;
      UNIQ_PTR< actionlib::SimpleActionClient<Action> > m_client;
    }; // TREX::ROS::ros_action 

  } // TREX::ROS
} // TREX

#endif // H_trex_ros_ros_action
