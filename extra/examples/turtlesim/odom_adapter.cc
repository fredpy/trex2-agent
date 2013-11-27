#include <trex/ros/ros_subscriber.hh>
#include <trex/ros/ros_action.hh>
#include <trex/domain/FloatDomain.hh>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>

#include <turtlebot_actions/TurtlebotMoveAction.h>

using namespace TREX::transaction;

namespace TREX {
  namespace ROS {

    /*
     * Example of a simple subscriber based on ros odometry sensor
     * It subscriber to messages of type Odometry and convert them 
     * into a trex observation 
     */
    template<>
    void ros_subscriber<nav_msgs::Odometry>::message(nav_msgs::Odometry::ConstPtr const &msg) {
      Observation obs = new_obs("Hold");
      obs.restrictAttribute("x", 
			    FloatDomain(msg->pose.pose.position.x));
      obs.restrictAttribute("y", 
			    FloatDomain(msg->pose.pose.position.y));
      obs.restrictAttribute("z", 
			    FloatDomain(msg->pose.pose.position.z));
      // More to be added ... but I neeed to go through python instead for
      // introspection 
      notify(obs);    
    }

    template<>
    void ros_subscriber<geometry_msgs::Twist>::message(geometry_msgs::Twist::ConstPtr const &msg) {
      Observation obs = new_obs("Hold");
      obs.restrictAttribute("linear_x", FloatDomain(msg->linear.x)); 
      obs.restrictAttribute("linear_y", FloatDomain(msg->linear.y)); 
      obs.restrictAttribute("linear_z", FloatDomain(msg->linear.z)); 
      obs.restrictAttribute("angular_x", FloatDomain(msg->angular.x)); 
      obs.restrictAttribute("angular_y", FloatDomain(msg->angular.y)); 
      obs.restrictAttribute("angular_z", FloatDomain(msg->angular.z));
      notify(obs);
    }

    template<>
    bool ros_subscriber<geometry_msgs::Twist>::handle_request(goal_id g) {
      geometry_msgs::Twist msg;
      if( g->hasAttribute("linear_x") )
	msg.linear.x = g->getDomain<FloatDomain>("linear_x").closestTo(0.0);
      else 
	msg.linear.x = 0.0;
      if( g->hasAttribute("linear_y") )
	msg.linear.y = g->getDomain<FloatDomain>("linear_y").closestTo(0.0);
      else 
	msg.linear.y = 0.0;
      if( g->hasAttribute("linear_z") )
	msg.linear.z = g->getDomain<FloatDomain>("linear_z").closestTo(0.0);
      else 
	msg.linear.z = 0.0;
      if( g->hasAttribute("angular_x") )
	msg.angular.x = g->getDomain<FloatDomain>("angular_x").closestTo(0.0);
      else 
	msg.angular.x = 0.0;
      if( g->hasAttribute("angular_y") )
	msg.angular.y = g->getDomain<FloatDomain>("angular_y").closestTo(0.0);
      else 
	msg.angular.y = 0.0;
      if( g->hasAttribute("angular_z") )
	msg.angular.z = g->getDomain<FloatDomain>("angular_z").closestTo(0.0);
      else 
	msg.angular.z = 0.0;
      dispatch(msg);
    }
    

    template<>
    bool ros_action<turtlebot_actions::TurtlebotMoveAction>::build_goal(goal_id g,
									goal_type &ros_g) {
      static TREX::utils::Symbol const turn_s("turn_distance");
      static TREX::utils::Symbol const fwd_s("forward_distance");
	
      // Get domain and possibly fix its value if required
      if( g->hasAttribute(turn_s) ) {
	FloatDomain tmp = g->getDomain<FloatDomain>(turn_s);
	ros_g.turn_distance = tmp.closestTo(0.0);
      } else 
	ros_g.turn_distance = 0.0;
      g->restrictAttribute(Variable(turn_s, 
				    FloatDomain(ros_g.turn_distance)));
      
      if( g->hasAttribute(fwd_s) ) {
	FloatDomain tmp = g->getDomain<FloatDomain>(fwd_s);
	ros_g.forward_distance = tmp.closestTo(0.0);
      } else 
	ros_g.forward_distance = 0.0;
      g->restrictAttribute(Variable(fwd_s, 
				    FloatDomain(ros_g.forward_distance)));
      return true; // Indicate that the goal can be sent
    }

    template<>
    void ros_action<turtlebot_actions::TurtlebotMoveAction>::handle_recall
    (TREX::transaction::goal_id g) {
      if( g->predicate()=="Active" ) {      
      }
    }

  }
}

using namespace TREX::ROS;
namespace utils=TREX::utils;

namespace {
  ros_factory::declare< ros_subscriber<nav_msgs::Odometry> > odo("Odometry");
  ros_factory::declare< ros_subscriber<geometry_msgs::Twist> > twist("Twist");

  ros_factory::declare< ros_action<turtlebot_actions::TurtlebotMoveAction> >
  turtle_move("TurtlebotMoveAction");
}
