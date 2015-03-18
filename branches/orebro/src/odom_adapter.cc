#include <trex/ros/ros_subscriber.hh>
#include <trex/domain/FloatDomain.hh>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>

using namespace TREX::transaction;

namespace TREX {
  namespace ROS {
    
    template<>
    struct ros_convert_traits<nav_msgs::Odometry> {
      typedef nav_msgs::Odometry message;
      typedef message::ConstPtr  message_ptr;

      enum {
	accept_goals = false
      };

      
      static transaction::observation_id ros_to_trex(utils::Symbol const &timeline,
						     message_ptr const &msg);

    };

    transaction::observation_id ros_convert_traits<nav_msgs::Odometry>::ros_to_trex(utils::Symbol const &timeline,
										    ros_convert_traits<nav_msgs::Odometry>::message_ptr const &msg) {
      transaction::observation_id obs = MAKE_SHARED<transaction::Observation>(timeline, utils::Symbol("Hold"));
      
      obs->restrictAttribute("position_x", transaction::FloatDomain(msg->pose.pose.position.x));
      obs->restrictAttribute("position_y", transaction::FloatDomain(msg->pose.pose.position.y));
      obs->restrictAttribute("position_z", transaction::FloatDomain(msg->pose.pose.position.z));

      obs->restrictAttribute("orientation_x", transaction::FloatDomain(msg->pose.pose.orientation.x));
      obs->restrictAttribute("orientation_y", transaction::FloatDomain(msg->pose.pose.orientation.y));
      obs->restrictAttribute("orientation_z", transaction::FloatDomain(msg->pose.pose.orientation.z));
      obs->restrictAttribute("orientation_w", transaction::FloatDomain(msg->pose.pose.orientation.w));

      return obs;
    } 

    template<>
    struct ros_convert_traits<geometry_msgs::Twist> {
      typedef geometry_msgs::Twist message;
      typedef message::ConstPtr  message_ptr;
      enum {
	accept_goals = true
      };
      
      static transaction::observation_id ros_to_trex(utils::Symbol const &timeline,
						     message_ptr const &msg);

      static message_ptr trex_to_ros(transaction::goal_id g);
    };

    transaction::observation_id ros_convert_traits<geometry_msgs::Twist>::ros_to_trex(utils::Symbol const &timeline,
										      ros_convert_traits<geometry_msgs::Twist>::message_ptr const &msg) {
      transaction::observation_id obs = MAKE_SHARED<transaction::Observation>(timeline, utils::Symbol("Hold"));
      
      obs->restrictAttribute("linear_x", transaction::FloatDomain(msg->linear.x));
      obs->restrictAttribute("linear_y", transaction::FloatDomain(msg->linear.y));
      obs->restrictAttribute("linear_z", transaction::FloatDomain(msg->linear.z));

      obs->restrictAttribute("angular_x", transaction::FloatDomain(msg->angular.x));
      obs->restrictAttribute("angular_y", transaction::FloatDomain(msg->angular.y));
      obs->restrictAttribute("angular_z", transaction::FloatDomain(msg->angular.z));

      return obs;
    } 

    
    ros_convert_traits<geometry_msgs::Twist>::message_ptr ros_convert_traits<geometry_msgs::Twist>::trex_to_ros(transaction::goal_id g) {
      geometry_msgs::Twist::Ptr msg;
      
      if( g->predicate()=="Hold" ) {
	msg.reset(new geometry_msgs::Twist);
	if( g->hasAttribute("linear_x") )
	  msg->linear.x = g->getDomain<FloatDomain>("linear_x").closestTo(0.0);
	else 
	  msg->linear.x = 0.0;
	if( g->hasAttribute("linear_y") )
	  msg->linear.y = g->getDomain<FloatDomain>("linear_y").closestTo(0.0);
	else 
	  msg->linear.y = 0.0;
	if( g->hasAttribute("linear_z") )
	  msg->linear.z = g->getDomain<FloatDomain>("linear_z").closestTo(0.0);
	else 
	  msg->linear.z = 0.0;
	if( g->hasAttribute("angular_x") )
	  msg->angular.x = g->getDomain<FloatDomain>("angular_x").closestTo(0.0);
	else 
	  msg->angular.x = 0.0;
	if( g->hasAttribute("angular_y") )
	  msg->angular.y = g->getDomain<FloatDomain>("angular_y").closestTo(0.0);
	else 
	  msg->angular.y = 0.0;
	if( g->hasAttribute("angular_z") )
	  msg->angular.z = g->getDomain<FloatDomain>("angular_z").closestTo(0.0);
	else 
	  msg->angular.z = 0.0;
      }

      return msg;
    }

  }
}

using namespace TREX::ROS;
namespace utils=TREX::utils;

namespace {
  ros_factory::declare< ros_subscriber<nav_msgs::Odometry> > odo("Odometry");
  ros_factory::declare< ros_subscriber<geometry_msgs::Twist> > twist("Twist");
}
