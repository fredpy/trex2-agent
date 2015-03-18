#include <trex/ros/ros_subscriber.hh>
#include <trex/domain/FloatDomain.hh>

#include <geometry_msgs/PoseWithCovarianceStamped.h>

namespace TREX {
  namespace ROS {

    template<>
    struct ros_convert_traits<geometry_msgs::PoseWithCovarianceStamped> {
      typedef geometry_msgs::PoseWithCovarianceStamped message;
      typedef message::ConstPtr                        message_ptr;
      
      enum {
	accept_goals = false
      }; 
      
      static transaction::observation_id ros_to_trex(utils::Symbol const &timeline,
						     message_ptr const &msg);
      
    }; // ros_convert_traits<geometry_msgs::PoseWithCovarianceStamped>
  }
}

using namespace TREX::ROS;
using namespace TREX::transaction;
using TREX::utils::Symbol;

observation_id ros_convert_traits<geometry_msgs::PoseWithCovarianceStamped>::ros_to_trex
(Symbol const &timeline, ros_convert_traits<geometry_msgs::PoseWithCovarianceStamped>::message_ptr const &msg) {
  // I just care about the pose
  geometry_msgs::Pose const &pose = msg->pose.pose;
  
  observation_id obs = MAKE_SHARED<Observation>(timeline, "Hold");
  
  // Same as odometry messages
  obs->restrictAttribute("position_x", transaction::FloatDomain(pose.position.x));
  obs->restrictAttribute("position_y", transaction::FloatDomain(pose.position.y));
  obs->restrictAttribute("position_z", transaction::FloatDomain(pose.position.z));
  
  obs->restrictAttribute("orientation_x", transaction::FloatDomain(pose.orientation.x));
  obs->restrictAttribute("orientation_y", transaction::FloatDomain(pose.orientation.y));
  obs->restrictAttribute("orientation_z", transaction::FloatDomain(pose.orientation.z));
  obs->restrictAttribute("orientation_w", transaction::FloatDomain(pose.orientation.w));
  
  return obs; 
}

namespace {
  ros_factory::declare< ros_subscriber<geometry_msgs::PoseWithCovarianceStamped> > odo("PoseWithCovarianceStamped");
}



