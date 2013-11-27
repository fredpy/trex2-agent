#include <trex/ros/ros_subscriber.hh>
#include <trex/domain/FloatDomain.hh>

#include <turtlesim/Pose.h>

namespace TREX {
  namespace ROS {

    /*
     * Example of a 
     */
    template<>
    void ros_subscriber<turtlesim::Pose>::message(turtlesim::Pose::ConstPtr const &msg) {
      TREX::transaction::Observation obs = new_obs("Hold");
      obs.restrictAttribute("x", 
			    TREX::transaction::FloatDomain(msg->x));
      obs.restrictAttribute("y", 
			    TREX::transaction::FloatDomain(msg->y));
      obs.restrictAttribute("theta", 
			    TREX::transaction::FloatDomain(msg->theta));
      obs.restrictAttribute("linear_velocity", 
			    TREX::transaction::FloatDomain(msg->linear_velocity));
      obs.restrictAttribute("angular_velocity", 
			    TREX::transaction::FloatDomain(msg->angular_velocity));
      notify(obs);    
    }

  }
}

using namespace TREX::ROS;
namespace utils=TREX::utils;

namespace {
  ros_factory::declare< ros_subscriber<turtlesim::Pose> > turtle_pose("Pose");

}
