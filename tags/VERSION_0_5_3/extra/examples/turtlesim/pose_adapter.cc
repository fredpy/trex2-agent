#include <trex/ros/ros_subscriber.hh>
#include <trex/domain/FloatDomain.hh>

#include <turtlesim/Pose.h>

namespace TREX {
  namespace ROS {

    template<>
    struct ros_convert_traits<turtlesim::Pose> {
      typedef turtlesim::Pose         message;
      typedef message::ConstPtr  message_ptr;
      
      enum {
	accept_goals = false
      };

      static transaction::observation_id ros_to_trex(utils::Symbol const &timeline,
						     message_ptr const &msg);
    };

    transaction::observation_id ros_convert_traits<turtlesim::Pose>::ros_to_trex(utils::Symbol const &timeline,
										 ros_convert_traits<turtlesim::Pose>::message_ptr const &msg) {
      transaction::observation_id obs = MAKE_SHARED<transaction::Observation>(timeline, utils::Symbol("Hold"));
      
      obs->restrictAttribute("x", transaction::FloatDomain(msg->x));
      obs->restrictAttribute("y", transaction::FloatDomain(msg->y));
      obs->restrictAttribute("theta", transaction::FloatDomain(msg->theta));
      obs->restrictAttribute("linear_velocity", transaction::FloatDomain(msg->linear_velocity));
      obs->restrictAttribute("angular_velocity", transaction::FloatDomain(msg->angular_velocity));
      return obs;
    } 

  }
}

using namespace TREX::ROS;
namespace utils=TREX::utils;

namespace {

  ros_factory::declare< ros_subscriber<turtlesim::Pose> > turtle_pose("TurltePose");

}
