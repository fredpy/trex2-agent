#include <trex/ros/ros_subscriber.hh>

#include <trex/domain/FloatDomain.hh>

#include <std_msgs/String.h>
#include <go_turtle/actions.h>


namespace TREX {
  namespace ROS {

    
    /*
     * class used as an helper to convert trex observation & goals to ROS 
     * messages of type go_turtle::actions 
     */
    template<>
    struct ros_convert_traits<go_turtle::actions> {
      // The type of ROS messages handled by this traits 
      typedef go_turtle::actions message;
      // The type of const pointer to a ROS message (always the same)
      typedef message::ConstPtr  message_ptr;

      /** indicate that this class support the conversion 
       * of T-REX goals into writing of message on the topic
       */
      enum {
	accept_goals = true
      };
      
     
      /** @brief convert a ROS message into a T-REX observation
       * @param timeline Name of the timeline that will receive the observation
       * @param msg The message received from ROS
       *
       * Convert @p msg inot a T-REX observation on the timeline @p timeline
       *
       * @retval a smart_pointer to the newly created observation
       * @retval a null pointer if no observation was produced from @p msg
       */
      static transaction::observation_id ros_to_trex(utils::Symbol const &timeline,
						     message_ptr const &msg);

      /** @brief T-REX goal into a ROS message
       * @param g The goal recevied 
       *
       * Convert the goal @p g into  a ROS message to be written on the 
       * topic handled 
       * @retval a smart_pointer to the newly created mesasage from @p g
       * @retval a null pointer if @p g do not convert into a ROS message
       *         to be sent (often due to the fact that @p g is 
       *         invalid/rejected)
       *
       * @note this method needs to be defined in a ros_conversion_traits 
       * only if the enum accept_goals is not false
       */
      static message_ptr trex_to_ros(transaction::goal_id g);
     
    };

    // A simple handler for ROS string messages
    template<>
    struct ros_convert_traits<std_msgs::String> {
      typedef std_msgs::String message;
      typedef message::ConstPtr  message_ptr;

      enum {
	accept_goals = false
      };
      
      // Just convert the string into a predicate
      static transaction::observation_id ros_to_trex(utils::Symbol const &timeline,
						     message_ptr const &msg) {
	return MAKE_SHARED<transaction::Observation>(timeline, utils::Symbol(msg->data));
      }
    };

    
  }
}

using namespace TREX::ROS;
using namespace TREX::transaction;
using namespace TREX::utils;

namespace {

  SingletonUse<LogManager> s_log;


  // Declare go_turtle/actions ROS message handler as <GoTurtleAction> xml tag
  // 
  // the real type of this ros_subscriber is in fact 
  // ros_subscriber<go_turtle::actions, ros_convert_traits<go_turtle::actions> >
  // but ros_convert_traits<go_turtle::actions> is implicit 
  ros_factory::declare< ros_subscriber<go_turtle::actions> > 
    go_decl("GoTurtleAction");

  // Declare std_msgs/String ROS message handler as StringPred xml tag
  // 
  // the real type of this ros_subscriber is in fact 
  // ros_subscriber<std_msgs::String, ros_convert_traits<std_msgs::String> >
  // but ros_convert_traits<std_msgs::String> is implicit 
  ros_factory::declare< ros_subscriber<std_msgs::String> > 
    str_decl("StringPred");

}

observation_id 
ros_convert_traits<go_turtle::actions>::ros_to_trex
(Symbol const &timeline,
 ros_convert_traits<go_turtle::actions>::message_ptr const &msg) {
  // Create a timeline.Hold observation
  observation_id 
    obs = MAKE_SHARED<Observation>(timeline,
				   Symbol("Hold"));
  // Set timeline.Hold.x to msg->x as a float
  obs->restrictAttribute("x", FloatDomain(msg->x));
  // Set timeline.Hold.y to msg->y as a float
  obs->restrictAttribute("y", FloatDomain(msg->y));
  
  // Set timeline.Hold.timeout to msg->timeout as a float
  obs->restrictAttribute("timeout", 
			 FloatDomain(msg->timeout));
  
  int action = msg->action;
  // Set timeline.Hold.action to msg->action as an integer
  obs->restrictAttribute("action", 
			     IntegerDomain(action));
  
  return obs;
}

ros_convert_traits<go_turtle::actions>::message_ptr 
ros_convert_traits<go_turtle::actions>::trex_to_ros
(transaction::goal_id g) {
  go_turtle::actions::Ptr msg;

  // Check  that the goals is a Hold predicate .. otherwise do not bother with this goal
  if( g->predicate()=="Hold" ) {
    msg.reset(new go_turtle::actions);
    
    // Check that it has an action value
    if( g->hasAttribute("action") ) {
      IntegerDomain act = g->getDomain<IntegerDomain>("action");

      // Check that the action is a singleton value (ie not an interval)
      if( act.isSingleton() ) {
	int act_val = act.lowerBound().value(); // get the value
	msg->action = act_val;
	
	// Handle action :
	//  -2 go and rotate 2 times clockwise
	//  -1 go and rotate 1 time clockwise
	//   0 go  
	//   1 go and rotate 1 time counter-clockwise 
	//   2 go and rotate 2 times counter-clockwise
	//   3 just rotate 3 times (x and y are not relevant)
	// anything else is invalid
	if( abs(act_val)<=2 ) {
	  // for all of these I need x and y 
	  // Default policy is to set them toward 0.0
	      
	  // Pick the x value closest to 0.0 
	  if( g->hasAttribute("x") )
	    msg->x = g->getDomain<transaction::FloatDomain>("x").closestTo(0.0);
	  else 
	    msg->x = 0.0;
	  // Pick the y value closest to 0.0
	  if( g->hasAttribute("y") )
	    msg->y = g->getDomain<transaction::FloatDomain>("y").closestTo(0.0);
	  else 
	    msg->y = 0.0;
	  
	} else if( act_val!=3 ) {
	  // If the value is not 3 here that it is an invalid action
	  // If it is 3 then we are set as x,y are not needed in that case
	  s_log->syslog("go_turtle", log::warn)<<"Rejected goal ["
					       <<g<<"]: action "<<act_val
					       <<" is undefined";
	  msg.reset();
	}
	// Check the timeout in seconds
	if( msg && g->hasAttribute("timeout")) {
	  // Get the largest possibel timeout
	  FloatDomain::bound t_out = g->getDomain<FloatDomain>("timeout").upperBound(); 
	  
	  // If this value is +inf then set it to a very large value
	  if( t_out.isInfinity() ) {
	    // Set it to a very large value 5 hours should be enough 
	    msg->timeout = 18000;
	  } else {
	    // other wise just set it to the value given
	    msg->timeout = t_out.value();
	  }  
	}
      } else {
	// If action has multiple possibel value I caanot know what to do => reject
	s_log->syslog("go_turtle", log::warn)<<"Rejected goal ["
					     <<g<<"]: action is not set";
	msg.reset();
      }
    } else {
      // If action attribute do not exist I know event less what value to pick => reject
      s_log->syslog("go_turtle", log::warn)<<"Rejected goal ["
					   <<g<<"]: action is not set";
      msg.reset();
    }
  }
  
  return msg;
}
