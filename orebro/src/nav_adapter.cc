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
      typedef go_turtle::actions message;
      typedef message::ConstPtr  message_ptr;

      enum {
	accept_goals = true
      };
      
      
 

      static transaction::observation_id ros_to_trex(utils::Symbol const &timeline,
						     message_ptr const &msg);

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


  // Declare go_turtle/actions ROS message handler as GoTurtleAction xml tag
  ros_factory::declare< ros_subscriber<go_turtle::actions> > go_decl("GoTurtleAction");

  // Declare std_msgs/String ROS message handler as StringPred xml tag
  ros_factory::declare< ros_subscriber<std_msgs::String> > str_decl("StringPred");

}

observation_id 
ros_convert_traits<go_turtle::actions>::ros_to_trex
(Symbol const &timeline,
 ros_convert_traits<go_turtle::actions>::message_ptr const &msg) {
  observation_id 
    obs = MAKE_SHARED<Observation>(timeline,
				   Symbol("Hold"));
  
  obs->restrictAttribute("x", FloatDomain(msg->x));
  obs->restrictAttribute("y", FloatDomain(msg->y));
  
  obs->restrictAttribute("timeout", 
			 FloatDomain(msg->timeout));
  
  int action = msg->action;
  obs->restrictAttribute("action", 
			     IntegerDomain(action));
  
  return obs;
}

ros_convert_traits<go_turtle::actions>::message_ptr 
ros_convert_traits<go_turtle::actions>::trex_to_ros
(transaction::goal_id g) {
  go_turtle::actions::Ptr msg;

  if( g->predicate()=="Hold" ) {
    msg.reset(new go_turtle::actions);
    
    if( g->hasAttribute("action") ) {
      IntegerDomain act = g->getDomain<IntegerDomain>("action");

      if( act.isSingleton() ) {
	int act_val = act.lowerBound().value();
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
	      
	  if( g->hasAttribute("x") )
	    msg->x = g->getDomain<transaction::FloatDomain>("x").closestTo(0.0);
	  else 
	    msg->x = 0.0;
	  if( g->hasAttribute("y") )
	    msg->y = g->getDomain<transaction::FloatDomain>("y").closestTo(0.0);
	  else 
	    msg->y = 0.0;
	  
	} else if( act_val!=3 ) {
	  s_log->syslog("go_turtle", log::warn)<<"Rejected goal ["
					       <<g<<"]: action "<<act_val
					       <<" is undefined";
	  msg.reset();
	}
	// Handle the timeout
	if( msg && g->hasAttribute("timeout")) {
	  FloatDomain::bound t_out = g->getDomain<FloatDomain>("timeout").upperBound(); 
	  
	  if( t_out.isInfinity() ) {
	    // Set it to a very large value 5 hours should be enough 
	    msg->timeout = 18000;
	  } else {
	    msg->timeout = t_out.value();
	  }  
	}
      } else {
	s_log->syslog("go_turtle", log::warn)<<"Rejected goal ["
					     <<g<<"]: action is not set";
	msg.reset();
      }
    } else {
      s_log->syslog("go_turtle", log::warn)<<"Rejected goal ["
					   <<g<<"]: action is not set";
      msg.reset();
    }
  }
  
  return msg;
}
