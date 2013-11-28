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
#ifndef H_trex_ros_ros_subscriber
# define H_trex_ros_ros_subscriber

# include "ros_convert_traits.hh"

# include <boost/bind.hpp>
# include <boost/static_assert.hpp>

# include "bits/ros_timeline.hh"

namespace TREX {
  namespace ROS {
    
    namespace details {

      template<class RosCvt, bool Goals>
      struct ros_goal_helper {
	typedef typename RosCvt::message_ptr message_ptr;
	
	static message_ptr trex_to_ros(transaction::goal_id g) {
	  return RosCvt::trex_to_ros(g);
	}
      };
      
      template<class RosCvt>
      struct ros_goal_helper<RosCvt, false> {
	
	typedef typename RosCvt::message_ptr message_ptr;
	
	static message_ptr trex_to_ros(transaction::goal_id g) {
	  message_ptr ret;
	  return ret;
	}
      };
    }


    /** @brief Observing timeline interface
     *
     * @tparam Message A ROS message type
     *
     * This class acts as an abstract adapter between a ROS subscriber receiving
     * messages of type @p Message and converting them into a timeline.
     *
     * As it is not easy in ROS C++ API to handle this in a generic way we just 
     * provide this class and it is the responsibility of the implementer to 
     * specialize its message handling callback based on the structure of 
     * Message
     *
     * @ingroup ros
     * @author Frederic Py <fpy@mbari.org>
     */
    template<typename Message, class Convert = ros_convert_traits<Message> >
    class ros_subscriber :public details::ros_timeline {
      BOOST_STATIC_ASSERT(::ros::message_traits::IsMessage<Message>::value);
    public:

      typedef typename Message::ConstPtr message_ptr;
      typedef Message message_type;
      typedef Convert converter;
      
      
      ros_subscriber(details::ros_timeline::xml_arg arg)
	:details::ros_timeline(arg, Convert::accept_goals && 
			       TREX::utils::parse_attr(false, details::ros_timeline::xml_factory::node(arg), 
						       "control")) {
        boost::property_tree::ptree::value_type &xml(details::ros_timeline::xml_factory::node(arg));
        
        m_service = TREX::utils::parse_attr<TREX::utils::Symbol>(xml, "ros_service");
	
	if( controlable() ) {
	  // syslog()<<"Creating service "<<m_service<<" to dispatch goals";
	  m_pub = advertise<message_type>(m_service.str());
	}
	try {
	  syslog()<<"Connecting to service "<<m_service;

	  // subscibr to the service with a queue of 10 ... may change this to be configurable in the future
	  m_sub = node().subscribe(m_service.str(), 10,
				   &ros_subscriber<message_type>::message, this);
	} catch(::ros::Exception const &e) {
	  std::ostringstream oss;
	  oss<<"Exception while trying to subscribe to \""<<m_service<<"\": "<<e.what();
	  throw ros_error(oss.str());
	}
	if( !m_sub ) {
	  std::ostringstream oss;
	  oss<<"Subscription ot publisher \""<<m_service<<"\" failed.";
	  throw ros_error(oss.str());
	}
      }
      virtual ~ros_subscriber() {}
      
      void message(message_ptr const &msg) {
	TREX::transaction::observation_id o = converter::ros_to_trex(name(), msg);
	if( o )
	  notify(*o);
      }
      
      void dispatch(Message const &msg) {
	if( controlable() ) {
	  syslog()<<"Publish message to ros topic "<<m_pub.getTopic()
			 <<":\n"<<msg; 
	  m_pub.publish(msg);
	}
      }

      void synchronize() {
	if( !updated() ) {
	  notify(new_obs(TREX::transaction::Predicate::undefined_pred()));
	}
      }
      
      bool handle_request(TREX::transaction::goal_id g) {
	message_ptr msg = details::ros_goal_helper<converter, converter::accept_goals>::trex_to_ros(g);
	if( msg ) {
	  dispatch(*msg);
	  return true;
	}
	return false;
      } 


    private:
      TREX::utils::Symbol m_service;
      ::ros::Subscriber m_sub;
      ::ros::Publisher m_pub;
    }; // TREX::ROS::ros_subscriber<>
    
  } // TREX::ROS
} // TREX

#endif // H_trex_ros_ros_subscriber