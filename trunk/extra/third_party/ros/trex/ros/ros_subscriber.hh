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

# include <boost/bind.hpp>
# include "bits/ros_timeline.hh"

namespace TREX {
  namespace ROS {
    
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
    template<typename Message>
    class ros_subscriber :public details::ros_timeline {
    public:
      typedef typename Message::ConstPtr message_ptr;
      
      
      ros_subscriber(details::ros_timeline::xml_arg arg):details::ros_timeline(arg, false) {
        boost::property_tree::ptree::value_type &xml(details::ros_timeline::xml_factory::node(arg));
        
        m_service = TREX::utils::parse_attr<TREX::utils::Symbol>(xml, "ros_service");
	try {

	  // subscibr to the service with a queue of 10 ... may change this to be configurable in the future
	  m_sub = node().subscribe(m_service.str(), 10,
				   &ros_subscriber<Message>::message, this);
	} catch(::ros::Exception const &e) {
	  std::ostringstream oss;
	  oss<<"Exception while trying to subscribe to \""<<m_service<<"\": "<<e.what();
	  throw ros_exception(oss.str());
	}
	if( !m_sub ) {
	  std::ostringstream oss;
	  oss<<"Subscription ot publisher \""<<m_service<<"\" failed.";
	  throw ros_exception(oss.str());
	}
      }
      virtual ~ros_subscriber() {}
      
      void message(message_ptr const &msg) {
        // To be implemented for each instance : default just log a warning
        syslog(TREX::utils::log::warn)<<"This ros subscriber handler is not implemented";
      }
      
    private:
      TREX::utils::Symbol m_service;
      ::ros::Subscriber m_sub;
    }; // TREX::ROS::ros_subscriber<>
    
  } // TREX::ROS
} // TREX

#endif // H_trex_ros_ros_subscriber
