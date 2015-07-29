/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, Frederic Py.
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
#ifndef H_trex_ros_cpp_topic
# define H_trex_ros_cpp_topic

# include "roscpp_inject.hh"
# include "bits/ros_timeline.hh"
# include "msg_cvt_traits.hh"
# include "ros_error.hh"

# include <boost/static_assert.hpp>

# include <ros/ros.h>

namespace TREX {
  namespace ROS {
    
    
    template<typename Message, bool Goals,
             class Convert = msg_cvt_traits<Message, Goals> >
    class cpp_topic :public details::ros_timeline {
      BOOST_STATIC_ASSERT(::ros::message_traits::IsMessage<Message>::value);
      typedef Convert translator;
    public:
      typedef typename translator::message     message_type;
      typedef typename translator::message_ptr message_ptr;
      
      
      using details::ros_timeline::xml_factory;
      using details::ros_timeline::xml_arg;

      cpp_topic(xml_arg arg);
      ~cpp_topic();
      
      std::string const &topic() const {
        return m_topic;
      }
      
    private:
      void message(message_ptr msg);
      
      
      bool handle_request(transaction::goal_id g) {}
      void handle_recall(transaction::goal_id g) {}
      void synchronize(transaction::TICK date);

      transaction::observation_id m_last_obs;
      bool const m_merge;
      bool m_extend;
      
      std::string const m_topic;
      std::string m_pred;
      ::ros::NodeHandle m_handle;
      
      
      ::ros::Subscriber m_sub;
      
      
    }; // TREX::ROS::cpp_topic<>
    
# define In_H_trex_ros_cpp_topic
#  include "bits/cpp_topic.tcc"
# undef In_H_trex_ros_cpp_topic
  } // TREX::ROS
} // TREX

#endif // H_trex_ros_cpp_topic
