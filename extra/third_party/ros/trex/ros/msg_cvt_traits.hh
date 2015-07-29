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
#ifndef H_trex_ros_msg_cvt_traits
# define H_trex_ros_msg_cvt_traits

# include <trex/transaction/Predicate.hh>
# include <ros/message_traits.h>

namespace TREX {
  namespace ROS {
    
    namespace details {
      
      template<typename Message>
      struct cvt_base {
        typedef Message                    message;
        typedef typename message::ConstPtr message_ptr;
        
        typedef ros::message_traits::DataType<message> data_type;
        
        static char const *msg_type() {
          return data_type::value();
        }
      };
      
    }
    
    
    template<typename Message, bool Goals>
    struct msg_cvt_traits: public details::cvt_base<Message> {
      using typename details::cvt_base<Message>::message;
      using typename details::cvt_base<Message>::message_ptr;
      
      enum {
        accept_goals = Goals
      };
      
      static void to_trex(message_ptr const &msg,
                          transaction::Predicate &pred);
      static message_ptr to_ros(transaction::Predicate const &pred);
      
    };
    
    template<typename Message>
    struct msg_cvt_traits<Message, false>: public details::cvt_base<Message> {
      using typename details::cvt_base<Message>::message;
      using typename details::cvt_base<Message>::message_ptr;

      enum {
        accept_goals = false
      };
      
      static void to_trex(message_ptr const &msg,
                          transaction::Predicate &pred);
    };
        
  }
}

#endif // H_trex_ros_msg_cvt_traits