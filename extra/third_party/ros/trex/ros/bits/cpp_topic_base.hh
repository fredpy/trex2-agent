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
#ifndef H_trex_ros_bits_cpp_topic_base
# define H_trex_ros_bits_cpp_topic_base

#include "ros_timeline.hh"

namespace TREX {
  namespace ROS {
    namespace details {
      
      
      class cpp_topic_base :public ros_timeline {
      public:
        virtual ~cpp_topic_base();
        
        std::string const &topic() const {
          return m_topic;
        }
        utils::Symbol const &predicate() const {
          return m_pred;
        }
        
      protected:
        cpp_topic_base(xml_arg const &arg, bool can_control,
                       std::string type_name);
        transaction::Observation new_obs() const;
        transaction::Observation new_undefined() const;
        
        boost::function<void ()> wrap(boost::function<void (cpp_topic_base *)> fn);
        
        void do_notify(transaction::Observation const &obs);

        virtual void init_publisher() {}
        virtual void process_pending(transaction::TICK date) {}
        virtual void init_subscriber(ros::Subscriber &sub) = 0;
        virtual void add_goal(transaction::goal_id g) {}
        virtual void remove_goal(transaction::goal_id g) {}
        
        std::optional<transaction::Observation> const &last_obs() const {
          return m_last_obs;
        }
        transaction::TICK obs_since() const {
          return m_obs_since;
        }
                
      private:
        std::string const m_topic;
        utils::Symbol m_pred;
        bool const m_merge;
        bool m_extend;
        
        ros::Subscriber m_sub;
        
        void handle_init();
        void synchronize(transaction::TICK date);
        bool handle_request(transaction::goal_id g);
        void handle_recall(transaction::goal_id g);
        
        std::optional<transaction::Observation> m_last_obs;
        transaction::TICK                         m_obs_since;
        
        static void async_exec(std::weak_ptr<ros_timeline> me,
                               boost::function<void (cpp_topic_base *)> fn);
        
      }; // TREX::ROS::details::cpp_topic_base
      
      
      
    }
  }
}

#endif // H_trex_ros_bits_cpp_topic_base