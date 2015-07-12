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
#ifndef H_trex_ros_bits_ros_timeline
# define H_trex_ros_bits_ros_timeline

# include <ros/ros.h>

# include <trex/utils/platform/cpp11_deleted.hh>
# include <trex/utils/XmlFactory.hh>
# include <trex/utils/log/stream.hh>

# include <trex/transaction/Observation.hh>
# include <trex/transaction/Goal.hh>

# include "../ros_error.hh"

namespace TREX {
  namespace ROS {
    
    class ros_reactor;
    
    namespace details {
      
      class ros_timeline :boost::noncopyable {
      public:
        typedef boost::shared_ptr<ros_timeline> pointer;
        
        typedef TREX::utils::XmlFactory<ros_timeline, pointer, ros_reactor *> xml_factory;
        typedef xml_factory::argument_type                                    xml_arg;
        
        
        virtual ~ros_timeline();
        
        utils::Symbol const &name() const {
          return m_name;
        }
        bool controlable() const {
          return m_controlable;
        }
        
        bool request(TREX::transaction::goal_id g);
        void recall(TREX::transaction::goal_id g);
        void do_init();
        void do_synchronize();
      protected:
        ros_timeline(xml_arg arg, bool control);
        ros_timeline(ros_reactor *r, utils::Symbol const &tl, bool init, bool control);
        
        virtual bool handle_request(TREX::transaction::goal_id g) =0;
        virtual void handle_recall(TREX::transaction::goal_id g) {}
        
        virtual void synchronize(transaction::TICK date) {}
        
        bool updated() const {
          return m_updated;
        }
        
        ::ros::NodeHandle &node();
        template<typename Msg>
        ::ros::Publisher advertise(std::string const &name) {
          ::ros::NodeHandle &n = node();
          return n.advertise<Msg>(name, 10, true);
        }
        
        void notify(transaction::Observation const &obs);
        TREX::utils::log::stream syslog(TREX::utils::Symbol const &kind=TREX::utils::log::null);
        TREX::transaction::Observation new_obs(TREX::utils::Symbol const &pred) const {
          return TREX::transaction::Observation(name(), pred);
        }
        
      private:
        void init_timeline();
        
        ros_reactor &m_reactor;
        utils::Symbol const m_name;
        bool const m_controlable;
        bool const m_init;
        
        bool m_undefined, m_updated;
        
# ifndef DOXYGEN
        ros_timeline() DELETED; // Non default constructible
# endif // DOXYGEN
        
        friend class ros_reactor;
      }; // TREX:ROS::details::ros_timeline
      
    } // TREX::ROS::details
    
    typedef details::ros_timeline::xml_factory ros_factory;
    
  } // TREX::ROS
} // TREX

#endif // H_trex_ros_bits_ros_timeline
