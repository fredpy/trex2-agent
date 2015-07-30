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
#ifndef H_trex_ros_roscpp_inject
# define H_trex_ros_roscpp_inject

# include <trex/utils/LogManager.hh>
# include <trex/utils/platform/memory.hh>
# include <trex/python/python_env.hh>
# include <trex/python/exception_helper.hh>

# include <ros/ros.h>
# include <ros/callback_queue.h>

namespace TREX {
  namespace ROS {
    
    class roscpp_initializer:boost::noncopyable {
    public:
      
      enum priority {
        init_p = 0,
        default_p
      };
      
      boost::python::object &rospy() {
        return m_rospy;
      }
      
      bool is_shutdown();
      
      ::ros::NodeHandle &handle();
      
      utils::priority_strand &strand() {
        return m_python->strand();
      }
      
    private:
      utils::SharedVar<bool> m_active;
      
      bool test_shutdown() const;
      void do_shutdown();
      
      void init_rospy();
      void init_cpp();
      
      roscpp_initializer();
      ~roscpp_initializer();
      
      boost::scoped_ptr<ros::NodeHandle> m_handle;
      SHARED_PTR<ros::AsyncSpinner> m_spin;
      ros::CallbackQueue *m_cpp;
      
      utils::SingletonUse<utils::LogManager>  m_log;
      utils::SingletonUse<python::python_env> m_python;
      utils::SingletonUse<python::exception_table> m_err;
      
      boost::python::object          m_rospy;
      
      friend utils::SingletonWrapper<roscpp_initializer>;
      friend class lock;
    };
    
  }
}

# endif // H_trex_ros_roscpp_inject