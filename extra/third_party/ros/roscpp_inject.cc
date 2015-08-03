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
#include "trex/ros/roscpp_inject.hh"
#include <trex/python/python_thread.hh>
#include <boost/date_time/posix_time/ptime.hpp>

using namespace TREX::ROS;
using namespace TREX::python;
using namespace TREX::utils;
namespace bp=boost::python;
namespace bpt=boost::posix_time;


// structors

roscpp_initializer::roscpp_initializer():m_timer(m_python->strand().strand().get_io_service()) {
}

roscpp_initializer::~roscpp_initializer() {
  if( ros::ok() ) {
    m_timer.cancel();
    m_log->syslog("ros", log::info)<<"Shutting down ros c++";
    ros::shutdown();
    ros::waitForShutdown();
    m_log->syslog("ros", log::info)<<"C++ ROS is dead";
  }
  
  bool active = strand().is_active();
  m_active = false;
  if( active ) {
    strand().stop();
    
    while( !strand().completed() ) {
      boost::this_thread::yield();
    }
  }
  do_shutdown();
  if( active )
    strand().start();
}

// Observers

bool roscpp_initializer::is_shutdown() {
  return !ros::ok();
}


// Manipulators

bp::object &roscpp_initializer::rospy() {
  if( !m_rospy ) {
    boost::packaged_task<void> fn(boost::bind(&roscpp_initializer::init_rospy, this));
    boost::shared_future<void> ret = fn.get_future().share();
    
    strand().strand().dispatch(boost::bind(&boost::packaged_task<void>::operator(), &fn));
    
    ret.get();
  }
  return m_rospy;
}


ros::NodeHandle &roscpp_initializer::handle() {
  if( !m_handle ) {
    boost::packaged_task<void> fn(boost::bind(&roscpp_initializer::init_cpp, this));
    boost::shared_future<void> ret = fn.get_future().share();
    
    strand().strand().dispatch(boost::bind(&boost::packaged_task<void>::operator(), &fn));
    
    ret.get();
  }
  return *m_handle;
}


void roscpp_initializer::init_rospy() {
  if( !m_rospy ) {
    m_log->syslog("ros", log::info)<<"Initialize python ros client";

    try {
      scoped_gil_release lock;
      // Load ROS Python API
      m_rospy = m_python->import("rospy");
    
      // Initialize ROS client node through Python API
      m_rospy.attr("init_node")("trex_py", bp::object(), true,
                                bp::object(), false, false,
                                true);
    } catch(bp::error_already_set const &e) {
      m_err->unwrap_py_error();
    }
    m_active = true;
  }
}

void roscpp_initializer::init_cpp() {
  if( !ros::isInitialized() ) {
    int argc = 0;
    char **argv = NULL;
    
    m_log->syslog("ros", log::info)<<"Initialize roscpp client";
    ros::init(argc, argv, "trex_cpp",
              ::ros::init_options::AnonymousName|::ros::init_options::NoSigintHandler);
    m_handle.reset(new ros::NodeHandle);
    m_timer.expires_from_now(bpt::millisec(50));
    m_timer.async_wait(boost::bind(&roscpp_initializer::async_poll, this, _1));
  }
}


void roscpp_initializer::do_shutdown() {
  if( m_rospy ) {
    try {
      scoped_gil_release lock;
      
      m_log->syslog("ros", log::info)<<"Send shutdown to rospy";
      rospy().attr("signal_shutdown")("end of TREX");
    } catch(bp::error_already_set const &e) {
      m_err->unwrap_py_error(m_log->syslog("ros", log::error)<<"Exception during ROS shutdown: ");
    }
    m_log->syslog("ros", log::info)<<"Shutdown completed";
  }
}

void roscpp_initializer::async_poll(boost::system::error_code const &ec) {
  if( !ec )
    try {
      if( ::ros::ok() ) {
        m_log->syslog("ros", log::info)<<"poll";
        ::ros::spinOnce();
        if( ::ros::ok() ) {
          m_timer.expires_from_now(bpt::millisec(50));
          m_timer.async_wait(boost::bind(&roscpp_initializer::async_poll, this, _1));
        }
      }
    } catch(ros::Exception const &e) {
      m_log->syslog("ros", log::error)<<"ROS exception during poll: "<<e.what();
    }
}



