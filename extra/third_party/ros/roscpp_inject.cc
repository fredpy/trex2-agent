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
#include "roscpp_inject.hh"
#include <trex/python/python_thread.hh>

using namespace TREX::ROS;
using namespace TREX::python;
using namespace TREX::utils;
namespace bp=boost::python;

// structors

roscpp_initializer::roscpp_initializer() {
  strand().send(boost::bind(&roscpp_initializer::init_rospy, this), init_p);
}

roscpp_initializer::~roscpp_initializer() {
}



// Manipulators


void roscpp_initializer::init_rospy() {
  try {
    scoped_gil_release lock;
    m_rospy = m_python->import("rospy");
    
    m_rospy.attr("init_node")("trex2", bp::object(), true,
                              bp::object(), false, false,
                              true);
  } catch(bp::error_already_set const &e) {
    m_err->unwrap_py_error();
  }
  strand().send(boost::bind(&roscpp_initializer::async_poll, this));
}

void roscpp_initializer::async_poll() {
//  m_log->syslog("ros", log::info)<<"Polling on rospy";
  try {
    try {
      bool cont = false;
      scoped_gil_release lock;
//      m_log->syslog("ros", log::info)<<"Polling on rospy (2)";
        cont = !bp::extract<bool>(m_rospy.attr("is_shutdown")());
      if( cont ) {
     // m_log->syslog("ros", log::info)<<"Polling on rospy (3)";
       m_rospy.attr("sleep")(0.5);
        strand().send(boost::bind(&roscpp_initializer::async_poll, this));
      }
    } catch(bp::error_already_set const &e) {
      m_err->unwrap_py_error();
    }
    m_log->syslog("ros", log::info)<<"rospy poll done";
  } catch(utils::Exception const &e) {
    m_log->syslog("ros", log::error)<<"TREX Exception on poll: "<<e;
  } catch(std::exception const &se) {
    m_log->syslog("ros", log::error)<<"Exception on poll: "<<se.what();
  } catch(...) {
    m_log->syslog("ros", log::error)<<"Unknown exception on poll";
  }
}


