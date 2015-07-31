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
#include "trex/ros/cpp_topic.hh"

using namespace TREX::ROS;
using namespace TREX::utils;
using namespace TREX::transaction;


/*
 * class TREX::ROS::details::handle_proxy
 */

using TREX::ROS::details::handle_proxy;

// structors

handle_proxy::handle_proxy(roscpp_initializer &init)
:m_handle(init.handle()) {}

handle_proxy::~handle_proxy() {}


// modifiers

void handle_proxy::set_log(handle_proxy::log_fn f) {
  m_log = f;
}


// manipulators

ros::NodeHandle &handle_proxy::handle() {
  return m_handle;
}

log::stream handle_proxy::log(Symbol const &kind) {
  return m_log(kind);
}

/*
 * class TREX::ROS::details::cpp_topic_base
 */

using TREX::ROS::details::cpp_topic_base;
using TREX::ROS::details::ros_timeline;

// statics

void cpp_topic_base::async_exec(WEAK_PTR<ros_timeline> me,
                                boost::function<void (cpp_topic_base *)> fn) {
  SHARED_PTR<ros_timeline> ptr = me.lock();
  
  if( ptr ) {
    cpp_topic_base *my_self = dynamic_cast<cpp_topic_base *>(ptr.get());
    if( NULL!=my_self )
      fn(my_self);
  }
}


// structors

cpp_topic_base::cpp_topic_base(cpp_topic_base::xml_arg const &arg,
                               bool can_control,
                               std::string type_name)
:ros_timeline(arg, can_control && parse_attr<bool>(false, xml_factory::node(arg),
                                                   "control")),
m_topic(parse_attr<std::string>(xml_factory::node(arg), "topic")),
m_merge(parse_attr<bool>(true, xml_factory::node(arg), "merge")),
m_extend(false) {
  size_t pos = type_name.find_last_of("/.");
  if( std::string::npos!=pos )
    type_name = type_name.substr(pos+1);
  m_pred = parse_attr<std::string>(type_name, xml_factory::node(arg), "pred");
}

cpp_topic_base::~cpp_topic_base() {
  if( m_sub ) {
    syslog()<<"Unsubscribe from \""<<topic()<<"\"";
    m_sub.shutdown();
  }
}


// manipulators

Observation cpp_topic_base::new_obs() const {
  return ros_timeline::new_obs(predicate());
}

Observation cpp_topic_base::new_undefined() const {
  return ros_timeline::new_obs(Predicate::undefined_pred());
}

boost::function<void ()> cpp_topic_base::wrap(boost::function<void (cpp_topic_base *)> fn) {
  WEAK_PTR<ros_timeline> me(shared_from_this());
  return boost::bind(&cpp_topic_base::async_exec, me, fn);
}

void cpp_topic_base::do_notify(Observation const &obs) {
  if( m_last_obs ) {
    if( m_merge && obs.consistentWith(*m_last_obs) ) {
      m_extend = true;
      return;
    }
  }
  m_last_obs = obs;
  notify(obs);
}


// callbacks

void cpp_topic_base::handle_init() {
  SHARED_PTR<ros_timeline> me(shared_from_this());
  
  if( controllable() ) {
    try {
      syslog()<<"Publishing ros topic \""<<m_topic<<"\".";
      init_publisher();
    } catch(ros::Exception const &e) {
      std::ostringstream oss;
      oss<<"Failed to create publisher for \""<<m_topic<<"\": "<<e.what();
      throw ros_error(oss.str());
    }
  }
  
  try {
    syslog()<<"Subscribing to ros topic \""<<m_topic<<"\".";
    init_subscriber(m_sub);
  } catch(ros::Exception const &e) {
    std::ostringstream oss;
    oss<<"Failed to subscribe to \""<<m_topic<<"\": "<<e.what();
    throw ros_error(oss.str());
  }
  if( !m_sub ) {
    std::ostringstream oss;
    oss<<"subscriber is still null after its attachment to \""<<m_topic<<"\"";
    throw ros_error(oss.str());
  }
}

void cpp_topic_base::synchronize(TICK date) {
  if( updated() )
    m_obs_since = date; // in case we posted an observation since last tick
  
  if( controllable() )
    process_pending(date);
 
  if( updated() )
    m_obs_since = date; // in case process_pending updated
  else if( !m_extend ) {
    // did not receive any update => set myself to undefined
    m_last_obs.reset();
    m_obs_since = date;
    notify(new_undefined());
  }
  m_extend = false;
}

bool cpp_topic_base::handle_request(goal_id g) {
  if( controllable() ) {
    if( g->predicate()==predicate() ) {
      add_goal(g);
      return true;
    } else
      syslog(log::warn)<<"Ignoring goal ["<<g<<"] as its predicate do not match "
        <<predicate();
  }
  return false;
}

void cpp_topic_base::handle_recall(goal_id g) {
  if( controllable() && g->predicate()==predicate() )
    remove_goal(g);
}






