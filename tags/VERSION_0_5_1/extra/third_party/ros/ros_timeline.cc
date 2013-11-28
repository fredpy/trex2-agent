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
#include "bits/ros_timeline.hh"
#include "ros_reactor.hh"

using namespace TREX::ROS;
using namespace TREX;

using TREX::utils::Symbol;

/*
 * class TREX::ROS::details::ros_timeline
 */
details::ros_timeline::ros_timeline(details::ros_timeline::xml_arg arg, bool control)
  :m_reactor(*arg.second), m_name(utils::parse_attr<Symbol>(xml_factory::node(arg), "timeline")),
   m_controlable(control), m_updated(false) {
  init_timeline();
}

details::ros_timeline::ros_timeline(ros_reactor *r, Symbol const &tl, bool control)
  :m_reactor(*r), m_name(tl), m_controlable(control), m_updated(false) {
  init_timeline();
}

details::ros_timeline::~ros_timeline() {
  m_reactor.unprovide(name());
}

void details::ros_timeline::init_timeline() {
  std::ostringstream oss;
  if( m_reactor.isInternal(name()) ) {
    oss<<"Attempted to provide \""<<name()<<"\" which is already owned.";
    throw TREX::transaction::ReactorException(m_reactor, oss.str());
  } else {
    m_reactor.provide(name(), controlable());
    if( !m_reactor.isInternal(name()) ) {
      oss<<"Failed to declare  \""<<name()<<"\" as Internal.";
      throw TREX::transaction::ReactorException(m_reactor, oss.str());
    }
    // Initialize the observation to undefined
    notify(new_obs(TREX::transaction::Predicate::undefined_pred()));
  }
}

void details::ros_timeline::do_synchronize() {
  // syslog()<<name()<<" - Updated="<<m_updated;
  synchronize();
  m_updated = m_undefined;  
}


::ros::NodeHandle &details::ros_timeline::node() {
  return m_reactor.m_ros;
}

void details::ros_timeline::notify(transaction::Observation const &obs) {
  if( obs.object()!=m_name ) {
    syslog(utils::log::error)<<"Attempted to post an observation which does not belong to "<<m_name<<":\n\t"<<obs;
  } else {
    // Probably need to be protected by some mutex....
    m_undefined = (obs.predicate()==TREX::transaction::Predicate::undefined_pred());
    m_reactor.postObservation(obs);
    m_updated = true;
  }
}

utils::log::stream details::ros_timeline::syslog(Symbol const &kind) {
  return m_reactor.syslog(m_name, kind);
}

bool details::ros_timeline::request(TREX::transaction::goal_id g) {
  if( g->object()!=name() )
    syslog(utils::log::error)<<"Goal "<<g<<" is not on "<<name();
  else if( controlable() )
    return handle_request(g);
  return false;
}

void details::ros_timeline::recall(TREX::transaction::goal_id g) {
  if( controlable() )
    handle_recall(g);
}

void details::ros_timeline::complete(TREX::transaction::goal_id g) {
  m_reactor.completed(g);
}
