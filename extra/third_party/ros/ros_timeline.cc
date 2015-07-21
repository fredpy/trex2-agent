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
#include "ros_reactor.hh"
#include "trex/ros/ros_error.hh"

using namespace TREX::ROS::details;
using namespace TREX::utils;
using namespace TREX::transaction;

using TREX::ROS::ros_reactor;
using TREX::ROS::ros_error;

/*
 * class TREX::ROS::details::ros_timeline
 */

// structors

ros_timeline::ros_timeline(ros_timeline::xml_arg const &arg, bool control)
:m_reactor(*(arg.second)), m_name(parse_attr<Symbol>(xml_factory::node(arg), "name")),
m_controllable(control),
m_init(utils::parse_attr<bool>(false, xml_factory::node(arg), "init")),
m_updated(false) {
  init_timeline();
}

ros_timeline::ros_timeline(ros_reactor *r, Symbol const &tl, bool init, bool control)
:m_reactor(*r), m_name(tl), m_controllable(control), m_init(init), m_updated(false) {
  init_timeline();
}

ros_timeline::~ros_timeline() {
  m_reactor.unprovide(name());
}


// manipulators

void ros_timeline::init_timeline() {
  std::ostringstream oss;
  
  if( m_reactor.isInternal(name()) ) {
    oss<<"Attempted to re-provide "<<name();
    throw ReactorException(m_reactor, oss.str());
  } else {
    m_reactor.provide(name(), controllable());
    if( !m_reactor.isInternal(name()) ) {
      oss<<"Failed to provide "<<name()<<" as Internal";
      throw ReactorException(m_reactor, oss.str());
    }
  }
  if( !m_init )
    notify(new_obs(Predicate::undefined_pred()));
}

log::stream ros_timeline::syslog(log::id_type const &kind) const {
  return m_reactor.syslog(name(), kind);
}


// modifiers

void ros_timeline::notify(transaction::Observation const &obs) {
  if( obs.object()!=name() ) {
    syslog(log::error)<<"IAttempted to post an observation which does not belong "
    "to this timeline:\n  "<<obs;
  } else {
    m_undefined = (obs.predicate()==Predicate::undefined_pred());
    m_reactor.postObservation(obs);
    m_updated = true;
  }
}

// callbacks

void ros_timeline::do_init() {
  if( m_init ) {
    syslog(log::info)<<"Waiting for first update from ROS";
    do {
      boost::this_thread::yield();
    } while( !m_updated );
  }
}

void ros_timeline::do_synchronize() {
  synchronize(m_reactor.getCurrentTick());
  m_updated = m_undefined;
}

bool ros_timeline::request(goal_id g) {
  if( g->object()!=name() )
    syslog(log::error)<<"Goal "<<g<<" is not on "<<name();
  else if( controllable() )
    return handle_request(g);
  return false;
}

void ros_timeline::recall(goal_id g) {
  if( controllable() )
    handle_recall(g);
}





