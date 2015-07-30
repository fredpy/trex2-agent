// -*- C++ -*-
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
#ifndef In_H_trex_ros_cpp_topic
# error "tcc files cannot be included outside of their corresponding header"
#else 

/*
 * class TREX::ROS::cpp_topic
 */

// structors

template<class Msg, bool G, class Cvt>
cpp_topic<Msg, G, Cvt>::cpp_topic(cpp_topic<Msg, G, Cvt>::xml_arg arg)
:details::ros_timeline(arg, Cvt::accept_goals &&
                       utils::parse_attr<bool>(false, xml_factory::node(arg),
                                               "control")),
details::publisher_proxy<Msg, G, Cvt>(ros()),
m_merge(utils::parse_attr<bool>(false, xml_factory::node(arg), "merge")),
m_extend(false),
m_topic(utils::parse_attr<std::string>(xml_factory::node(arg), "topic")) {
  publisher::set_log(boost::bind(&details::ros_timeline::syslog, this, _1));
  
  if( controllable() ) {
    try {
      syslog()<<"Creating publisher to topic "<<m_topic;
      publisher::advertise(m_topic);
      publisher::set_dispatch(boost::bind(&cpp_topic::dispatch, this, _1));
    } catch(ros::Exception const &e) {
      std::ostringstream oss;
      oss<<"Publish on topic \""<<m_topic<<"\" failed: "<<e.what();
      throw ros_error(oss.str());
    }
  }
  try {
    syslog()<<"Subscribing to ROS topic "<<m_topic;
    // subscribe to topic with a wqueue size of 10
    // TODO: make the queue size as an XML option
    m_sub = handle().subscribe(m_topic, 10, &cpp_topic::message, this);
  } catch(ros::Exception const &e) {
    std::ostringstream oss;
    oss<<"Subscription ot topic \""<<m_topic<<"\" failed: "<<e.what();
    throw ros_error(oss.str());
  }
  if( !m_sub ) {
    std::ostringstream oss;
    oss<<"subscriber is null aftter its attchement to \""<<m_topic<<"\".";
    throw ros_error(oss.str());
  }
  
  m_pred = Cvt::msg_type();
  size_t pos = m_pred.find_last_of("/.");
  if( pos!=std::string::npos )
    m_pred = m_pred.substr(pos+1);
}

template<class Msg, bool G, class Cvt>
cpp_topic<Msg, G, Cvt>::~cpp_topic() {
  syslog()<<"close sub to "<<m_topic;
  m_sub.shutdown();
  syslog()<<"no more sub to "<<m_topic;
}


// manipulators

template<class Msg, bool G, class Cvt>
void cpp_topic<Msg, G, Cvt>::dispatch(typename cpp_topic<Msg, G, Cvt>::message_type const &msg) {
  if( controllable() ) {
    syslog()<<"Publish message to topic "<<m_topic<<":\n"<<msg;
    publisher::publish(msg);
  }
}



// callbacks

template<class Msg, bool G, class Cvt>
void cpp_topic<Msg, G, Cvt>::message(cpp_topic<Msg, G, Cvt>::message_ptr msg) {
  transaction::observation_id obs = MAKE_SHARED<transaction::Observation>(new_obs(m_pred));
  
  translator::to_trex(msg, *obs);
  
  if( m_last_obs ) {
    if( m_merge && obs->consistentWith(*m_last_obs) ) {
      m_extend = true;
      return;
    }
  }
  m_last_obs = obs;
  notify(*m_last_obs);
}


template<class Msg, bool G, class Cvt>
void cpp_topic<Msg,G,Cvt>::synchronize(transaction::TICK date) {
  publisher::update_tick(date);
  
  if( controllable() )
    publisher::process_pending(m_last_obs, m_obs_since);

  if( updated() )
    m_obs_since = date;
  else if( !m_extend ) {
    // did not receive any update => set myself to undefined
    m_last_obs.reset();
    m_obs_since = date;
    notify(new_obs(transaction::Predicate::undefined_pred()));
  }
  m_extend = false;
  
}


#endif