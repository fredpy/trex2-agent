/*********************************************************************
 * Software License Agreement (BSD License)
 * 
 *  Copyright (c) 2011, MBARI.
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
#include <set>

#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>

#include <trex/domain/IntegerDomain.hh>
#include <trex/domain/FloatDomain.hh>

#include <mbari/shared/GeoUTM.hh>

#include "DrifterTracker.hh"


using namespace mbari;
using namespace TREX::transaction;

namespace {

  /** @brief Light reactor declaration */
  TeleoReactor::xml_factory::declare<DrifterTracker> decl("DrifterTracker");  

  int const WGS_84 = 23;

}

namespace mbari {

  void geo_to_utm(double lat, double lon, double &north, double &east) {
    char zone[4];
    LLtoUTM(WGS_84, lat, lon, north, east, zone);
  }
  void utm_to_geo(double north, double east, double &lat,  double &lon) {
    const char* Zone = "10";
    const int NORTHERN_HEMISPHERE_BUFFER = 10000000;
    UTMtoLL(WGS_84, 
	    north + NORTHERN_HEMISPHERE_BUFFER, east, 
	    Zone, lat, lon);
    if(east == 0.0)
      lon = -126.0;
  }
} // mbari

DrifterTracker::DrifterTracker(TeleoReactor::xml_arg_type arg) 
  :TeleoReactor(arg, false) {
  m_connection.open("messaging.shore.mbari.org", 5672);
  m_connection.login("tracking", "MBARItracking", "trackingvhost");

  syslog("amqp", info)<<"Creating queue \"trex2"<<getName()<<std::endl;
  m_queue = m_connection.create_queue("trex2"+getName().str());
  
  boost::property_tree::ptree::value_type &node(xml_factory::node(arg));
  TREX::utils::Symbol tl_name;
  std::string exch_name;
  std::set<std::string> exchs;
  TREX::utils::ext_xml(node.second, "config");

  // Now parse the sub tags
  boost::property_tree::ptree::iterator initial = node.second.begin();
  DrifterTracker *me = this;
  MessageHandler::factory::iter_traits<boost::property_tree::ptree::iterator>::type
    it = MessageHandler::factory::iter_traits<boost::property_tree::ptree::iterator>::build(initial, me);
  boost::shared_ptr<MessageHandler> handler;

  while( m_msg_factory->iter_produce(it, node.second.end(), handler) ) {
    // bind to this exchange
    m_queue->bind(handler->exchange(), handler->route());
    // add the handler
    m_message_handlers.insert(std::make_pair(handler->exchange(), handler));
  }

  m_listener.reset(new amqp::listener(m_queue, m_messages));
}

DrifterTracker::~DrifterTracker() {}

void DrifterTracker::handleInit() {
  // It is time to start my thread 
  syslog()<<"Initialize queue.";
  m_queue->configure(false, true, false);
  syslog()<<"Starting the amqp queue listener.";
  m_thread.reset(new boost::thread(*m_listener));
}

void DrifterTracker::goalHandler(std::string const &timeline, 
                                 MessageHandler *handle) {
  m_goal_handlers.insert(std::make_pair(timeline, handle));
}

void DrifterTracker::handleRequest(TREX::transaction::goal_id const &g) {
  std::map<std::string, MessageHandler *>::const_iterator from, to;
  
  boost::tie(from, to) = m_goal_handlers.equal_range(g->object().str());
  for( ; to!=from; ++from) 
    if( from->second->handleRequest(g) )
      return;
}

void DrifterTracker::handleRecall(TREX::transaction::goal_id const &g) {
  syslog("WARN")<<" No support for goal recall yet.";
}


bool DrifterTracker::synchronize() {
  handle_map::const_iterator from, to;


  while( !m_messages.empty() ) {
    boost::shared_ptr<amqp::queue::message> msg = m_messages.pop();
    syslog()<<"New message["<<msg->key()<<"]: "<<msg->size()<<" bytes from \""
            <<msg->exchange()<<"\"";

    boost::tie(from, to) = m_message_handlers.equal_range(msg->exchange());
    for( ; to!=from; ++from) {
      if( from->second->handleMessage(*msg) )
	syslog()<<"AMQP message "<<msg->key()<<" handled.";
    }
  }

  for(from=m_message_handlers.begin(); m_message_handlers.end()!=from; ++from) 
    if( !from->second->synchronize() ) {
      syslog("ERROR")<<" Handler on exchange "<<from->second<<" failed to synchronize";
      return false;
    }
  return true;
}
