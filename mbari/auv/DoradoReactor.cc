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
#include "DoradoReactor.hh"

#include <trex/domain/FloatDomain.hh>
#include <trex/domain/BooleanDomain.hh>

#include <boost/lambda/lambda.hpp>

#include <algorithm>
#include <cmath>


using namespace TREX::mbari;
using namespace TREX::transaction;
using namespace TREX::utils;

namespace ip=boost::asio::ip;

/*
 * class TREX::mbari::DoradoReactor
 */

// statics

boost::asio::io_service DoradoReactor::s_io_service;
Symbol const DoradoReactor::s_depth_enveloppe("depthEnveloppe");
Symbol const DoradoReactor::s_sp_timeline("vehicleState");

IntegerDomain::bound const DoradoReactor::s_temporal_uncertainty(2);


// structors 

DoradoReactor::DoradoReactor(DoradoReactor::xml_arg_type &arg)
:TeleoReactor(arg.second, parse_attr<utils::Symbol>(xml_factory::node(arg), "name"),
              0, 1, parse_attr<bool>(true, xml_factory::node(arg), "log")), 
m_sp_socket(DoradoReactor::s_io_service),
m_sp_timer(DoradoReactor::s_io_service),
m_vcs_socket(DoradoReactor::s_io_service), 
m_sp_buff(NULL), m_xdr_buff_size(sizeof(uint16_t)+2*sizeof(StatePacket)),
m_behavior_count(0), m_sequential_count(0), m_sequential_execute(true) {
  
  boost::property_tree::ptree::value_type &node(xml_factory::node(arg));
  // Populate node with external XML config
  TREX::utils::ext_xml(node.second, "config");

  m_vcs_server.address(ip::address_v4::from_string(parse_attr<std::string>(node, "vcs_host")));
  m_vcs_server.port(parse_attr<unsigned short>(8004, node, "vcs_port"));
  syslog(info)<<"VCS address set to TCP : "<<m_vcs_server;
  
  syslog(info)<<"Extracting auv timelines.";
  for(boost::property_tree::ptree::iterator i=node.second.begin();
      node.second.end()!=i; ++i) {
    if( is_tag(*i, "Timeline") ) {
      Symbol name = parse_attr<Symbol>(*i, "name");
      if( name.empty() )
        throw XmlError(*i, "Timeline cannot have an empty name");
      provide(name);
      boost::optional<Symbol> alias =parse_attr< boost::optional<Symbol> >(*i, "alias");
      bool sequential = parse_attr<bool>(true, *i, "sequential");
      if( alias ) {
        syslog(info)<<"Timeline "<<name<<" aliased to command "<<alias;
        m_behaviors[name] = std::make_pair(sequential, *alias);
      } else 
        m_behaviors[name] = std::make_pair(sequential, name);
      postObservation(Observation(name, "Inactive"));
    }
  }
  // TODO: create timelines for the gulpers
  
  
  syslog(info)<<"Adding default timelines";
  provide(s_depth_enveloppe, false);
  provide(s_sp_timeline, false);
  
  bool found;
  
  m_init_plan = manager().use(parse_attr<std::string>(node, "initial_plan"), found);
  if( !found )
    throw XmlError(node, 
                   "Unable to locate initial vcs plan file \""+m_init_plan.string()+"\"");
  
  
  unsigned short my_port = parse_attr<unsigned short>(8002, node, "port");
  syslog(info)<<"Creating state update UDP server on port "<<my_port;
  m_sp_socket.open(ip::udp::v4());
  m_sp_socket.bind(ip::udp::endpoint(ip::udp::v4(), my_port));
  
  // Lastly create xdr buffer
  m_sp_buff = new char[m_xdr_buff_size];
  m_xdr_buff = m_sp_buff+sizeof(uint16_t);
  xdrmem_create(&m_xdr_stream, m_xdr_buff, m_xdr_buff_size-sizeof(uint16_t),
                XDR_DECODE);
}

DoradoReactor::~DoradoReactor() {
  if(NULL!=m_sp_buff ) {
    delete[] m_sp_buff;
    m_sp_buff = m_xdr_buff = NULL;
  }
}

// modifiers 

void DoradoReactor::handleInit() {
  syslog(info)<<"Extracting initial plan from "<<m_init_plan;
  std::ifstream plan_file(m_init_plan.c_str());
  std::ostringstream init_msg;
  
  init_msg<<"init|"<<plan_file.rdbuf();
  plan_file.close();
  
  Observation envelope(s_depth_enveloppe, "Active");
  // Extracting data from depth enveloppe
  std::string plan = init_msg.str(), args;
  size_t b_start = plan.find(s_depth_enveloppe.str()), b_arg_start, b_arg_end;
  b_arg_start = plan.find('{', b_start);
  b_arg_end = plan.find('}', b_arg_start);
  
  // Get the arguments of the depth enveloppe
  args = plan.substr(b_arg_start+1, b_arg_end-b_arg_start-1);
  syslog(info)<<"Parsing argument of "<<s_depth_enveloppe;
  bool end_of_parse = false;
  while( !end_of_parse ) {
    size_t eq_pos, sc_pos;
    
    eq_pos = args.find('=');
    if( std::string::npos==eq_pos ) 
      break;
    sc_pos = args.find(';', eq_pos);
    if( std::string::npos==sc_pos ) {
      sc_pos = args.length();
      end_of_parse = true;
    }
    
    std::string name(args, 0, eq_pos), val(args, eq_pos+1, sc_pos-eq_pos-1);
    size_t v_start, v_end;
    // parse the attribute name
    v_start = name.find_first_not_of(" \n\t");
    v_end = name.find_first_of(" \n\t", v_start);
    if( std::string::npos==v_end )
      name = name.substr(v_start);
    else 
      name = name.substr(v_start, v_end-v_start);
    // parse the numeric value
    v_start = val.find_first_of("-0123456789.e");
    v_end = val.find_first_not_of("-0123456789.e", v_start);
    if( std::string::npos==v_end )
      val = val.substr(v_start);
    else 
      val = val.substr(v_start, v_end-v_start);

    syslog()<<"  - "<<name<<" = "<<val;
    envelope.restrictAttribute(name, FloatDomain(string_cast<double>(val)));
    if( !end_of_parse )
      args = args.substr(sc_pos+1);
  }
  syslog(info)<<"Done parsing";
  postObservation(envelope);
  
  syslog(info)<<"Connecting to vcs server at "<<m_vcs_server;
  m_vcs_socket.connect(m_vcs_server);
  syslog(info)<<"Sending initial plan:\n"<<plan;
  send_string(plan);
  m_layered_control_ready = false;
  m_fresh_xdr = false;
  
  // Now I need to wait for the vehicle feedback
  syslog(info)<<"Waiting for state pusblisher connection";
  while( !get_sp_updates(boost::posix_time::milliseconds(100)) );
  syslog(info)<<"Waiting for Layered Control to be ready";
  while( !( m_fresh_xdr && m_layered_control_ready )) 
    get_sp_updates(boost::posix_time::milliseconds(100));
  syslog(info)<<"VCS connection established.";
  // Notify VCS rhat we are starting
  send_string("start");
}


void DoradoReactor::handleTickStart() {
}

bool DoradoReactor::synchronize() {
  if( m_pinged )
    m_pinged = false;
  else if( m_sequential_count>0 )
    send_string("PING");
  
  boost::posix_time::milliseconds wait(100);
  
  if( get_sp_updates(wait) ) {
    wait = boost::posix_time::milliseconds(10);
    while( get_sp_updates(wait) );
  }
  if( m_fresh_xdr ) {
    StatePacket state;
    
    xdr_setpos(&m_xdr_stream, 0);
    bool decoded = xdr_StatePacket(&m_xdr_stream, &state);
    if( !decoded ) {
      syslog(error)<<"Failed to decode last state packet";
      return false;
    }
    
    Observation obs(s_sp_timeline, "Holds");
    
    // Position
    obs.restrictAttribute("x", FloatDomain(state.position.x));
    obs.restrictAttribute("y", FloatDomain(state.position.y));
    obs.restrictAttribute("z", FloatDomain(state.position.z));
    
    // cluster
    obs.restrictAttribute("cluster_id", IntegerDomain(state.cluster.id));
    
    // CTD
    obs.restrictAttribute("ctdValid", BooleanDomain(state.ctd.valid));
    if( state.ctd.valid ) {
      obs.restrictAttribute("conductivity", FloatDomain(state.ctd.c));
      obs.restrictAttribute("temperature", FloatDomain(state.ctd.t));
      obs.restrictAttribute("density", FloatDomain(state.ctd.d));
      obs.restrictAttribute("salinity", FloatDomain(state.ctd.salinity));
    }
    
    // Isus
    obs.restrictAttribute("isusValid", BooleanDomain(state.isus.valid));
    if( state.isus.valid )
      obs.restrictAttribute("nitrate", FloatDomain(state.isus.nitrate));
    
    // HS2
    obs.restrictAttribute("hydroscatValid", BooleanDomain(state.hydroscat.valid));
    if( state.hydroscat.valid ) {
      obs.restrictAttribute("bb470", FloatDomain(state.hydroscat.bb470));
      obs.restrictAttribute("bb676", FloatDomain(state.hydroscat.bb676));
      obs.restrictAttribute("ch_fl", FloatDomain(state.hydroscat.fl));
    }
    postObservation(obs);
    m_fresh_xdr = false;
    // TODO handle gulpers
    
  } else {
    syslog(warn)<<"No state update received from VCS";
  }
  std::map<utils::Symbol, std::list<Observation> >::iterator o = m_queued_obs.begin();
  
  for( ; m_queued_obs.end()!=o; ++o) {
    if( !o->second.empty() ) {
      postObservation(o->second.front(), true);
      o->second.pop_front();
    }
  }
  
  return true;
}

void DoradoReactor::handleRequest(goal_id const &g) {
  std::map< utils::Symbol, 
            std::pair<bool, 
                      utils::Symbol> >::const_iterator cmd = m_behaviors.find(g->object());
  if( m_behaviors.end()!=cmd ) {
    if( g->predicate()!="Inactive" ) {
      if( cmd->second.first ) {
        m_pending.push_back(g);
        process_pendings();
      } else 
        send_request(g);
    }
  }
}

void DoradoReactor::handleRecall(goal_id const &g) {
  // Check if the goal is already sent
  std::map< int32_t, std::pair<goal_id, bool> >::iterator i=m_sent_cmd.begin();
  
  for( ; m_sent_cmd.end()!=i; ++i) {
    if( g==i->second.first ) {
      std::ostringstream oss;
      
      oss<<"delete|"<<i->first;
      syslog(info)<<"Sending "<<oss.str();
      send_string(oss.str());
      m_pinged = true;
      return;
    }
  }
  
  // Not sent : check if its still on the pending list 
  for(std::list<goal_id>::iterator p=m_pending.begin(); m_pending.end()!=p; ++p) {
    if( g==*p ) {
      m_pending.erase(p);
      return;
    }
  }
}

void DoradoReactor::send_string(std::string const &msg) {
  std::vector<boost::asio::const_buffer> buffs; 

  // identify the length of the string in net format
  uint32_t len = msg.length(), net_len = htonl(len);
 
  // build message <len><content>
  buffs.push_back(boost::asio::buffer(&net_len, sizeof(uint32_t)));
  buffs.push_back(boost::asio::buffer(msg));
 
  // Send the message
  boost::asio::write(m_vcs_socket, buffs);
  m_pinged = true;
}

void DoradoReactor::send_request(transaction::goal_id const &g) {
  IntegerDomain::bound time_out = g->getDuration().upperBound();
  std::ostringstream oss;
  
  // Identifies the maximum duration to be sent
  if( !g->getDuration().isSingleton() ) {
    IntegerDomain::bound late_duration = g->getEnd().upperBound(),
      min_duration = g->getDuration().lowerBound();
    
    late_duration.minus(g->getStart().upperBound(), time_out);
    min_duration.plus(s_temporal_uncertainty, min_duration);
    time_out.minus(s_temporal_uncertainty, time_out);
    
    if( late_duration>=min_duration && late_duration<time_out ) {
      time_out = late_duration;
      time_out.minus(s_temporal_uncertainty, time_out);
    } 
  }  
  int32_t id = next_id();
  
  oss.setf(std::ios::fixed);
  oss<<"insert|behavior "<<m_behaviors[g->object()].second<<" { id="<<id<<"; ";
  if( !time_out.isInfinity() ) {
    // Convert the duration from tick into seconds
    duration_type dur = time_out.value()*tickDuration();
    
    oss<<"duration="
       <<boost::chrono::duration_cast<boost::chrono::seconds>(dur)<<"; ";
  } else
    syslog(warn)<<"Goal "<<g<<" has no maximum duration.";
  
  for(Predicate::const_iterator v=g->begin(); g->end()!=v; ++v) {
    Variable const &var = v->second;
    
    if( var.isComplete() && var.domain().isSingleton() )
      oss<<var.name()<<'='<<var.domain().getStringSingleton()<<"; ";
  }
  
  oss<<"}";
  syslog(info)<<"Sending "<<oss.str(); 
  send_string(oss.str());
  m_sent_cmd[id] = std::make_pair(g, false);
  ++m_behavior_count;
}


bool DoradoReactor::get_sp_updates(boost::posix_time::milliseconds const &timeout) {
  boost::array<char, 3> header;
  bool timed_out = false;
  boost::system::error_code ec(boost::asio::error::would_block);
  boost::asio::ip::udp::endpoint from;
  
  
  // Change ec value on execution :
  m_sp_socket.async_receive_from(boost::asio::buffer(header), 
                                 from, 
                                 boost::lambda::var(ec) = boost::lambda::_1);
  m_sp_timer.expires_from_now(timeout);
  // Set timed_out to true on time out 
  m_sp_timer.async_wait(boost::lambda::var(timed_out) = true);
  
  do {
    // run either of the two async calls
    s_io_service.run_one();
    if( timed_out ) {
      // looks like we timed out => cancel the socket reception
      m_sp_socket.cancel();
      return false;
    }
  } while( boost::asio::error::would_block==ec );
  // The socket did execute before timing out => cancel the timer
  m_sp_timer.cancel();
  // Do things here 
  
  if( ec ) {
    // We had a socket error => report it
    syslog(error)<<"StatePacket socket error: "<<ec;
    throw ec;
  } 
  syslog()<<"New message from "<<from;
  
  switch( header[0] ) {
    case 'S':
      get_statepacket(ntohs(*reinterpret_cast<uint16_t *>(header.data()+1)));
      break;
      
    case 'B':
      // Get behavior information
      get_behavior();
      break;
      
    default:
      syslog(error)<<"Received message from "<<from<<" with unknown header type '"
      <<header[0]<<"'(0x"<<std::hex<<static_cast<short>(header[0])<<").";
      throw ReactorException(*this, "Invalid StatePublisher header received.");
  }
  
  return true;
}

void DoradoReactor::get_behavior() {
  char     ignore, event_type;
  uint32_t net_id;
  boost::array<boost::asio::mutable_buffer, 3> msg = {
    boost::asio::buffer(&ignore, 1),
    boost::asio::buffer(&event_type, 1),
    boost::asio::buffer(&net_id, 1) };
  boost::asio::ip::udp::endpoint from;

  m_sp_socket.receive_from(msg, from);
  int32_t behavior_id = -1;
  
  *reinterpret_cast<uint32_t *>(behavior_id) = ntohs(net_id);
  if( -1==behavior_id && 'A'==event_type ) {
    syslog(warn)<<"Abort received from VCS at "<<from;
    throw ReactorException(*this, "Mission aborted by VCS");
  }
  if( 0==behavior_id && !m_layered_control_ready ) {
    if( 'F'==event_type ) {
      syslog()<<"Behavior with ID "<<behavior_id<<" did complete => LC ready";
      m_layered_control_ready = true;
      m_id = 0;
    }
  } else {
    std::map< int32_t, std::pair<goal_id, bool> >::iterator 
      i = m_sent_cmd.find(behavior_id);
    
    if( m_sent_cmd.end()==i ) {
      syslog(warn)<<"Ignoring event from unknown behavior id "<<behavior_id;
    } else if( 'F' == event_type ) {
      // Behavior completed 
      syslog(info)<<i->second.first->object()<<"[id="<<behavior_id<<"] FINISHED.";
      if( !i->second.second ) {
        syslog(warn)<<"behavior "<<behavior_id<<" completed before being started ...";
        queue_obs(*(i->second.first));
      }
      queue_obs(Observation(i->second.first->object(), "Inactive"));
      m_behavior_count = std::max(1ul, m_behavior_count)-1;
      if( m_behaviors[i->second.first->object()].first ) {
        m_sequential_count = std::max(1ul, m_sequential_count)-1;
        m_sequential_execute = (0==m_sequential_count);
        process_pendings();
      }
      m_sent_cmd.erase(i);
    } else if( 'S' == event_type ) {
      if( !i->second.second ) {
        syslog(info)<<i->second.first->object()<<"[id="<<behavior_id<<"] STARTED.";
        queue_obs(*(i->second.first));
        if( m_behaviors[i->second.first->object()].first ) {
          m_sequential_execute = (1==m_sequential_count);
          process_pendings();
        }
        i->second.second = true;
      } else {
        syslog(warn)<<"Ignoring multiple starts for behavior "<<behavior_id;
      }
    } else {
      syslog(warn)<<"Ignoring unknown behavior event '"<<event_type<<"'(0x"
      <<std::hex<<static_cast<short>(event_type)<<std::dec<<") for behavior "
      <<behavior_id;
    }
  }
}

void DoradoReactor::get_statepacket(uint16_t size) {
  static bool first = true;
  
  if( first ) {
    first = false;
    if( size+3 < static_cast<uint16_t>(m_xdr_buff_size) )
      m_xdr_buff_size = size+3;
  }
  if( size+3 != static_cast<uint16_t>(m_xdr_buff_size) ) {
    syslog(error)<<"State packet header is inconsitent with previous size value ( "
    <<size+3<<"!="<<m_xdr_buff_size<<").";
    throw ReactorException(*this, "Spurious state packet update.");
  }
  boost::asio::ip::udp::endpoint from;

  size_t len = m_sp_socket.receive_from(boost::asio::buffer(m_sp_buff, 
                                                            m_xdr_buff_size), 
                                        from);
  if( len!=m_xdr_buff_size ) {
    syslog(error)<<"Data received ("<<len<<") differs from expected ("<<m_xdr_buff_size<<")";
    throw ReactorException(*this, "StatePacket reception error"); 
  }
  m_fresh_xdr = true;
}

size_t DoradoReactor::next_id() {
  return ++m_id;
}

void DoradoReactor::queue_obs(Observation const &obs) {
  std::list<Observation> &events = m_queued_obs[obs.object()];
  
  if( !events.empty() ) {
    if( events.back().predicate()=="Inactive" )
      events.pop_back();
  }
  events.push_back(obs);
}

void DoradoReactor::process_pendings() {
  if( m_sequential_execute || 0==m_sequential_count ) {
    if( !m_pending.empty() ) {
      goal_id next(m_pending.front());
      m_pending.pop_front();
      // TODO: Check if this goal is executable (ie can be started on next tick) 
      m_sequential_count += 1;
      m_sequential_execute = false;
      send_request(next);
    }
  }
}




