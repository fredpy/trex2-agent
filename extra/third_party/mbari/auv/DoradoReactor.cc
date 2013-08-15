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
#include "StatePacket.h"

#include <trex/domain/FloatDomain.hh>
#include <trex/domain/BooleanDomain.hh>
#include <trex/domain/IntegerDomain.hh>

#include "auv_factors.hh"

#include <boost/bimap.hpp>

namespace asio=boost::asio;

namespace TREX {
  namespace mbari {
    
    class DoradoReactor::msg_api
    :boost::noncopyable
    ,public ENABLE_SHARED_FROM_THIS<DoradoReactor::msg_api> {
    public:
      static unsigned short const out_default;
      static unsigned short const in_default;
      
      msg_api(asio::io_service &io, DoradoReactor &owner);
      ~msg_api();
      
      void connect(std::string const &host, unsigned short out_port, unsigned short in_port);
      bool connected() const;
      void disconnect();
      
      void post_behavior(transaction::goal_id req);
      
      /** @brief Send a message
       *
       * @param[in] msg The message to b sent
       *
       * @pre This instance is connected
       *
       * Send the text message @p msg to the out server this instance is 
       * connected to with the proper encoding. This call will block until 
       * message sending completion or a failure.
       *
       * @throw boost::system::system_error An error coccured while trying to 
       * send the message
       */
      void send(std::string const &msg, bool verbose=false);

      void start();
      bool started() const;
      bool stop();
      
      bool inited();
      
      bool get_state_packet(StatePacket &p);
      
      void add_alias(utils::Symbol name, utils::Symbol target) {
        m_alias[name] = target;
      }
      
      void ping(bool test = true);

    private:
      std::map<utils::Symbol, utils::Symbol> m_alias;
      
      DoradoReactor &m_owner;

      bool m_init, m_pinged;
      boost::promise<bool> m_init_p;
      
      
      asio::ip::address m_server_ip;
      
      // udp socket for incoming messages
      asio::ip::udp::socket   m_in_socket;
      // incoming udp message header
      boost::array<char, 3>     m_sp_header;
      boost::optional<uint16_t> m_xdr_buff_size;

      typedef boost::array<char, sizeof(uint16_t)+2*sizeof(StatePacket)> xdr_buff_t;
      SHARED_PTR<xdr_buff_t>  m_xdr_buff;
      xdr_buff_t m_xdr_data;
      XDR m_xdr_stream;
            
      // incoming udp message sender
      asio::ip::udp::socket::endpoint_type m_from;
      
      // tcp socket for outgoing messages
      asio::ip::tcp::endpoint m_out_server;
      asio::ip::tcp::socket   m_out_socket;
      
      utils::SharedVar<int32_t> m_cpt;
      typedef boost::bimap<int32_t, transaction::goal_id> req_map;
      req_map m_goals;
      
      
      utils::Symbol alias(utils::Symbol const &name) const;
      
      int32_t get(transaction::goal_id g) {
        utils::SharedVar<int32_t>::scoped_lock lock(m_cpt);
        req_map::right_const_iterator pos = m_goals.right.find(g);
        
        if( m_goals.right.end()==pos ) {
          int32_t idx = ++(*m_cpt);
          m_goals.insert(req_map::value_type(idx, g));
          
          return idx;
        } else
          return pos->second;
      }
      
      transaction::goal_id get(int32_t id) {
        utils::SharedVar<int32_t>::scoped_lock lock(m_cpt);
        req_map::left_const_iterator pos = m_goals.left.find(id);
        transaction::goal_id ret;
        
        if( m_goals.left.end()!=pos )
          ret = pos->second;
        return ret;
      }
      
      void erase(transaction::goal_id g) {
        utils::SharedVar<int32_t>::scoped_lock lock(m_cpt);
        m_goals.right.erase(g);
      }
      void erase(int32_t id) {
        utils::SharedVar<int32_t>::scoped_lock lock(m_cpt);
        m_goals.left.erase(id);
      }
      
      void async_rcv();
      void sp_header_rvcd(boost::system::error_code const & error, // Result of operation.
                          std::size_t bytes_transferred);
      void get_state_packet(size_t size);
      void get_behavior();
      
      
      void behavior_started(int32_t id);
      void behavior_finished(int32_t id);
      
      mutable utils::SharedVar<bool> m_started;
      static size_t const s_xdr_size;
      
      
      bool valid_sender(asio::ip::udp::endpoint const &from) const;
      
      
      msg_api() DELETED;
    };
    
    
  }
}

using namespace TREX::mbari;
using namespace TREX::utils;

namespace tlog=TREX::utils::log;

/*
 * class TREX::mbari::DoradoReactor::msg_api
 */

// statics 

unsigned short const DoradoReactor::msg_api::out_default(8004);
unsigned short const DoradoReactor::msg_api::in_default(8002);
size_t const DoradoReactor::msg_api::s_xdr_size(sizeof(uint16_t)+2*sizeof(StatePacket));

// structors

DoradoReactor::msg_api::msg_api(asio::io_service &io, DoradoReactor &owner)
:m_owner(owner), m_in_socket(io), m_out_socket(io) {
  // Set up the xdr stream
  xdrmem_create(&m_xdr_stream, m_xdr_data.data()+sizeof(uint16_t),
                2*sizeof(StatePacket), XDR_DECODE);
}

DoradoReactor::msg_api::~msg_api() {
  disconnect();
}

// modifiers 

void DoradoReactor::msg_api::connect(std::string const &host,
                                     unsigned short out_port,
                                     unsigned short in_port) {
  if( connected() )
    disconnect();
  m_owner.syslog(tlog::info)<<"Connecting to AUV at "<<host
  <<"\n\t- outbound TCP port "<<out_port
  <<"\n\t- inbound UDP port "<<in_port;
  
  // First try to resolve the name
  asio::ip::tcp::resolver dns(m_out_socket.get_io_service());
  asio::ip::tcp::resolver::query q(host, "");
  asio::ip::tcp::resolver::iterator iter = dns.resolve(q);
  
  // store sresolved server address
  m_server_ip = iter->endpoint().address();
  
  asio::ip::tcp::endpoint tcp_server;
  
  tcp_server.address(m_server_ip);
  tcp_server.port(out_port);
  
  // Attempt to connect to vcs for outgoing tcp messages
  m_out_socket.connect(tcp_server);
  
  // Create udp connection for incoming udp messages
  m_in_socket.open(asio::ip::udp::v4());
  m_in_socket.bind(asio::ip::udp::endpoint(asio::ip::udp::v4(), in_port));
}

void DoradoReactor::msg_api::disconnect() {
  m_owner.syslog(tlog::info)<<"Disconnecting from AUV";
  stop();
  m_in_socket.close();
  m_out_socket.close();
}

void DoradoReactor::msg_api::send(std::string const &msg, bool verbose) {
  std::vector<asio::const_buffer> buffs;
  uint32_t len = msg.length(), net_len = htonl(len);
  
  // A string packet is encoded as follow :
  //  - int 32 with the length of the string (with endianess encoding)
  //  - the corresponding number of characters
  buffs.push_back(asio::buffer(&net_len, sizeof(uint32_t)));
  buffs.push_back(asio::buffer(msg));
  
  if( verbose )
    m_owner.syslog()<<"Sending: "<<msg;
  // Directly write the message
  asio::write(m_out_socket, buffs);
  m_pinged = true;
}

void DoradoReactor::msg_api::ping(bool test) {
  if( test && !m_pinged )
    send("PING");
  m_pinged = false;
}


void DoradoReactor::msg_api::start() {
  if( connected() ) {
    bool running = true;
    m_started.swap(running);
    if( !running )
      async_rcv();
  }
}

void DoradoReactor::msg_api::async_rcv() {
  // Post my asynchronous listen to m_out_socket only if still active
  if( started() )
    m_in_socket.async_receive_from(asio::buffer(m_sp_header), m_from,
                                   boost::bind(&msg_api::sp_header_rvcd,
                                               shared_from_this(),
                                               _1, _2));
}


void DoradoReactor::msg_api::post_behavior(TREX::transaction::goal_id req) {
  int32_t id = get(req);
  std::ostringstream oss;

  
  transaction::IntegerDomain::bound duration = req->getDuration().upperBound();
  boost::optional<size_t> vcs_dur;
  
  if( duration.isInfinity() ) {
    m_owner.syslog(tlog::warn)<<"No maximum duration set for goal "<<req<<" on "<<req->object();
  } else {
    vcs_dur = duration.value();
    if( 2 < *vcs_dur )
      *vcs_dur -= 2;
      
  }
  
  oss.setf(std::ios::fixed);
  oss<<"insert|behavior "<<alias(req->object())<<" { "
  <<" id="<<id<<"; ";
  if( vcs_dur )
    oss<<" duration="<<*vcs_dur<<"; ";
  
  for(transaction::Predicate::const_iterator a=req->begin(); req->end()!=a; ++a) {
    transaction::DomainBase const &dom = a->second.domain();
    
    if( dom.isSingleton() ) {
      // TODO: handle the special case for boolean domains : vcs expect True or False
      oss<<a->first<<"="<<dom.getStringSingleton()<<"; ";
    } else {
      m_owner.syslog(tlog::warn)<<"Ignoring attribute "<<a->first<<" of request "<<req<<" as its domain is not singleton ("<<dom<<")";
    }
  }
  oss<<"}";
  
  send(oss.str(), true);
}


bool DoradoReactor::msg_api::stop() {
  bool ret = false;
  m_started.swap(ret);
  return ret;
}

void DoradoReactor::msg_api::sp_header_rvcd(boost::system::error_code const & error, // Result of operation.
                                            std::size_t bytes_transferred) {
  if( started() ) {
    if( error ) {
      // Handle the error message
      m_owner.syslog(tlog::error)<<"Error while waiting for new message: "<<error.message();
      disconnect();
    } else if( valid_sender(m_from) ) {
      // Check the type of the message
      
      switch (m_sp_header[0]) {
        case 'S':
          get_state_packet(*reinterpret_cast<uint16_t *>(m_sp_header.data()+1));
          break;
        case 'B':
          get_behavior();
          break;
        default:
          // handle unknown error
          m_owner.syslog(tlog::error)<<"Unknown message header type ("<<m_sp_header[0]
          <<"=0x"<<std::hex<<static_cast<short>(m_sp_header[0])<<std::dec<<")";
          break;
      }
      // Repost 
      async_rcv();
    } else {
      // Notify that the message has been disregarded
      m_owner.syslog(tlog::warn)<<"Ignoring message from unexpected source: "<<m_from;
    }
  }
}



void DoradoReactor::msg_api::get_state_packet(size_t size) {
  if( m_xdr_buff_size ) {
    if( size+3 != *m_xdr_buff_size ) {
      // error
      return; // but should throw an exception instead
    }
  } else
    m_xdr_buff_size = size+3;

  asio::ip::udp::endpoint  from;
  SHARED_PTR<xdr_buff_t> buff = MAKE_SHARED<xdr_buff_t>();
  
  size_t len;
  bool received = false;
  
  
  while( !received ) {
    len = m_in_socket.receive_from(asio::buffer(buff.get(), *m_xdr_buff_size), from);
    if( !valid_sender(from) ) {
      m_owner.syslog(tlog::warn)<<"Ignoring "<<len<<" bytes received from unexpeted source: "<<from;
    } else
      received = true;
  }
  
  if( len!=*m_xdr_buff_size ) {
    m_owner.syslog(tlog::error)<<"Received "<<len<<" bytes when a state packet should be "<<*m_xdr_buff_size;
    return; // probably better to throw
  }
  m_xdr_buff = buff;
}


bool DoradoReactor::msg_api::get_state_packet(StatePacket &p) {
  SHARED_PTR<xdr_buff_t> tmp;
  m_xdr_buff.swap(tmp);
  
  if( tmp ) {
    m_xdr_data = *tmp;
    xdr_setpos(&m_xdr_stream, 0);
    return xdr_StatePacket(&m_xdr_stream, &p);
  }
  return false;
}


void DoradoReactor::msg_api::get_behavior() {
  boost::array<char, 6> msg;
  asio::ip::udp::endpoint  from;
  uint32_t id;
  uint32_t *net_id = reinterpret_cast<uint32_t *>(msg.data()+2);
  uint32_t *local_id = reinterpret_cast<uint32_t *>(&id);

  
  bool received = false;
  size_t len;
  
  while( !received ) {
    len = m_in_socket.receive_from(asio::buffer(msg), from);
    if( valid_sender(from) )
      received = true;
    else
      m_owner.syslog(tlog::warn)<<"Ignoring "<<len<<" bytes received from unexpeted source: "<<from;
  }
  if( 0==len ) {
    m_owner.syslog(tlog::error)<<"Connection to AUV lost";
    return; // probably better to throw 
  }
  // Convert net id inot local id
  *local_id = ntohl(*net_id);
  
  if( -1==id && 'A'==msg[1] ) {
    m_owner.syslog(tlog::error)<<"Received an ABORT from AUV";
    // Do something to disconnect
    return;
  }

  if( '0'==id && !m_init ) {
    if( 'F'==msg[1] ) {
      m_init = true;
      m_init_p.set_value(m_init);
    }
  } else {
    switch( msg[1] ) {
      case 'S':
        // started event
        behavior_started(id);
        break;
      case 'F':
        behavior_finished(id);
        break;
      default:
        m_owner.syslog(tlog::warn)<<"Unknown behavior event ("<<msg[1]
        <<"=0x"<<std::hex<<static_cast<short>(msg[1])<<") for behavior id="
        <<id;
        return;
    }
  }
  
}

void DoradoReactor::msg_api::behavior_started(int32_t id) {
  transaction::goal_id g = get(id);

  if( !g && id>0 )
    m_owner.syslog(tlog::warn)<<"Received end notification from unknown behavior id="<<id;
  
  if( m_init )
    m_owner.started(g);
}

void DoradoReactor::msg_api::behavior_finished(int32_t id) {
  transaction::goal_id g = get(id);

  if( !g && id>0 )
    m_owner.syslog(tlog::warn)<<"Received start notification from unknown behavior id="<<id;
  erase(id);
  if( m_init )
    m_owner.completed(g);
}

// observers

Symbol DoradoReactor::msg_api::alias(Symbol const &name) const {
  std::map<Symbol, Symbol>::const_iterator pos = m_alias.find(name);
  if( m_alias.end()!=pos )
    return pos->second;
  return name;
}

bool DoradoReactor::msg_api::connected() const {
  return m_in_socket.is_open() && m_out_socket.is_open();
}


bool DoradoReactor::msg_api::started() const {
  SharedVar<bool>::scoped_lock lck(m_started);
  return *m_started;
}

bool DoradoReactor::msg_api::valid_sender(asio::ip::udp::endpoint const &from) const {
  return from.address()==m_server_ip;
}

bool DoradoReactor::msg_api::inited() {
  return m_init_p.get_future().get();
}



namespace fs=boost::filesystem;

using TREX::transaction::FloatDomain;
using TREX::transaction::BooleanDomain;
using TREX::transaction::IntegerDomain;

#include <boost/math/special_functions/pow.hpp>
#include <boost/math/special_functions/round.hpp>

namespace {
  
  template<typename Ty, int places>
  Ty round_p(Ty v) {
    static Ty const factor = boost::math::pow<places>(Ty(10));
    return boost::math::round(factor*v)/factor;
  }

}

/*
 * class DoradoReactor
 */

// statics

Symbol const DoradoReactor::s_inactive("Inactive");
Symbol const DoradoReactor::s_depth_envelope("depthEnvelope");
Symbol const DoradoReactor::s_sp_timeline("vehicleState");

// structors

DoradoReactor::DoradoReactor(TREX::transaction::TeleoReactor::xml_arg_type arg)
:TREX::transaction::TeleoReactor(arg.second, parse_attr<utils::Symbol>(xml_factory::node(arg), "name"),
                                 0, 1, parse_attr<bool>(true, xml_factory::node(arg), "log")) {
  boost::property_tree::ptree::value_type &node(xml_factory::node(arg));
  m_api = MAKE_SHARED<msg_api>(boost::ref(manager().service()), boost::ref(*this));
  
  // Populate with external config
  ext_xml(node.second, "config");
  
  std::ostringstream oss;
  write_json(oss, node.second);
  syslog()<<node.first<<": "<<oss.str();
  
  // Declare all of my timelines
  provide(s_sp_timeline, false);  // state updates are read only
  provide(s_depth_envelope, false); // so is the depth enveloppe
  
  syslog()<<"Extracting behavior timelines";
  for(boost::property_tree::ptree::iterator i = node.second.begin();
      node.second.end()!=i; ++i) {
    if( is_tag(*i, "Timeline") ) {
      Symbol name = parse_attr<Symbol>(*i, "name");
      if( name.empty() )
        throw XmlError(*i, "Behavior cannot have an empty name");
      provide(name);
      boost::optional<Symbol> alias = parse_attr< boost::optional<Symbol> >(*i, "alias");
      bool sequential = parse_attr<bool>(true, *i, "sequential"); // behaviors are sequential by default
      
      if( alias ) {
        if( alias->empty() )
          throw XmlError(*i, "Behavior alias cannot be empty");
        m_api->add_alias(name, *alias);
      }
      if( sequential )
        m_sequential.insert(name);
      
      postObservation(TREX::transaction::Observation(name, s_inactive));
    }
  }
  // TODO: create also gulper timelines
  
  bool found;
  
  fs::path init = manager().use(parse_attr<std::string>("auv_init.cfg", node, "initial_plan"),
                                               found);
  if( !found )
    throw XmlError(node, "Unable to locate auv inital plan file \""+init.string()+"\"");
  // get socket info
  std::string host = parse_attr<std::string>(node, "vcs_host");
  unsigned short out_port = parse_attr<unsigned short>(8004, node, "vcs_port"),
    in_port = parse_attr<unsigned short>(8002, node, "port");
  
  syslog()<<"Connecting to vcs server at "<<host<<':'<<out_port;
  m_api->connect(host, out_port, in_port);
  initial_plan(init); // extract and send the initial plan to vcs
}

DoradoReactor::~DoradoReactor() {
  m_api->disconnect();
}

// manipulators

void DoradoReactor::initial_plan(fs::path const &file) {
  // Load the content of the file
  TREX::transaction::Observation envelope(s_depth_envelope, "Active");
  
  syslog(tlog::info)<<"Extracting initial plan from "<<file;

  std::string plan, de_args;
  {
    std::ifstream f(file.c_str());
    std::ostringstream oss;
    oss<<"init|"<<f.rdbuf();
    plan = oss.str();
    
    //
    // Now start a very loose parsing of depthEnvelope
    //
    
    // locate depthEnvelope
    size_t b_start = plan.find(s_depth_envelope.str()), b_arg_start, b_arg_end;
    // Locate the brackets delimiting depthEnvelope arguments
    b_arg_start = plan.find('{', b_start);
    b_arg_end = plan.find('}', b_arg_start);
    de_args = plan.substr(b_arg_start+1, b_arg_end-b_arg_start-1);
  }
  syslog(tlog::info)<<"Parsing arguments of "<<s_depth_envelope;
  bool end_of_parse = false;
  
  while( !end_of_parse ) {
    size_t eq_pos, sc_pos;
    
    eq_pos = de_args.find('=');
    if( std::string::npos==eq_pos )
      break; // No more '=' ; no more variables to parse
    sc_pos = de_args.find(';', eq_pos);
    if( std::string::npos==sc_pos ) {
      sc_pos = de_args.length(); // No ';' means that it is the last argument
      end_of_parse = true;
    }
    
    std::string var_name(de_args, 0, eq_pos), // name of the variable is before the '='
      value(de_args, eq_pos+1, sc_pos-eq_pos-1); // value is between '=' and ';'
    
    // trim white spaces on var_name
    size_t v_start = var_name.find_first_not_of(" \n\t"), v_end;
    v_end = var_name.find_first_of(" \n\t", v_start);
    if( std::string::npos==v_end )
      var_name = var_name.substr(v_start);
    else
      var_name = var_name.substr(v_start, v_end-v_start);
    // same for value except that here we target for a float
    v_start = value.find_first_of("0123456789.e-");
    v_end = value.find_first_not_of("0123456789.e-", v_start);
    if( std::string::npos==v_end )
      value = value.substr(v_start);
    else
      value = value.substr(v_start, v_end-v_start);
    // Now that I have cleand up these lets try to add these as an argument
    envelope.restrictAttribute(var_name, FloatDomain(string_cast<double>(value)));
    
    if( !end_of_parse ) // Remove parsed element fro mthe string
      de_args = de_args.substr(sc_pos+1);
  }
  // done parsing
  postObservation(envelope); // Now post the resulting depth envelope
  m_api->send(plan, true);
}

// callbacks

void DoradoReactor::handleInit() {
  // At this point the inital plan was already sent (during construction)
  // I just need to wait for inital feedback
  syslog(tlog::info)<<"Waiting for VCS to complete its initial plan";
  m_api->start();
  if( !m_api->inited() )
    throw TREX::transaction::ReactorException(*this, "Failed to complete VCS handshake");
  m_api->send("start");
  syslog(tlog::info)<<"VCS ready : sent \"start\" to confirm hand-shake";
  m_api->ping(false); // Just to reset the ping without sending the ping
  m_seq_count = 0;
  m_last_state = MAKE_SHARED<transaction::Observation>(s_sp_timeline, transaction::Predicate::undefined_pred);
  postObservation(*m_last_state);
  m_last_packet = getInitialTick();
}

void DoradoReactor::handleTickStart() {
  // send a ping whenever we did not send any message and are not rtunning a bahvior
  m_api->ping(m_seq_count==0);
}
 
bool DoradoReactor::synchronize() {
  //
  // Produce sensors update
  //
  StatePacket sp;
  transaction::TICK now = getCurrentTick();
  
  if( m_api->get_state_packet(sp) ) {
    m_last_state = MAKE_SHARED<transaction::Observation>(s_sp_timeline, "Holds");
    
    // get lat,lon, northing, easting, depth
    m_last_state->restrictAttribute("latitude", FloatDomain(sp.position.latitude));
    m_last_state->restrictAttribute("longitude", FloatDomain(sp.position.longitude));
    // round all metric values to cm accuracy
    m_last_state->restrictAttribute("x", FloatDomain(round_p<long double, 2>(sp.position.x)));
    m_last_state->restrictAttribute("y", FloatDomain(round_p<long double, 2>(sp.position.y)));
    m_last_state->restrictAttribute("z", FloatDomain(round_p<long double, 2>(sp.position.z)));
    
    // there used to be cluster data in here bu I don't really use it anymore
    
    // CTD data
    m_last_state->restrictAttribute("ctdValid", BooleanDomain(sp.ctd.valid));
    if( sp.ctd.valid ) {
      m_last_state->restrictAttribute("conductivity", FloatDomain(sp.ctd.c));
      m_last_state->restrictAttribute("temperature", FloatDomain(sp.ctd.t));
      m_last_state->restrictAttribute("density", FloatDomain(sp.ctd.d));
      m_last_state->restrictAttribute("salinity", FloatDomain(sp.ctd.salinity));
    }
    
    // Isus data
    m_last_state->restrictAttribute("isusValid", BooleanDomain(sp.isus.valid));
    if( sp.isus.valid ) {
      m_last_state->restrictAttribute("nitrate", FloatDomain(sp.isus.nitrate));
    }
    
    // Backscatter data
    m_last_state->restrictAttribute("hydroscatValid", BooleanDomain(sp.hydroscat.valid));
    if( sp.hydroscat.valid ) {
      // NOTE: all the hs2 data are changed by a factor to better handle small values
      m_last_state->restrictAttribute("ch_fl", FloatDomain(sp.hydroscat.fl/CH_fact));
      m_last_state->restrictAttribute("bb470", FloatDomain(sp.hydroscat.bb470/BB_fact));
      m_last_state->restrictAttribute("bb676", FloatDomain(sp.hydroscat.bb676/BB_fact));
    }
    postObservation(*m_last_state, false); // silently post the new state obs
    m_last_packet = now;
    
    // TODO: sp also contains gulper info -> process it
    
  } else {
    if( m_last_state->predicate()!=transaction::Predicate::undefined_pred ) {
      syslog(tlog::warn)<<"No sensor update received starting from now.";
      m_last_state = MAKE_SHARED<transaction::Observation>(s_sp_timeline, transaction::Predicate::undefined_pred);
      postObservation(*m_last_state, true); // undefined obs is not silent so we see it in the logs
    } else if( m_last_packet+3 < now ) {
      syslog(tlog::error)<<"Did not receive sensor updates for more than 3 ticks";
      return false;
    }
  }
  
  // TODO process behaviors
  
  return true;
}


void DoradoReactor::handleRequest(transaction::goal_id const &g) {
  
}

void DoradoReactor::handleRecall(transaction::goal_id const &g) {
  
}


void DoradoReactor::started(TREX::transaction::goal_id g) {
  if( g ) {
    if( is_sequential(g->object()) ) {
      m_seq_count += 1;
    } else {
      
    }
  }
}

void DoradoReactor::completed(TREX::transaction::goal_id g) {
  if( g ) {
    if( is_sequential(g->object()) ) {
      // Add lock.Free as an observation that can be overwritten
      // Add this observation the same way
      // process possible pending sequential request
      if( m_seq_count>0 )
        m_seq_count -= 1;
      else
        syslog(tlog::warn)<<"Completion of "<<g->object()<<" resulted on a count of posted sequentials below 0.";
      
    } else {
      
    }
  }
}





//#include <trex/domain/FloatDomain.hh>
//#include <trex/domain/BooleanDomain.hh>
//
//#include <boost/lambda/lambda.hpp>
//
//#include <algorithm>
//#include <cmath>
//
//
//using namespace TREX::mbari;
//using namespace TREX::transaction;
//using namespace TREX::utils;
//
//namespace {
//  TeleoReactor::xml_factory::declare<DoradoReactor> decl("DoradoReactor");
//} // ::
//
//namespace ip=boost::asio::ip;

///*
// * class TREX::mbari::DoradoReactor
// */
//
//// statics
//
//boost::asio::io_service DoradoReactor::s_io_service;
//Symbol const DoradoReactor::s_depth_enveloppe("depthEnveloppe");
//Symbol const DoradoReactor::s_sp_timeline("vehicleState");
//
//IntegerDomain::bound const DoradoReactor::s_temporal_uncertainty(2);
//
//
//// structors 
//
//DoradoReactor::DoradoReactor(TeleoReactor::xml_arg_type arg)
//:TeleoReactor(arg.second, parse_attr<utils::Symbol>(xml_factory::node(arg), "name"),
//              0, 1, parse_attr<bool>(true, xml_factory::node(arg), "log")), 
//m_sp_socket(DoradoReactor::s_io_service),
//m_sp_timer(DoradoReactor::s_io_service),
//m_vcs_socket(DoradoReactor::s_io_service), 
//m_sp_buff(NULL), m_xdr_buff_size(sizeof(uint16_t)+2*sizeof(StatePacket)),
//m_behavior_count(0), m_sequential_count(0), m_sequential_execute(true) {
//  
//  boost::property_tree::ptree::value_type &node(xml_factory::node(arg));
//  // Populate node with external XML config
//  TREX::utils::ext_xml(node.second, "config");
//
//  m_vcs_server.address(ip::address_v4::from_string(parse_attr<std::string>(node, "vcs_host")));
//  m_vcs_server.port(parse_attr<unsigned short>(8004, node, "vcs_port"));
//  syslog(info)<<"VCS address set to TCP : "<<m_vcs_server;
//  
//  syslog(info)<<"Extracting auv timelines.";
//  for(boost::property_tree::ptree::iterator i=node.second.begin();
//      node.second.end()!=i; ++i) {
//    if( is_tag(*i, "Timeline") ) {
//      Symbol name = parse_attr<Symbol>(*i, "name");
//      if( name.empty() )
//        throw XmlError(*i, "Timeline cannot have an empty name");
//      provide(name);
//      boost::optional<Symbol> alias =parse_attr< boost::optional<Symbol> >(*i, "alias");
//      bool sequential = parse_attr<bool>(true, *i, "sequential");
//      if( alias ) {
//        syslog(info)<<"Timeline "<<name<<" aliased to command "<<alias;
//        m_behaviors[name] = std::make_pair(sequential, *alias);
//      } else 
//        m_behaviors[name] = std::make_pair(sequential, name);
//      postObservation(Observation(name, "Inactive"));
//    }
//  }
//  // TODO: create timelines for the gulpers
//  
//  
//  syslog(info)<<"Adding default timelines";
//  provide(s_depth_enveloppe, false);
//  provide(s_sp_timeline, false);
//  
//  bool found;
//  
//  m_init_plan = manager().use(parse_attr<std::string>("auv_init.cfg", 
//                                                      node, "initial_plan"), found);
//  if( !found )
//    throw XmlError(node, 
//                   "Unable to locate initial vcs plan file \""+m_init_plan.string()+"\"");
//  
//  
//  unsigned short my_port = parse_attr<unsigned short>(8002, node, "port");
//  syslog(info)<<"Creating state update UDP server on port "<<my_port;
//  m_sp_socket.open(ip::udp::v4());
//  m_sp_socket.bind(ip::udp::endpoint(ip::udp::v4(), my_port));
//  
//  // Lastly create xdr buffer
//  m_sp_buff = new char[m_xdr_buff_size];
//  m_xdr_buff = m_sp_buff+sizeof(uint16_t);
//  xdrmem_create(&m_xdr_stream, m_xdr_buff, m_xdr_buff_size-sizeof(uint16_t),
//                XDR_DECODE);
//}
//
//DoradoReactor::~DoradoReactor() {
//  if(NULL!=m_sp_buff ) {
//    delete[] m_sp_buff;
//    m_sp_buff = m_xdr_buff = NULL;
//  }
//}
//
//// modifiers 
//
//void DoradoReactor::handleInit() {
//  syslog(info)<<"Extracting initial plan from "<<m_init_plan;
//  std::ifstream plan_file(m_init_plan.c_str());
//  std::ostringstream init_msg;
//  
//  init_msg<<"init|"<<plan_file.rdbuf();
//  plan_file.close();
//  
//  Observation envelope(s_depth_enveloppe, "Active");
//  // Extracting data from depth enveloppe
//  std::string plan = init_msg.str(), args;
//  size_t b_start = plan.find(s_depth_enveloppe.str()), b_arg_start, b_arg_end;
//  b_arg_start = plan.find('{', b_start);
//  b_arg_end = plan.find('}', b_arg_start);
//  
//  // Get the arguments of the depth enveloppe
//  args = plan.substr(b_arg_start+1, b_arg_end-b_arg_start-1);
//  syslog(info)<<"Parsing argument of "<<s_depth_enveloppe;
//  bool end_of_parse = false;
//  while( !end_of_parse ) {
//    size_t eq_pos, sc_pos;
//    
//    eq_pos = args.find('=');
//    if( std::string::npos==eq_pos ) 
//      break;
//    sc_pos = args.find(';', eq_pos);
//    if( std::string::npos==sc_pos ) {
//      sc_pos = args.length();
//      end_of_parse = true;
//    }
//    
//    std::string name(args, 0, eq_pos), val(args, eq_pos+1, sc_pos-eq_pos-1);
//    size_t v_start, v_end;
//    // parse the attribute name
//    v_start = name.find_first_not_of(" \n\t");
//    v_end = name.find_first_of(" \n\t", v_start);
//    if( std::string::npos==v_end )
//      name = name.substr(v_start);
//    else 
//      name = name.substr(v_start, v_end-v_start);
//    // parse the numeric value
//    v_start = val.find_first_of("-0123456789.e");
//    v_end = val.find_first_not_of("-0123456789.e", v_start);
//    if( std::string::npos==v_end )
//      val = val.substr(v_start);
//    else 
//      val = val.substr(v_start, v_end-v_start);
//
//    syslog()<<"  - "<<name<<" = "<<val;
//    envelope.restrictAttribute(name, FloatDomain(string_cast<double>(val)));
//    if( !end_of_parse )
//      args = args.substr(sc_pos+1);
//  }
//  syslog(info)<<"Done parsing";
//  postObservation(envelope);
//  
//  syslog(info)<<"Connecting to vcs server at "<<m_vcs_server;
//  m_vcs_socket.connect(m_vcs_server);
//  syslog(info)<<"Sending initial plan:\n"<<plan;
//  send_string(plan);
//  m_layered_control_ready = false;
//  m_fresh_xdr = false;
//  
//  // Now I need to wait for the vehicle feedback
//  syslog(info)<<"Waiting for state pusblisher connection";
//  while( !get_sp_updates(boost::posix_time::milliseconds(100)) );
//  syslog(info)<<"Waiting for Layered Control to be ready";
//  while( !( m_fresh_xdr && m_layered_control_ready )) 
//    get_sp_updates(boost::posix_time::milliseconds(100));
//  syslog(info)<<"VCS connection established.";
//  // Notify VCS rhat we are starting
//  send_string("start");
//}
//
//
//void DoradoReactor::handleTickStart() {
//}
//
//bool DoradoReactor::synchronize() {
//  if( m_pinged )
//    m_pinged = false;
//  else if( m_sequential_count>0 )
//    send_string("PING");
//  
//  boost::posix_time::milliseconds wait(100);
//  
//  if( get_sp_updates(wait) ) {
//    wait = boost::posix_time::milliseconds(10);
//    while( get_sp_updates(wait) );
//  }
//  if( m_fresh_xdr ) {
//    StatePacket state;
//    
//    xdr_setpos(&m_xdr_stream, 0);
//    bool decoded = xdr_StatePacket(&m_xdr_stream, &state);
//    if( !decoded ) {
//      syslog(error)<<"Failed to decode last state packet";
//      return false;
//    }
//    
//    Observation obs(s_sp_timeline, "Holds");
//    
//    // Position
//    obs.restrictAttribute("x", FloatDomain(state.position.x));
//    obs.restrictAttribute("y", FloatDomain(state.position.y));
//    obs.restrictAttribute("z", FloatDomain(state.position.z));
//    
//    // cluster
//    obs.restrictAttribute("cluster_id", IntegerDomain(state.cluster.id));
//    
//    // CTD
//    obs.restrictAttribute("ctdValid", BooleanDomain(state.ctd.valid));
//    if( state.ctd.valid ) {
//      obs.restrictAttribute("conductivity", FloatDomain(state.ctd.c));
//      obs.restrictAttribute("temperature", FloatDomain(state.ctd.t));
//      obs.restrictAttribute("density", FloatDomain(state.ctd.d));
//      obs.restrictAttribute("salinity", FloatDomain(state.ctd.salinity));
//    }
//    
//    // Isus
//    obs.restrictAttribute("isusValid", BooleanDomain(state.isus.valid));
//    if( state.isus.valid )
//      obs.restrictAttribute("nitrate", FloatDomain(state.isus.nitrate));
//    
//    // HS2
//    obs.restrictAttribute("hydroscatValid", BooleanDomain(state.hydroscat.valid));
//    if( state.hydroscat.valid ) {
//      obs.restrictAttribute("bb470", FloatDomain(state.hydroscat.bb470));
//      obs.restrictAttribute("bb676", FloatDomain(state.hydroscat.bb676));
//      obs.restrictAttribute("ch_fl", FloatDomain(state.hydroscat.fl));
//    }
//    postObservation(obs);
//    m_fresh_xdr = false;
//    // TODO handle gulpers
//    
//  } else {
//    syslog(warn)<<"No state update received from VCS";
//  }
//  std::map<utils::Symbol, std::list<Observation> >::iterator o = m_queued_obs.begin();
//  
//  for( ; m_queued_obs.end()!=o; ++o) {
//    if( !o->second.empty() ) {
//      postObservation(o->second.front(), true);
//      o->second.pop_front();
//    }
//  }
//  
//  return true;
//}
//
//void DoradoReactor::handleRequest(goal_id const &g) {
//  std::map< utils::Symbol, 
//            std::pair<bool, 
//                      utils::Symbol> >::const_iterator cmd = m_behaviors.find(g->object());
//  if( m_behaviors.end()!=cmd ) {
//    if( g->predicate()!="Inactive" ) {
//      if( cmd->second.first ) {
//        m_pending.push_back(g);
//        process_pendings();
//      } else 
//        send_request(g);
//    }
//  }
//}
//
//void DoradoReactor::handleRecall(goal_id const &g) {
//  // Check if the goal is already sent
//  std::map< int32_t, std::pair<goal_id, bool> >::iterator i=m_sent_cmd.begin();
//  
//  for( ; m_sent_cmd.end()!=i; ++i) {
//    if( g==i->second.first ) {
//      std::ostringstream oss;
//      
//      oss<<"delete|"<<i->first;
//      syslog(info)<<"Sending "<<oss.str();
//      send_string(oss.str());
//      m_pinged = true;
//      return;
//    }
//  }
//  
//  // Not sent : check if its still on the pending list 
//  for(std::list<goal_id>::iterator p=m_pending.begin(); m_pending.end()!=p; ++p) {
//    if( g==*p ) {
//      m_pending.erase(p);
//      return;
//    }
//  }
//}
//
//void DoradoReactor::send_string(std::string const &msg) {
//  std::vector<boost::asio::const_buffer> buffs; 
//
//  // identify the length of the string in net format
//  uint32_t len = msg.length(), net_len = htonl(len);
// 
//  // build message <len><content>
//  buffs.push_back(boost::asio::buffer(&net_len, sizeof(uint32_t)));
//  buffs.push_back(boost::asio::buffer(msg));
// 
//  // Send the message
//  boost::asio::write(m_vcs_socket, buffs);
//  m_pinged = true;
//}
//
//void DoradoReactor::send_request(transaction::goal_id const &g) {
//  IntegerDomain::bound time_out = g->getDuration().upperBound();
//  std::ostringstream oss;
//  
//  // Identifies the maximum duration to be sent
//  if( !g->getDuration().isSingleton() ) {
//    IntegerDomain::bound late_duration = g->getEnd().upperBound(),
//      min_duration = g->getDuration().lowerBound();
//    
//    late_duration.minus(g->getStart().upperBound(), time_out);
//    min_duration.plus(s_temporal_uncertainty, min_duration);
//    time_out.minus(s_temporal_uncertainty, time_out);
//    
//    if( late_duration>=min_duration && late_duration<time_out ) {
//      time_out = late_duration;
//      time_out.minus(s_temporal_uncertainty, time_out);
//    } 
//  }  
//  int32_t id = next_id();
//  
//  oss.setf(std::ios::fixed);
//  oss<<"insert|behavior "<<m_behaviors[g->object()].second<<" { id="<<id<<"; ";
//  if( !time_out.isInfinity() ) {
//    // Convert the duration from tick into seconds
//    duration_type dur = time_out.value()*tickDuration();
//    
//    oss<<"duration="
//       <<boost::chrono::duration_cast<boost::chrono::seconds>(dur)<<"; ";
//  } else
//    syslog(warn)<<"Goal "<<g<<" has no maximum duration.";
//  
//  for(Predicate::const_iterator v=g->begin(); g->end()!=v; ++v) {
//    Variable const &var = v->second;
//    
//    if( var.isComplete() && var.domain().isSingleton() )
//      oss<<var.name()<<'='<<var.domain().getStringSingleton()<<"; ";
//  }
//  
//  oss<<"}";
//  syslog(info)<<"Sending "<<oss.str(); 
//  send_string(oss.str());
//  m_sent_cmd[id] = std::make_pair(g, false);
//  ++m_behavior_count;
//}
//
//
//bool DoradoReactor::get_sp_updates(boost::posix_time::milliseconds const &timeout) {
//  boost::array<char, 3> header;
//  bool timed_out = false;
//  boost::system::error_code ec(boost::asio::error::would_block);
//  boost::asio::ip::udp::endpoint from;
//  
//  
//  // Change ec value on execution :
//  m_sp_socket.async_receive_from(boost::asio::buffer(header), 
//                                 from, 
//                                 boost::lambda::var(ec) = boost::lambda::_1);
//  m_sp_timer.expires_from_now(timeout);
//  // Set timed_out to true on time out 
//  m_sp_timer.async_wait(boost::lambda::var(timed_out) = true);
//  
//  do {
//    // run either of the two async calls
//    s_io_service.run_one();
//    if( timed_out ) {
//      // looks like we timed out => cancel the socket reception
//      m_sp_socket.cancel();
//      return false;
//    }
//  } while( boost::asio::error::would_block==ec );
//  // The socket did execute before timing out => cancel the timer
//  m_sp_timer.cancel();
//  // Do things here 
//  
//  if( ec ) {
//    // We had a socket error => report it
//    syslog(error)<<"StatePacket socket error: "<<ec;
//    throw ec;
//  } 
//  syslog()<<"New message from "<<from;
//  
//  switch( header[0] ) {
//    case 'S':
//      get_statepacket(ntohs(*reinterpret_cast<uint16_t *>(header.data()+1)));
//      break;
//      
//    case 'B':
//      // Get behavior information
//      get_behavior();
//      break;
//      
//    default:
//      syslog(error)<<"Received message from "<<from<<" with unknown header type '"
//      <<header[0]<<"'(0x"<<std::hex<<static_cast<short>(header[0])<<").";
//      throw ReactorException(*this, "Invalid StatePublisher header received.");
//  }
//  
//  return true;
//}
//
//void DoradoReactor::get_behavior() {
//  char     ignore, event_type;
//  uint32_t net_id;
//  boost::array<boost::asio::mutable_buffer, 3> msg = {
//    boost::asio::buffer(&ignore, 1),
//    boost::asio::buffer(&event_type, 1),
//    boost::asio::buffer(&net_id, 1) };
//  boost::asio::ip::udp::endpoint from;
//
//  m_sp_socket.receive_from(msg, from);
//  int32_t behavior_id = -1;
//  
//  *reinterpret_cast<uint32_t *>(behavior_id) = ntohs(net_id);
//  if( -1==behavior_id && 'A'==event_type ) {
//    syslog(warn)<<"Abort received from VCS at "<<from;
//    throw ReactorException(*this, "Mission aborted by VCS");
//  }
//  if( 0==behavior_id && !m_layered_control_ready ) {
//    if( 'F'==event_type ) {
//      syslog()<<"Behavior with ID "<<behavior_id<<" did complete => LC ready";
//      m_layered_control_ready = true;
//      m_id = 0;
//    }
//  } else {
//    std::map< int32_t, std::pair<goal_id, bool> >::iterator 
//      i = m_sent_cmd.find(behavior_id);
//    
//    if( m_sent_cmd.end()==i ) {
//      syslog(warn)<<"Ignoring event from unknown behavior id "<<behavior_id;
//    } else if( 'F' == event_type ) {
//      // Behavior completed 
//      syslog(info)<<i->second.first->object()<<"[id="<<behavior_id<<"] FINISHED.";
//      if( !i->second.second ) {
//        syslog(warn)<<"behavior "<<behavior_id<<" completed before being started ...";
//        queue_obs(*(i->second.first));
//      }
//      queue_obs(Observation(i->second.first->object(), "Inactive"));
//      m_behavior_count = std::max(1ul, m_behavior_count)-1;
//      if( m_behaviors[i->second.first->object()].first ) {
//        m_sequential_count = std::max(1ul, m_sequential_count)-1;
//        m_sequential_execute = (0==m_sequential_count);
//        process_pendings();
//      }
//      m_sent_cmd.erase(i);
//    } else if( 'S' == event_type ) {
//      if( !i->second.second ) {
//        syslog(info)<<i->second.first->object()<<"[id="<<behavior_id<<"] STARTED.";
//        queue_obs(*(i->second.first));
//        if( m_behaviors[i->second.first->object()].first ) {
//          m_sequential_execute = (1==m_sequential_count);
//          process_pendings();
//        }
//        i->second.second = true;
//      } else {
//        syslog(warn)<<"Ignoring multiple starts for behavior "<<behavior_id;
//      }
//    } else {
//      syslog(warn)<<"Ignoring unknown behavior event '"<<event_type<<"'(0x"
//      <<std::hex<<static_cast<short>(event_type)<<std::dec<<") for behavior "
//      <<behavior_id;
//    }
//  }
//}
//
//void DoradoReactor::get_statepacket(uint16_t size) {
//  static bool first = true;
//  
//  if( first ) {
//    first = false;
//    if( size+3 < static_cast<uint16_t>(m_xdr_buff_size) )
//      m_xdr_buff_size = size+3;
//  }
//  if( size+3 != static_cast<uint16_t>(m_xdr_buff_size) ) {
//    syslog(error)<<"State packet header is inconsitent with previous size value ( "
//    <<size+3<<"!="<<m_xdr_buff_size<<").";
//    throw ReactorException(*this, "Spurious state packet update.");
//  }
//  boost::asio::ip::udp::endpoint from;
//
//  size_t len = m_sp_socket.receive_from(boost::asio::buffer(m_sp_buff, 
//                                                            m_xdr_buff_size), 
//                                        from);
//  if( len!=m_xdr_buff_size ) {
//    syslog(error)<<"Data received ("<<len<<") differs from expected ("<<m_xdr_buff_size<<")";
//    throw ReactorException(*this, "StatePacket reception error"); 
//  }
//  m_fresh_xdr = true;
//}
//
//size_t DoradoReactor::next_id() {
//  return ++m_id;
//}
//
//void DoradoReactor::queue_obs(Observation const &obs) {
//  std::list<Observation> &events = m_queued_obs[obs.object()];
//  
//  if( !events.empty() ) {
//    if( events.back().predicate()=="Inactive" )
//      events.pop_back();
//  }
//  events.push_back(obs);
//}
//
//void DoradoReactor::process_pendings() {
//  if( m_sequential_execute || 0==m_sequential_count ) {
//    if( !m_pending.empty() ) {
//      goal_id next(m_pending.front());
//      m_pending.pop_front();
//      // TODO: Check if this goal is executable (ie can be started on next tick) 
//      m_sequential_count += 1;
//      m_sequential_execute = false;
//      send_request(next);
//    }
//  }
//}
//
//
//

