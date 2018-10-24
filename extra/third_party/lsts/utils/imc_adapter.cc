#include <DUNE/IMC/Header.hpp>
#include <DUNE/IMC/IridiumMessageDefinitions.hpp>
#include <DUNE/Network/Fragments.hpp>
#include <DUNE/IMC/Definitions.hpp>

// dune is defineing likely as a Macro which messes up with some boost libraries (also likely is to become a c++ 20 keyword ...)
#undef likely

#include "imc_adapter.hh"

#include <trex/domain/BooleanDomain.hh>
#include <trex/domain/EnumDomain.hh>
#include <trex/domain/StringDomain.hh>
#include <trex/domain/FloatDomain.hh>
#include <trex/transaction/Goal.hh>

using namespace trex_lsts;

using namespace TREX::utils;
using TREX::transaction::graph;

namespace {
  
  class graph_proxy :public trex_proxy {
  public:
    graph_proxy(graph &g):m_graph(g) {}
    virtual ~graph_proxy() {}
    
    tick_type current_tick() const {
      return m_graph.getCurrentTick();
    }
    date_type tick_to_date(tick_type t) const {
      return m_graph.tickToTime(t);
    }
    tick_type date_to_tick(date_type const &d) const {
      return m_graph.timeToTick(d);
    }
    
    std::string date_str(tick_type t) const {
      return m_graph.date_str(t);
    }
    std::string duration_str(tick_type t) const {
      return m_graph.duration_str(t);
    }

    tick_type as_date(std::string const &s) const {
      return m_graph.as_date(s);
    }
    tick_type as_duration(std::string const &s) const {
      return m_graph.as_duration(s);
    }
    
    boost::asio::io_service &service() {
      return m_graph.strand().get_io_service();
    }
    TREX::utils::log::stream log(Symbol const &type) {
      return m_graph.syslog("imc", type);
    }

  private:
    graph &m_graph;
  };
  
}

/*
 * class trex_lsts::imc_adapter
 */

imc_adapter::imc_adapter():m_trex_id(65000) {
}

imc_adapter::~imc_adapter() {
}


bool imc_adapter::is_connected() const {
  return NULL!=m_listen.get();
}


void imc_adapter::set_graph(graph &g) {
  m_proxy = MAKE_SHARED<graph_proxy>(boost::ref(g));
  m_proxy->log(log::info)<<"IMC adapter connected to agent "<<g.getName();
}


void imc_adapter::init_connection(int local_port, int id, std::string const &remote_addr, int remote_port) {
  if( !m_proxy )
    throw Exception("Attempted to initialize connection before setting agent");
  
  if( is_connected() )
    throw Exception("Connection can only be initialized once");
  
  m_trex_id = id;
  m_proxy->log(log::info)<<"Agent dune ID set to "<<m_trex_id;
  
  m_dune_ip = remote_addr;
  m_dune_port = remote_port;
  m_proxy->log(log::info)<<"Dune host set to "<<m_dune_ip<<':'<<m_dune_port;
  m_send.reset(new strand(m_proxy->service()));
    
  m_local_port = local_port;
  m_receiver.bind(m_local_port, DUNE::Network::Address::Any, true);
  m_poll.add(m_receiver);
  m_proxy->log(log::info)<<"Listening to Dune on port "<<m_local_port;
  m_listen.reset(new strand(m_proxy->service()));
  
  m_listen->post(boost::bind(&imc_adapter::async_poll, this));
}


bool imc_adapter::send(message &m, std::string const &host, int port) {
  if( NULL==m_send.get() )
    return false;
  else {
    SHARED_PTR<message> msg(m.clone());
    m_send->post(boost::bind(&imc_adapter::async_send, this, msg, host, port));
    return true;
  }
}


bool imc_adapter::send(message &m) {
  return send(m, m_dune_ip, m_dune_port);
}

void imc_adapter::async_poll() {
  static uint8_t buffer[65535];
  
  if( m_poll.poll(m_receiver, 100) ) {
    DUNE::Network::Address addr;
    uint16_t rcv = m_receiver.read(buffer, 65535, &addr);
    SHARED_PTR<message> msg(DUNE::IMC::Packet::deserialize(buffer, rcv));
    m_rcv(msg); // emit the message
  }
  m_listen->post(boost::bind(&imc_adapter::async_poll, this));
}

void imc_adapter::async_send(SHARED_PTR<message> m, std::string host, int port) {
  try {
    DUNE::Utils::ByteBuffer bb;
    
    DUNE::IMC::Packet::serialize(m.get(), bb);
    m_sender.write(bb.getBuffer(), bb.getSize(),
                   DUNE::Network::Address(host.c_str()), port);
  } catch(std::runtime_error const &e) {
    m_proxy->log(log::error)<<"Error while sending message to Dune: "<<e.what();
  }
}



//// structors
//
//imc_adapter::imc_adapter():c_imc_header_length(sizeof(DUNE::IMC::Header)), c_max_iridium_payload_length(260),
//m_trex_id(65000), m_platform_id(0), m_iridium_req(0) {
//
//}
//
//
//// manipulator
//
//void imc_adapter::set_graph(TREX::transaction::graph &g) {
//  m_proxy = MAKE_SHARED<graph_proxy>(g);
//}
//
//void imc_adapter::set_proxy(trex_proxy *p) {
//  m_proxy.reset(p);
//}
//
//void imc_adapter::set_trex_id(int id) {
//  m_trex_id = id;
//}
//
//void imc_adapter::set_platform_id(int id) {
//  m_platform_id = id;
//}
//
//bool imc_adapter::bind(int port) {
//  m_messenger.stop_listen();
//  m_messenger.start_listen(port);
//  return true;
//}
//
//bool imc_adapter::unbind() {
//  m_messenger.stop_listen();
//  return true;
//}
//
//SHARED_PTR<imc_adapter::message> imc_adapter::poll() {
//  SHARED_PTR<message> ret(m_messenger.receive());
//  return ret;
//}
//
//bool imc_adapter::send(message &m, int port, std::string const &host) {
//  if( m.getTimeStamp()==0 )
//    m.setTimeStamp();
//  if( m.getSource()<=0 || m.getSource()==65535 )
//    m.setSource(m_trex_id);
//  m_messenger.post(m, port, host);
//  return true;
//}
//
//int imc_adapter::iridium_req() {
//  return (++m_iridium_req) % 65535;
//}
//
//bool imc_adapter::send_via_iridium(message &m, int port, std::string const &host) {
//  if( m.getTimeStamp()==0 )
//    m.setTimeStamp();
//
//  DUNE::IMC::ImcIridiumMessage ir_msg(m.clone());
//  uint8_t buff[65535];
//
//  ir_msg.destination = m.getDestination();
//  if( m.getSource()<=0 || m.getSource()==65535 ) {
//    ir_msg.source = m_platform_id;
//    m.setSource(m_platform_id);
//  } else
//    ir_msg.source = m.getSource();
//
//  int len = ir_msg.serialize(buff);
//
//  if( c_max_iridium_payload_length < len ) {
//    // need to split message into smaller bits
//    DUNE::Network::Fragments frags(&m, c_max_iridium_payload_length);
//    proxy()->log(TREX::transaction::info)<<" Splitting Iridium message into "<<frags.getNumberOfFragments()<<" fragments";
//
//    for(int i=0; i<frags.getNumberOfFragments(); ++i) {
//      DUNE::IMC::ImcIridiumMessage ir_frag(frags.getFragment(i));
//
//      ir_frag.destination = m.getDestination();
//      ir_frag.source = m.getSource();
//      len = ir_frag.serialize(buff);
//
//      DUNE::IMC::IridiumMsgTx tx;
//      tx.ttl = 1800; // set message sending timeout to 30mn
//      tx.data.assign(buff, buff+len);
//      tx.setTimeStamp();
//      tx.req_id = iridium_req();
//      if( !send(tx, port, host) )
//        return false;
//    }
//    return true;
//  } else {
//    DUNE::IMC::IridiumMsgTx tx;
//    tx.setSource(m.getSource());
//    tx.setTimeStamp();
//    tx.ttl = 1800;
//    tx.data.assign(buff, buff+len);
//    tx.req_id = iridium_req();
//    return send(tx, port, host);
//  }
//}
//
//boost::optional<imc_adapter::imc_var> imc_adapter::variable_to_imc(trex_var const &var) const {
//  Symbol type = var.domain().getTypeName();
//  imc_var ret;
//
//  ret.name = var.name().str();
//
//  if( var.name()==Goal::s_startName || var.name()==Goal::s_endName ) {
//    // Attribute describe a date
//    ret.attr_type = imc_var::TYPE_STRING;
//    IntegerDomain const &dom_i = dynamic_cast<IntegerDomain const &>(var.domain());
//
//    if( dom_i.isSingleton() ) {
//      std::string s = m_proxy->date_str(dom_i.lowerBound());
//      ret.min = s;
//      ret.max = s;
//    } else {
//      if( dom_i.hasUpper() )
//        ret.max = m_proxy->date_str(dom_i.upperBound());
//      if( dom_i.hasLower() )
//        ret.min = m_proxy->date_str(dom_i.lowerBound());
//    }
//  } else if( var.name()==Goal::s_durationName ) {
//    // Attribute is a duration
//    ret.attr_type = imc_var::TYPE_STRING;
//    IntegerDomain const &dom_i = dynamic_cast<IntegerDomain const &>(var.domain());
//    if( dom_i.isSingleton() ) {
//      std::string s = m_proxy->duration_str(dom_i.lowerBound());
//      ret.min = s;
//      ret.max = s;
//    } else {
//      if( dom_i.hasUpper() )
//        ret.max = m_proxy->duration_str(dom_i.upperBound());
//      if( dom_i.hasLower() )
//        ret.min = m_proxy->duration_str(dom_i.lowerBound());
//    }
//  } else {
//    if( type==FloatDomain::type_name )
//      ret.attr_type = imc_var::TYPE_FLOAT;
//    else if( type==IntegerDomain::type_name )
//      ret.attr_type = imc_var::TYPE_INT;
//    else if( type==BooleanDomain::type_name )
//      ret.attr_type = imc_var::TYPE_BOOL;
//    else if( type==StringDomain::type_name )
//      ret.attr_type = imc_var::TYPE_STRING;
//    else if( type==EnumDomain::type_name )
//      ret.attr_type = imc_var::TYPE_ENUM;
//    else {
//      if( var.domain().isEnumerated() ) {
//        m_proxy->log(TREX::transaction::warn)<<"Unknown enum type \""<<type<<"\" for attribute "<<var.name();
//        ret.attr_type = imc_var::TYPE_ENUM;
//      } else {
//        m_proxy->log(TREX::transaction::error)<<"Unknown type \""<<type<<"\" for attribute "<<var.name();
//        return boost::optional<imc_var>();
//      }
//    }
//    if( var.domain().isSingleton() ) {
//      ret.max = var.domain().getStringSingleton();
//      ret.min = ret.max;
//    } else {
//      // would be better to create the interval but previous implementation was like this
//      ret.min = "";
//      ret.max = "";
//    }
//  }
//  return ret;
//}
//
//imc_adapter::imc_token imc_adapter::predicate_to_imc(trex_pred const &pred, trex_proxy::tick_type date) const {
//  imc_token ret;
//  bool duration_full = true, end_implied = true;
//
//  ret.timeline = pred.object().str();
//  ret.predicate = pred.predicate().str();
//
//  std::list<Symbol> attrs;
//  pred.listAttributes(attrs);
//
//  if( pred.has_temporal_scope() ) {
//    IntegerDomain duration(Goal::s_durationDomain);
//
//    // Check if duration is just the default [1, +inf)
//    duration.restrictWith(pred.getAttribute(Goal::s_durationName).domain());
//    duration_full = duration.equals(Goal::s_durationDomain);
//
//    // Check if end can be directly deuced by start+duration
//    IntegerDomain start(Goal::s_dateDomain);
//
//    start.restrictWith(pred.getAttribute(Goal::s_startName).domain());
//    // create a dummy goal with same start and duration as pred but completely free end
//    Goal test("foo", "bar");
//    test.restrictTime(start, duration, Goal::s_dateDomain);
//    // check if resulting end is same as pred end
//    end_implied = test.getEnd().equals(pred.getAttribute(Goal::s_endName).domain());
//  }
//
//  for(std::list<Symbol>::const_iterator i=attrs.begin(); attrs.end()!=i; ++i) {
//    Variable const &v = pred.getAttribute(*i);
//    bool full;
//
//    if( v.name()==Goal::s_durationName )
//      full = duration_full;
//    else if( v.name()==Goal::s_endName )
//      full = end_implied;
//    else
//      full = v.domain().isFull();
//    // No need to add domain which are not constrained
//    if( !full ) {
//      boost::optional<imc_var> a = variable_to_imc(v);
//      if( a.is_initialized() )
//        ret.attributes.push_back(*a);
//    }
//  }
//  // compute duration since POSIX epoch
//  boost::posix_time::time_duration time = m_proxy->tick_to_date(date)-boost::posix_time::from_time_t(0);
//
//  long double delta_sec = time.total_milliseconds();
//  delta_sec /= 1000.0; // seconds since posix epoch (ie typical POSIX time represenation)
//  ret.setTimeStamp(static_cast<double>(delta_sec));
//  ret.setSource(m_trex_id);
//  return ret;
//}
