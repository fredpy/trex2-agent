#include <arpa/inet.h>
#include <boost/array.hpp>

#include <trex/utils/Plugin.hh>
#include <trex/utils/LogManager.hh>
#include "Vitre.hh"

using namespace TREX::utils;
using namespace TREX::transaction;
using namespace TREX::vitre;

namespace {

  SingletonUse<LogManager> s_log;

  TeleoReactor::xml_factory::declare<VitreReactor> decl("VitreReactor");
  
}

namespace TREX {
  
  /** @brief Plug-in initialisation
   *
   * This function is called by TREX after loading the lighswitch plug-in.
   * It manage the initialisation of this plug-in
   *
   * @ingroup lightswitch
   */
  void initPlugin() {
    ::s_log->syslog("plugin.vitre")<<"Vitre loaded."<<std::endl;
    // ::decl;
  }

  boost::asio::io_service &get_io_service() {
    static boost::asio::io_service serv;
    return serv;
  }
  
} // TREX

VitreReactor::VitreReactor(TeleoReactor::xml_arg_type arg)
  :TeleoReactor(arg),
   m_host(parse_attr<std::string>("127.0.0.1", // put IP instead of localhost to avoid DNS issues
                                  TeleoReactor::xml_factory::node(arg),
                                  "addr")),
   m_port(parse_attr<std::string>(TeleoReactor::xml_factory::node(arg),
                                  "port")),
   m_socket(get_io_service()) {
  syslog()<<"Will try to connect to "<<m_host<<':'<<m_port;
}

VitreReactor::~VitreReactor() {
  if( m_socket.is_open() ) 
    m_socket.close();
}

void VitreReactor::handleInit() {
  boost::asio::ip::tcp::resolver resolver(get_io_service());
  boost::asio::ip::tcp::resolver::query 
    query(boost::asio::ip::tcp::v4(), m_host, m_port);
  boost::asio::ip::tcp::resolver::iterator 
    iterator = resolver.resolve(query),end;
  boost::system::error_code error = boost::asio::error::host_not_found;

  while (error && iterator != end) {
    m_socket.close();
    m_socket.connect(*iterator++, error);
  }
  if ( !error )
    syslog()<<"Connection to "<<m_host<<':'<<m_port<<" succeeded.";
}

void VitreReactor::send(std::string const &str) {
  uint32_t msg_len = htonl(str.length());
  boost::array<boost::asio::const_buffer, 2> msg = {
      {boost::asio::buffer(&msg_len, sizeof(msg_len)),
      boost::asio::buffer(str)} };
  try {
    m_socket.send(msg);
  } catch( std::exception &e) {
    syslog()<<" message failure : "<<e.what();
    m_socket.close();
  }
}


void VitreReactor::handleTickStart() {
  if( !m_socket.is_open() ) {
    handleInit();
  }
}

void VitreReactor::notify(Observation const &obs) {
  if( m_socket.is_open() ) {
    std::ostringstream oss;
    oss<<"<Token tick=\""<<getCurrentTick()<<"\" on=\""
       <<obs.object()<<"\" pred=\""<<obs.predicate()<<"\">"
       <<obs<<"</Token>";
    send(oss.str());
  }
}
   
bool VitreReactor::synchronize() {
  if( m_socket.is_open() ) {
    std::ostringstream oss;
    oss<<"<Tick value=\""<<(getCurrentTick()+1)<<"\"/>";
    send(oss.str());
  }
  return true;
}

