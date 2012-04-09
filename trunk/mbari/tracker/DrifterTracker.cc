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

  syslog("amqp")<<"Creating queue \"trex2"<<getName()<<std::endl;
  m_queue = m_connection.create_queue("trex2"+getName().str());
  provide("MessagesFromTrex");
  if( isInternal("MessagesFromTrex") ) {    
    // Bind to TREX messages
    syslog("amqp")<<"Binding to TREX messages (MessagesFromTrex/TrackingApp)";
    m_queue->bind("MessagesFromTrex", "TrackingApp");
  } else 
    syslog("WARN")<<"MessagesFromTrex timeline already declared";
  
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
    m_queue->bind(handler->exchange(), "");
    // add the handler
    m_handlers.insert(std::make_pair(handler->exchange(), handler));
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

bool DrifterTracker::synchronize() {
  handle_map::const_iterator from, to;


  while( !m_messages.empty() ) {
    boost::shared_ptr<amqp::queue::message> msg = m_messages.pop();
    syslog()<<"New message["<<msg->key()<<"]: "<<msg->size()<<" bytes from \""
            <<msg->exchange()<<"\"";

    if( msg->exchange()=="MessagesFromTrex" ) {
      try {
	trex_msg(*msg);
      } catch(...) {}
    } 

    boost::tie(from, to) = m_handlers.equal_range(msg->exchange());
    for( ; to!=from; ++from) {
      if( from->second->handleMessage(*msg) )
	syslog()<<"AMQP message "<<msg->key()<<" handled.";
    }
  }

  for(from=m_handlers.begin(); m_handlers.end()!=from; ++from) 
    if( !from->second->synchronize() ) {
      syslog("ERROR")<<" Handler on exchange "<<from->second<<" failed to synchronize";
      return false;
    }
  return true;
}
  
void DrifterTracker::trex_msg(amqp::queue::message const &msg) {
   std::string txt(static_cast<char const *>(msg.body()), msg.size());
   syslog("amqp")<<"Parsing TREX message : \""<<txt<<"\".";

  /* Former code when I was expecting to recive what TREX actually sent */
  // rapidxml::xml_document<> doc;
  // doc.parse<0>(const_cast<char *>(txt.c_str()));
  // rapidxml::xml_node<> *root = doc.first_node();
  // Observation obs("MessagesFromTrex", root->name());
  // time_t sec = TREX::utils::parse_attr<time_t>(*root, "utime"); 
  // obs.restrictAttribute("tick", IntegerDomain(timeToTick(sec)));
  /*
    string str1("hello abc-*-ABC-*-aBc goodbye");

    typedef vector< iterator_range<string::iterator> > find_vector_type;
    
    find_vector_type FindVec; // #1: Search for separators
    ifind_all( FindVec, str1, "abc" ); // FindVec == { [abc],[ABC],[aBc] }

    typedef vector< string > split_vector_type;
    
    split_vector_type SplitVec; // #2: Search for tokens
    split( SplitVec, str1, is_any_of("-*"), token_compress_on ); // SplitVec == { "hello abc","ABC","aBc goodbye" }
  */
   std::vector< std::string > tokens;
   boost::split(tokens, txt, boost::is_any_of(","), boost::token_compress_on);
   if( tokens.size()==3 ) { 
     Observation obs("MessagesFromTrex", tokens[0]);
     obs.restrictAttribute("tick", 
			   IntegerDomain(timeToTick(TREX::utils::string_cast<time_t>(tokens[1]))));
     obs.restrictAttribute("imei", 
			   IntegerDomain(TREX::utils::string_cast<unsigned long>(tokens[2])));
     postObservation(obs);
   }
}
