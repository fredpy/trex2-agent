#include <trex/domain/IntegerDomain.hh>
#include <trex/domain/FloatDomain.hh>

#include <rapidxml/rapidxml.hpp>

#include "DrifterTracker.hh"

#include "mbari/tracker/platformMessage.pb.h"

using namespace mbari;
using namespace TREX::transaction;

namespace {

  /** @brief TREX log entry point */
  TREX::utils::SingletonUse<TREX::utils::LogManager> s_log;

  /** @brief Light reactor declaration */
  TeleoReactor::xml_factory::declare<DrifterTracker> decl("DrifterTracker");  
}

namespace TREX {
  
  void initPlugin() {
    ::s_log->syslog("plugin.tracker")<<"MBARI tracker loaded."<<std::endl;
    // ::decl;
  }

} // TREX


DrifterTracker::DrifterTracker(TeleoReactor::xml_arg_type arg) 
  :TeleoReactor(arg, false) {
  m_connection.open("messaging.shore.mbari.org", 5672);
  m_connection.login("tracking", "MBARItracking", "trackingvhost");

  m_queue = m_connection.create_queue(getName().str());
  provide("MessagesFromTrex");
  if( isInternal("MessagesFromTrex") ) {    
    // Bind to TREX messages
    m_queue->bind("MessagesFromTrex", "SensorApp");
  } else 
    syslog("WARN")<<"MessagesFromTrex timeline already declared";
  
  rapidxml::xml_node<> &node(xml_factory::node(arg));
  TREX::utils::Symbol tl_name;
  std::string exch_name;
  std::set<std::string> exchs;

  // Now look if I have some asset I need to track 
  for(TREX::utils::ext_iterator iter(node, "config"); iter.valid(); ++iter) {
    if( TREX::utils::is_tag(*iter, "Track") ) {
      // First we try to declare it 
      tl_name = TREX::utils::parse_attr<TREX::utils::Symbol>(*iter, "name");
      exch_name = TREX::utils::parse_attr<std::string>(*iter, "exchange");
      if( tl_name.empty() )
	throw TREX::utils::XmlError(*iter, 
				    "Unable to find the name attribute to Track");
      if( exch_name.empty() )
	throw TREX::utils::XmlError(*iter, 
				    "Unable to find the exchange attribute to Track");
      else 
	exch_name = exch_name+"_pb";
      if( isInternal(tl_name) )
	syslog("WARN")<<"I have already declared "<<tl_name<<" as Internal.";
      else { 
	provide(tl_name);
	if( isInternal(tl_name) ) {
	  if( exchs.insert(exch_name).second ) {
	    m_queue->bind(exch_name, "");
	  }	  
	  m_drifters.insert(tl_name);
	} else 
	  syslog("WARN")<<"Failed to declare "<<tl_name<<" as Internal."
			<<"\n\tI will not track this asset.";
      }
    }
  }
  m_listener.reset(new amqp::listener(m_queue, m_messages));
}

DrifterTracker::~DrifterTracker() {}

void DrifterTracker::handleInit() {
  // It is time to start my thread 
  syslog()<<"Starting the amqp queue listener.";
  m_thread.reset(new boost::thread(*m_listener));
}

bool DrifterTracker::synchronize() {
  while( !m_messages.empty() ) {
    boost::shared_ptr<amqp::queue::message> msg = m_messages.pop();
    if( msg->exchange()=="MessagesFromTrex" ) {
      try {
	postObservation(trex_msg(*msg));
      } catch(...) {}
    } else 
      drifter_msg(*msg);
  }
  return true;
}
   

void DrifterTracker::drifter_msg(amqp::queue::message const &msg) {
  MBARItracking::PlatformReport report;
  
  if( report.ParseFromArray(msg.body(), msg.size()) ) {
    if( report.has_name() ) {
      TREX::utils::Symbol name(report.name());
      if( m_drifters.end()!=m_drifters.find(name) ) {
	// build the Observation
	Observation obs(name, "Msg");
	if( report.has_epoch_seconds() ) {
	  time_t sec = report.epoch_seconds();
	  suseconds_t usec = 0;
	  TICK date = timeToTick(sec, usec);
	  obs.restrictAttribute("tick", IntegerDomain(date));
	}
	if( report.has_latitude() ) 
	  obs.restrictAttribute("latitude", FloatDomain(report.latitude()));
	if( report.has_longitude() ) 
	  obs.restrictAttribute("longitude", FloatDomain(report.longitude()));
	postObservation(obs);
      }
    }
  } 
}
  
Observation DrifterTracker::trex_msg(amqp::queue::message const &msg) {
  rapidxml::xml_document<> doc;
  std::string txt(static_cast<char const *>(msg.body()), msg.size());
  doc.parse<0>(const_cast<char *>(txt.c_str()));
  rapidxml::xml_node<> *root = doc.first_node();
  Observation obs("MessagesFromTrex", root->name());
  time_t sec = TREX::utils::parse_attr<time_t>(*root, "utime"); 
  suseconds_t usec = 0;
  TICK date = timeToTick(sec, usec);
  obs.restrictAttribute("tick", IntegerDomain(date));
  return obs;
}
