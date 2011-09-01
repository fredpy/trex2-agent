#include <set>

#include <trex/domain/IntegerDomain.hh>
#include <trex/domain/FloatDomain.hh>

#include <rapidxml/rapidxml.hpp>
#include <mbari/shared/GeoUTM.hh>

#include "DrifterTracker.hh"

#include "platformMessage.pb.h"

using namespace mbari;
using namespace TREX::transaction;

namespace {

  /** @brief Light reactor declaration */
  TeleoReactor::xml_factory::declare<DrifterTracker> decl("DrifterTracker");  

  int const WGS_84 = 23;

  void geo_to_utm(double lat, double lon, double &north, double &east) {
    char zone[4];
    LLtoUTM(WGS_84, lat, lon, north, east, zone);
  }
}

DrifterTracker::DrifterTracker(TeleoReactor::xml_arg_type arg) 
  :TeleoReactor(arg, false) {
  m_connection.open("messaging.shore.mbari.org", 5672);
  m_connection.login("tracking", "MBARItracking", "trackingvhost");

  syslog("amqp")<<"Creating queue \"trex2"<<getName()<<std::endl;
  m_queue = m_connection.create_queue("trex2"+getName().str());
  provide("MessagesFromTrex");
  if( isInternal("MessagesFromTrex") ) {    
    // Bind to TREX messages
    syslog("amqp")<<"Binding to TREX messages (MessagesFromTrex/SensorApp)";
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
	    syslog("amqp")<<"Binding to "<<exch_name<<"/*";
	    m_queue->bind(exch_name, "");
	  }	  
	  m_drifters.insert(std::make_pair(tl_name, point()));
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
  syslog()<<"Initialize queue.";
  m_queue->configure(false, true, false);
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
      std::map<TREX::utils::Symbol, point>::iterator 
	i  = m_drifters.find(report.name());
      
      if( m_drifters.end()!=i ) {
	// I only accpet message that have :
	//  -  a date
	//  - a position 
	time_t sec;
	double lat, lon;
	syslog("amqp")<<"New message from "<<report.name();

	// build the Observation
	Observation obs(i->first, "Msg");
	
	if( report.has_epoch_seconds() ) {
	  sec = report.epoch_seconds();
	  obs.restrictAttribute("tick", IntegerDomain(timeToTick(sec)));
	  if( report.has_latitude() ) {
	    lat = report.latitude();
	    obs.restrictAttribute("latitude", FloatDomain(lat));
	 	  
	    if( report.has_longitude() ) {
	      lon = report.longitude();
	      obs.restrictAttribute("longitude", FloatDomain(lon));

	      double north, east, sn, se;
	      geo_to_utm(report.latitude(), report.longitude(), north, east);
	      if( i->second.update(sec, north, east, sn, se) ) {
		obs.restrictAttribute("speed_north", FloatDomain(sn));
		obs.restrictAttribute("speed_east", FloatDomain(se));
	      }	
	      postObservation(obs);	
	    }
	  }
	}
      } else {
	syslog("amqp")<<"Ignoring message from "<<report.name();
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
  obs.restrictAttribute("tick", IntegerDomain(timeToTick(sec)));
  return obs;
}

bool DrifterTracker::point::update(time_t t, double n, double e, double &sn, 
				   double &se) {
  bool ret = false;

  if( valid ) {
    long dt = t-date;
    sn = (n-north);
    se = (e-east);
    sn /= dt;
    se /= dt;
    ret = true;
  }
  date = t;
  north = n;
  east = e;
  valid = true;
  
  return ret;
}
