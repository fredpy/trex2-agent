#include <set>

#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>

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
  void utm_to_geo(double north, double east, double &lat,  double &lon) {
    const char* Zone = "10";
    const int NORTHERN_HEMISPHERE_BUFFER = 10000000;
    UTMtoLL(WGS_84, 
	    north + NORTHERN_HEMISPHERE_BUFFER, east, 
	    Zone, lat, lon);
    if(east == 0.0)
      lon = -126.0;
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
    syslog("amqp")<<"Binding to TREX messages (MessagesFromTrex/TrackingApp)";
    m_queue->bind("MessagesFromTrex", "TrackingApp");
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
	provide(tl_name.str()+"_msg");
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

  std::map<TREX::utils::Symbol, point>::const_iterator i = m_drifters.begin();
  for(;m_drifters.end()!=i; ++i) {
    Observation obs(i->first, "undefined");
    postObservation(obs);
    Observation obs2(i->first.str()+"_msg", "none");
    postObservation(obs2);
  }
}

bool DrifterTracker::synchronize() {
  while( !m_messages.empty() ) {
    boost::shared_ptr<amqp::queue::message> msg = m_messages.pop();
    if( msg->exchange()=="MessagesFromTrex" ) {
      try {
	trex_msg(*msg);
      } catch(...) {}
    } else 
      drifter_msg(*msg);
  }
  std::map<TREX::utils::Symbol, point>::const_iterator i = m_drifters.begin();
  for(;m_drifters.end()!=i; ++i) {
    if( i->second.is_valid() ) {
      Observation obs(i->first, "Holds");
      // obs.restrictAttribute("tick", 
      // 			    IntegerDomain(timeToTick(i->second.last_update())));
      time_t now = std::floor(tickToTime(getCurrentTick()));
      std::pair<double, double> tmp = i->second.position(now);
      obs.restrictAttribute("northing",
      			    FloatDomain(tmp.first));
      obs.restrictAttribute("easting",
      			    FloatDomain(tmp.second));
      double lat, lon;
      utm_to_geo(tmp.first, tmp.second, lat, lon);
      obs.restrictAttribute("latitude",
			    FloatDomain(lat));
      obs.restrictAttribute("longitude",
			    FloatDomain(lon));
      
      // if( i->second.has_speed() ) {
      // 	tmp = i->second.speed();
      // 	obs.restrictAttribute("speed_north", FloatDomain(tmp.first));
      // 	obs.restrictAttribute("speed_east", FloatDomain(tmp.second));
      // }
      postObservation(obs);
    }
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
	Observation obs(i->first.str() + "_msg", "Msg");
	
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
	      i->second.update(sec, north, east);
	      // obs.restrictAttribute("northing", FloatDomain(north));
	      // obs.restrictAttribute("easting", FloatDomain(east));
	      if( i->second.has_speed() ) {
		obs.restrictAttribute("speed_north", 
				      FloatDomain(i->second.speed().first));
		obs.restrictAttribute("speed_east", 
				      FloatDomain(i->second.speed().second));
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

void DrifterTracker::point::update(time_t t, double n, double e) {
  if( valid ) {
    long dt = t-date;
    m_speed.first = (n-m_position.first);
    m_speed.second = (e-m_position.second);
    m_speed.first /= dt;
    m_speed.second /= dt;
    with_speed = true;
  } else 
    with_speed = false;
  date = t;
  m_position.first = n;
  m_position.second = e;
  valid = true;  
}

std::pair<double, double> DrifterTracker::point::position(time_t now) const {
  std::pair<double,double> est_pos(m_position);
  long int delta_t = now-date;
  
  est_pos.first += delta_t*m_speed.first;
  est_pos.second += delta_t*m_speed.second;
  
  return est_pos;
}
