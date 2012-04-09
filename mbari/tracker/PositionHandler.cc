#include "PositionHandler.hh"
#include "DrifterTracker.hh"

#include <trex/domain/IntegerDomain.hh>
#include <trex/domain/FloatDomain.hh>

#include "platformMessage.pb.h"

using namespace mbari;
using namespace TREX::utils;
using namespace TREX::transaction;

namespace {
  PositionHandler::factory::declare<PositionHandler> ph_decl("Tracker");
}

PositionHandler::PositionHandler(PositionHandler::xml_arg const &arg)
  :MessageHandler(arg), 
   m_asset(parse_attr<std::string>(factory::node(arg), "asset")) {
  m_exchange += "_pb";

  provide(m_asset);
  notify(TREX::transaction::Observation(m_asset, "undefined"));
  m_fresh = false;
}

bool PositionHandler::handleMessage(amqp::queue::message &msg) {
  MBARItracking::PlatformReport report;
  
  if( report.ParseFromArray(msg.body(), msg.size()) ) {
    if( report.has_name() && m_asset==report.name() ) {
      time_t secs;
      double lat, lon;
      
      // Check that it has the necessary attributes
      if( report.has_epoch_seconds() ) {
	secs = report.epoch_seconds();
	if( report.has_latitude() ) {
	  lat = report.latitude();
	  if( report.has_longitude() ) {
	    lon = report.longitude();
	    double north, east;
	    geo_to_utm(lat, lon, north, east);
	    m_position.update(secs, north, east);
	    m_fresh = true;
	    return true;
	  }
	}
      }
    } 
  }
  return false;
}

bool PositionHandler::synchronize() {
  if( m_fresh || m_position.have_speed() ) {
    TREX::transaction::Observation obs(m_asset, "Holds");
    time_t now_t = std::floor(tickToTime(now()));
    long int dt;
    point<2> vect(m_position.position(now_t, dt));
    double lat, lon;
    
    m_fresh = false;

    double dtick = dt;
    dtick /= tickDuration();
    
    obs.restrictAttribute("freshness", IntegerDomain(std::floor(dtick+0.5)));

    obs.restrictAttribute("northing", FloatDomain(vect[0]));
    obs.restrictAttribute("easting", FloatDomain(vect[1]));
    utm_to_geo(vect[0], vect[1], lat, lon);
    obs.restrictAttribute("latitude", FloatDomain(lat));
    obs.restrictAttribute("longitude", FloatDomain(lon));
    
    if( m_position.have_speed() ) {
      obs.restrictAttribute("speed_north", FloatDomain(m_position.speed()[0]));
      obs.restrictAttribute("speed_east", FloatDomain(m_position.speed()[1]));
    }
    notify(obs);
  }
  return true;
}
