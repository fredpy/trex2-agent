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
:MessageHandler(arg) { 
  m_exchange += "_pb";
}

bool PositionHandler::handleMessage(amqp::queue::message &msg) {
  MBARItracking::PlatformReport report;
  
  if( report.ParseFromArray(msg.body(), msg.size()) ) {
    if( report.has_name() ) {
      std::string asset = report.name();      
      time_t secs;
      double lat, lon;
      
      if( report.has_epoch_seconds() )
        secs = report.epoch_seconds();
      else
        return false;
      
      if( report.has_latitude() )
        lat = report.latitude();
      else 
        return false;
      
      if( report.has_longitude() )
        lon = report.longitude();
      else 
        return false;
      
      asset_info base;
      asset_map::iterator pos;
      bool inserted;
      double north, east;
      
      geo_to_utm(lat, lon, north, east);

      boost::tie(pos, inserted) = m_assets.insert(asset_map::value_type(asset, base));
      pos->second.second.update(secs, north, east);
      pos->second.first = true;
      if( inserted )
        provide(asset);
      return true;
    } 
  }
  return false;
}

bool PositionHandler::synchronize() {
  time_t now_t = std::floor(tickToTime(now()));
  
  for(asset_map::iterator i=m_assets.begin(); m_assets.end()!=i; ++i) {
    if( i->second.first || i->second.second.have_speed() ) {
      TREX::transaction::Observation obs(i->first, "Holds");
      long int dt;
      point<2> vect(i->second.second.position(now_t, dt));
      double lat, lon;
    
      i->second.first = false;

      double dtick = dt;
      dtick /= tickDuration();
    
      obs.restrictAttribute("freshness", IntegerDomain(std::floor(dtick+0.5)));

      obs.restrictAttribute("northing", FloatDomain(vect[0]));
      obs.restrictAttribute("easting", FloatDomain(vect[1]));
      utm_to_geo(vect[0], vect[1], lat, lon);
      obs.restrictAttribute("latitude", FloatDomain(lat));
      obs.restrictAttribute("longitude", FloatDomain(lon));
    
      if( i->second.second.have_speed() ) {
        obs.restrictAttribute("speed_north", FloatDomain(i->second.second.speed()[0]));
        obs.restrictAttribute("speed_east", FloatDomain(i->second.second.speed()[1]));
      }
      notify(obs);
    }
  }
  return true;
}
