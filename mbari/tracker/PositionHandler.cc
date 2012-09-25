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
#include "PositionHandler.hh"
#include "DrifterTracker.hh"

#include <trex/domain/IntegerDomain.hh>
#include <trex/domain/FloatDomain.hh>

#include <boost/date_time/posix_time/posix_time_io.hpp>

#include "platformMessage.pb.h"

using namespace mbari;
using namespace TREX::utils;
using namespace TREX::transaction;

namespace {
  PositionHandler::factory::declare<PositionHandler> ph_decl("Tracker");
}

PositionHandler::PositionHandler(PositionHandler::xml_arg const &arg)
:MessageHandler(arg), 
 m_should_project(parse_attr<bool>(true, MessageHandler::factory::node(arg), "projected")) { 
  m_exchange += "_pb";
}

bool PositionHandler::handleMessage(amqp::queue::message &msg) {
  MBARItracking::PlatformReport report;
  
  if( report.ParseFromArray(msg.body(), msg.size()) ) {
    if( report.has_name() ) {
      std::string asset = report.name();      
      date_type secs;
      double lat, lon;
      
      if( report.has_epoch_seconds() )
        secs = boost::posix_time::from_time_t(report.epoch_seconds());
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

      syslog(info)<<"New position update from "<<asset<<" ("<<lat<<", "<<lon<<")"
		  <<" at "<<secs;

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
  date_type now_t = tickToTime(now());
  typedef TREX::utils::chrono_posix_convert<duration_type> cvt;
  
  for(asset_map::iterator i=m_assets.begin(); m_assets.end()!=i; ++i) {
    if( i->second.first || (m_should_project && i->second.second.have_speed()) ) {
      TREX::transaction::Observation obs(i->first, "Holds");
      location::duration_type dt;
      point<2> vect(i->second.second.position(now_t, dt, m_should_project));
      duration_type delta_t = cvt::to_chrono(dt); 
      double lat, lon;
    
      i->second.first = false;

      
      long double dtick = delta_t.count();
      dtick /= tickDuration().count();
    
      obs.restrictAttribute("freshness", 
                            IntegerDomain(std::floor(dtick+0.5)));

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
