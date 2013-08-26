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
#include <trex/domain/BooleanDomain.hh>
#include <trex/domain/EnumDomain.hh>

#include <boost/date_time/posix_time/posix_time_io.hpp>

#include "platformMessage.pb.h"

using namespace mbari;
using namespace TREX::utils;
using namespace TREX::transaction;

namespace {
  PositionHandler::factory::declare<PositionHandler> ph_decl("Tracker");
}

PositionHandler::PositionHandler(PositionHandler::xml_arg const &arg)
:MessageHandler(arg, "_pos_"),
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
      earth_point loc(lon, lat);
      
//      syslog(info)<<"New position update from "<<asset<<" ("<<loc.latitude()<<", "<<loc.longitude()<<")"
//		  <<" at "<<secs;

      boost::tie(pos, inserted) = m_assets.insert(asset_map::value_type(asset, base));
      
      pos->second.second.update(secs, loc);
      pos->second.first = true;
      if( inserted )
        provide(tl_name(asset));
      return true;
    } 
  }
  return false;
}

bool PositionHandler::synchronize() {
  date_type now_t = tickToTime(now());
  
  for(asset_map::iterator i=m_assets.begin(); m_assets.end()!=i; ++i) {
    if( i->second.first || (m_should_project && i->second.second.have_speed()) ) {
      TREX::transaction::Observation obs(tl_name(i->first), "Holds");
      location::duration_type dt;
      earth_point vect(i->second.second.position(now_t, dt, m_should_project));
      
      duration_type delta_t = dt.to_chrono<duration_type>();
      
          
      i->second.first = false;
      
      long double dtick = delta_t.count();
      dtick /= tickDuration().count();
    
      obs.restrictAttribute("freshness", 
                            IntegerDomain(std::floor(dtick+0.5)));

      if( vect.is_utm() ) {
        obs.restrictAttribute("utm", BooleanDomain(true));
        obs.restrictAttribute("northing", FloatDomain(vect.utm_northing()));
        obs.restrictAttribute("easting", FloatDomain(vect.utm_easting()));
        obs.restrictAttribute("utm_number", IntegerDomain(vect.utm_number()));
        Symbol ltr(std::string(1, vect.utm_letter()));
        obs.restrictAttribute("utm_letter", EnumDomain(ltr));
      } else 
        obs.restrictAttribute("utm", BooleanDomain(false));
      obs.restrictAttribute("latitude", FloatDomain(vect.latitude()));
      obs.restrictAttribute("longitude", FloatDomain(vect.longitude()));
    
      if( i->second.second.have_speed() && m_should_project ) {
        double lin_sp = i->second.second.lin_speed();
        point<2> sp = i->second.second.speed();
        obs.restrictAttribute("speed_north", FloatDomain(sp[0]));
        obs.restrictAttribute("speed_east", FloatDomain(sp[1]));
        obs.restrictAttribute("lin_speed", FloatDomain(lin_sp));
        if( lin_sp>0.0 )
          // do not set heading if we do not move
          obs.restrictAttribute("heading", FloatDomain(i->second.second.heading()));
      }
      notify(obs);
    }
  }
  return true;
}
