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
#include "DoradoHandler.hh"
#include "sensor.pb.h"

#include <trex/domain/IntegerDomain.hh>
#include <trex/domain/FloatDomain.hh>
#include <trex/domain/BooleanDomain.hh>
#include <trex/domain/EnumDomain.hh>

# define DATA_TL "trex_data"
# define MBFD_TL "mbfd"
# define TREX_TL "drifterFollow"

using namespace mbari;
using namespace TREX::transaction;

namespace {
  MessageHandler::factory::declare<DoradoHandler> decl("MBFD");
}

DoradoHandler::DoradoHandler(MessageHandler::xml_arg const &arg) 
  :MessageHandler(arg), m_updated(false), 
  m_last_obs(TREX_TL, "undefined"), m_obs_fresh(false) {
  provide(DATA_TL);
  notify(TREX::transaction::Observation(DATA_TL, "undefined"));
  provide(MBFD_TL, true);
  notify(TREX::transaction::Observation(MBFD_TL, "Inactive"));
  provide(TREX_TL);
  notify(m_last_obs);
    

  m_filters.insert(std::make_pair("temperature", kalman_filter_fn(1.0, 1.0, .00024, .05)));
  m_filters.insert(std::make_pair("salinity", kalman_filter_fn(1.0, 1.0, .00012, .05)));
}

bool DoradoHandler::handleRequest(TREX::transaction::goal_id const &g) {
  // I need to process the goal or at least initiate some processing 
  return g->object()==MBFD_TL; 
}


bool DoradoHandler::handleMessage(amqp::queue::message &msg) {
  org::mbari::trex::SensorMessage data;
  
  if( data.ParseFromArray(msg.body(), msg.size()) ) {
    int samples = data.sample_size();
    for(int i=0; i<samples; ++i) {
      org::mbari::trex::SensorMessage_Sample const &samp = data.sample(i);
      time_t date = samp.utime();
      point<3> pos;
      sensor_data values;
      
      pos[0] = samp.northing();
      pos[1] = samp.easting();
      pos[2] = samp.depth();
      
      if( samp.has_temperature() ) 
	values.insert(sensor_data::value_type("temperature", samp.temperature()));
      if( samp.has_salinity() ) 
	values.insert(sensor_data::value_type("salinity", samp.salinity()));
      if( samp.has_nitrate() ) 
	values.insert(sensor_data::value_type("nitrate", samp.nitrate()));
      if( samp.has_chfl() ) 
	values.insert(sensor_data::value_type("ch_fl", samp.chfl()));
      m_serie.add(date, pos, values);
    }
    if( data.has_gps_fix() ) {
      org::mbari::trex::SensorMessage_GpsError const &fix = data.gps_fix();
      point<2> error_rate;
      error_rate[0] = fix.northing_error_rate();
      error_rate[1] = fix.easting_error_rate();
      m_serie.align(fix.from_time(), fix.to_time(), error_rate);
    }
    for(size_t i=0; i<data.observations_size(); ++i) {
      org::mbari::trex::Predicate const &pred(data.observations(i));
      if( TREX_TL==pred.object() ) {
        time_t date = pred.start();
        
        if( m_obs_fresh && m_since>date )
          continue; // ignore old observations
        m_last_obs = Observation(pred.object(), pred.predicate());
        m_obs_fresh = true;
        m_since = date;
        // Add token attributes :
        for(int v=0; v<pred.attributes_size(); ++v) {
          org::mbari::trex::Predicate_Variable const &var = pred.attributes(v);
          std::string name = var.name();
          if( var.has_int_d() ) {
            org::mbari::trex::Predicate_FloatDomain const &dom = var.int_d();
            FloatDomain::bound lo(FloatDomain::minus_inf), 
                                hi(FloatDomain::plus_inf);
            
            if( dom.has_min_v() )
              lo = dom.min_v();
            if( dom.has_max_v() )
              hi = dom.max_v();
            m_last_obs.restrictAttribute(name, FloatDomain(lo, hi));
          } else if( var.has_bool_d() ) {
            m_last_obs.restrictAttribute(name, BooleanDomain(var.bool_d()));
          } else {
            size_t enums = var.enum_d_size();
            if( enums>0 ) {
              EnumDomain values;
              for(size_t e=0; e<enums; ++e)
                values.add(var.enum_d(e));
              m_last_obs.restrictAttribute(name, values);
            }
          }
        }
      }
    }
    
    m_updated = true;
    return true;
  }
  return false;
}

bool DoradoHandler::synchronize() {
  if( m_updated ) {
    // Set all the 
    TREX::transaction::Observation obs(DATA_TL, "Received");
    double delta_t = std::floor(tickToTime(now()));
    delta_t -= m_serie.newest();
    delta_t /= tickDuration();
    obs.restrictAttribute("freshness", IntegerDomain(std::floor(delta_t+0.5)));
    obs.restrictAttribute("nsamples", IntegerDomain(m_serie.size()));
    notify(obs);
    
    if( m_obs_fresh ) {
      // Received an observcation from trex/dorado
      delta_t = std::floor(tickToTime(now()));
      delta_t -= m_since;
      delta_t /= tickDuration();
      // indicate how aold this observation is in term of ticks
      m_last_obs.restrictAttribute("freshness", IntegerDomain(std::floor(delta_t+0.5)));
      notify(m_last_obs);
      m_obs_fresh = false;
    }
  }
  m_updated = false;
  return true;
}
