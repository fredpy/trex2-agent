#include "DoradoHandler.hh"
#include "sensor.pb.h"

#include <trex/domain/IntegerDomain.hh>
#include <trex/domain/FloatDomain.hh>

# define DATA_TL "trex_data"

using namespace mbari;
using namespace TREX::transaction;

namespace {
  MessageHandler::factory::declare<DoradoHandler> decl("MBFD");
}

DoradoHandler::DoradoHandler(MessageHandler::xml_arg const &arg) 
  :MessageHandler(arg), m_updated(false) {
  provide(DATA_TL);
  notify(TREX::transaction::Observation(DATA_TL, "undefined"));
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
    m_updated = true;
    return true;
  }
  return false;
}

bool DoradoHandler::synchronize() {
  if( m_updated ) {
    TREX::transaction::Observation obs(DATA_TL, "Received");
    double delta_t = std::floor(tickToTime(now()));
    delta_t -= m_serie.newest();
    delta_t /= tickDuration();
    obs.restrictAttribute("freshness", IntegerDomain(std::floor(delta_t+0.5)));
    obs.restrictAttribute("nsamples", IntegerDomain(m_serie.size()));
    notify(obs);
  }
  m_updated = false;
  return true;
}
