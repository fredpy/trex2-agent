/*
 * PlumeIndicatorReactor.cc
 *
 *  Created on: May 18, 2017
 *      Author: pcooksey
 */

#include "PlumeIndicatorReactor.hh"

namespace
{

  /** @brief TREX log entry point */
  SingletonUse<LogManager> s_log;

  /** @brief Platform reactor declaration */
  TeleoReactor::xml_factory::declare<TREX::LSTS::PlumeIndicatorReactor> decl("PlumeIndicatorReactor");

}

namespace TREX {
  namespace LSTS {

    // Symbol equality test is faster than string : use global Symbols to improve performances
    utils::Symbol const PlumeIndicatorReactor::s_trex_pred("TREX");
    utils::Symbol const PlumeIndicatorReactor::s_control_tl("control");
    
    // Use timelines
    utils::Symbol const PlumeIndicatorReactor::s_position_tl("estate");
    utils::Symbol const PlumeIndicatorReactor::s_depth_tl("depth");
    utils::Symbol const PlumeIndicatorReactor::s_temperature_tl("temperature");
    utils::Symbol const PlumeIndicatorReactor::s_salinity_tl("salinity");
    
    // Provide timelines
    utils::Symbol const PlumeIndicatorReactor::s_plumeindicator_tl("plumeindicator");

    PlumeIndicatorReactor::PlumeIndicatorReactor(TeleoReactor::xml_arg_type arg) :
              LstsReactor(arg), m_lastControl(s_control_tl, "Failed")
    {
      state = UNKNOWN;
      
      v_salinity = v_temperature = v_depth = std::vector<double>();
      v_positions = std::vector<Position>();
      m_last_salinity = m_last_temperature = m_last_depth = -1;
      m_trex_control = false;
      
      use(s_position_tl);
      use(s_depth_tl);
      use(s_temperature_tl);
      use(s_salinity_tl);
      provide(s_plumeindicator_tl);
    }

    void
    PlumeIndicatorReactor::handleInit()
    {
      Observation indicator(s_plumeindicator_tl, "Unknown");
      postObservation(indicator);
    }

    void
    PlumeIndicatorReactor::handleTickStart()
    {
    }
    
    double 
    getVariance(double mean, std::vector<double>& values)
    {
        double temp = 0;
        for(std::vector<double>::iterator it = values.begin();
            it!=values.end();
            ++it)
            temp += ((*it)-mean)*((*it)-mean);
        return temp/values.size();
    }

    bool
    PlumeIndicatorReactor::synchronize()
    {
      TICK cur = getCurrentTick();
      static Position plume_loc;
      
      //syslog(utils::log::info)<<"Observed :"<<m_last_depth<<" , "<<m_last_salinity<<" , "<<m_last_temperature<<std::endl;
      
      // Determines if inside or outside a plume
      // TODO: Need model of plume for more accurate identification
      if (m_trex_control || true)
      {
        if (v_depth.rbegin()!=v_depth.rend() && (*v_depth.rbegin()) < depth_for_plume)
        {
          double avg = 0;
          int count_avg = 0;
          //std::cout<<std::endl;
          for (std::vector<double>::const_reverse_iterator it_depth = v_depth.rbegin(), it_salinity = v_salinity.rbegin();
              it_depth != v_depth.rend() && it_salinity != v_salinity.rend();
              ++it_depth, ++it_salinity) {
            if ((*it_depth) < depth_for_plume) {
              avg += (*it_salinity);
              //std::cout<<(*it_salinity)<<std::endl;
              count_avg++;
            } else {
              break; 
            }
          }
          
          avg /= count_avg;
          //std::cout<<"Average: "<<avg<<std::endl<<std::endl;
          if (count_avg >= sample_size) 
          {
            if (avg<=avg_below)
            {
              state = INSIDE;
              plume_loc.lat = v_positions.rbegin()->lat;
              plume_loc.lon = v_positions.rbegin()->lon;
            } 
            else
            {
              state = OUTSIDE;
              plume_loc.lat = v_positions.rbegin()->lat;
              plume_loc.lon = v_positions.rbegin()->lon;
            }
          }
        }
      }
      
      /*
      if (m_trex_control || true)
      {
        if(v_salinity.size()>sample_size/2)
        {
          std::vector<double> diff_depth(v_salinity.size());
          double avg = std::accumulate(v_salinity.begin(), v_salinity.end(), 0.0)/v_salinity.size();
          double half_variance = getVariance(avg, v_salinity)/2;
          //std::adjacent_difference(v_depth.begin(), v_depth.end(), diff_depth.begin());
          //double depth_diff_avg =  std::accumulate(diff_depth.begin(), diff_depth.end(), 0.0)/v_depth.size();

          if((avg-half_variance) > 30)
          {
            state = OUTSIDE;
            plume_loc.lat = v_positions.rbegin()->lat;
            plume_loc.lon = v_positions.rbegin()->lon;
          } 
          else
          {
            state = INSIDE;
            plume_loc.lat = v_positions.rbegin()->lat;
            plume_loc.lon = v_positions.rbegin()->lon;
          }
        }
      }
      */
      
      
      switch (state)
      {
        case (UNKNOWN):
          postUniqueObservation(Observation(s_plumeindicator_tl, "Unknown"));
          break;
        case (INSIDE):
        {
          Observation obs = Observation(s_plumeindicator_tl, "Inside");
          obs.restrictAttribute("latitude", FloatDomain(plume_loc.lat, plume_loc.lat));
          obs.restrictAttribute("longitude", FloatDomain(plume_loc.lon, plume_loc.lon));
          //postUniqueObservation(obs);
          postObservation(obs);
        }
          break;
        case (OUTSIDE):
        {
          Observation obs = Observation(s_plumeindicator_tl, "Outside");
          obs.restrictAttribute("latitude", FloatDomain(plume_loc.lat, plume_loc.lat));
          obs.restrictAttribute("longitude", FloatDomain(plume_loc.lon, plume_loc.lon));
          //postUniqueObservation(obs);
          postObservation(obs);
        }
          break;
      }
      return true;
    }

    void
    PlumeIndicatorReactor::notify(TREX::transaction::Observation const &obs)
    {
      if (s_temperature_tl == obs.object()) 
        getCTDData(obs, v_temperature, m_last_temperature);
      else if (s_salinity_tl == obs.object()) 
        getCTDData(obs, v_salinity, m_last_salinity);
      else if (s_depth_tl == obs.object()) 
        getCTDData(obs, v_depth, m_last_depth);
      else if (s_position_tl == obs.object())
      {
        if (obs.hasAttribute("latitude") && obs.hasAttribute("longitude"))
        {
          m_lastPosition.lat = obs.getAttribute("latitude").domain().getTypedSingleton<double,true>();
          m_lastPosition.lon = obs.getAttribute("longitude").domain().getTypedSingleton<double,true>();
          if(v_positions.size()>=sample_size)
            v_positions.erase(v_positions.begin());
          v_positions.push_back(m_lastPosition);
        }
      }
      else if (s_control_tl == obs.object())
      {
        m_lastControl = obs;
        if (m_lastControl.predicate() != "TREX")
        {
          state = UNKNOWN;
        } else {
          m_trex_control = true;
        }
      }
    }
    
    void PlumeIndicatorReactor::getCTDData(const Observation& obs, std::vector< double >& vec, double& value)
    {
      if (obs.predicate() == "Value")
      {
        value = obs.getAttribute("value").domain().getTypedSingleton<double,true>();
        if (value != -1)
        {
          if (vec.size()>=sample_size)
            vec.erase(vec.begin());
          vec.push_back(value);
        }
      }
    }


    PlumeIndicatorReactor::~PlumeIndicatorReactor()
    {
      // TODO Auto-generated destructor stub
    }
  }
}