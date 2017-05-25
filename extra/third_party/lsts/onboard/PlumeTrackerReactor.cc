/*
 * PlumeTrackerReactor.cc
 *
 *  Created on: May 22, 2017
 *      Author: pcooksey
 */

#include "PlumeTrackerReactor.hh"

namespace
{

  /** @brief TREX log entry point */
  SingletonUse<LogManager> s_log;

  /** @brief Platform reactor declaration */
  TeleoReactor::xml_factory::declare<TREX::LSTS::PlumeTrackerReactor> decl("PlumeTrackerReactor");

}

namespace TREX {
  namespace LSTS {

    // Symbol equality test is faster than string : use global Symbols to improve performances
    // Use timelines
    utils::Symbol const PlumeTrackerReactor::s_trex_pred("TREX");
    utils::Symbol const PlumeTrackerReactor::s_control_tl("control");
    utils::Symbol const PlumeTrackerReactor::s_position_tl("estate");
    utils::Symbol const PlumeTrackerReactor::s_reference_tl("reference");

    utils::Symbol const PlumeTrackerReactor::s_plume_tl("plumeindicator");
    utils::Symbol const PlumeTrackerReactor::s_yoyo_tl("yoyo");
    utils::Symbol const PlumeTrackerReactor::s_yoyo_state_tl("yoyo_state");
    utils::Symbol const PlumeTrackerReactor::s_depth_tl("depth");
    
    // Plume detection
    utils::Symbol const PlumeTrackerReactor::s_plume_unknown("Unknown");
    utils::Symbol const PlumeTrackerReactor::s_plume_inside("Inside");
    utils::Symbol const PlumeTrackerReactor::s_plume_outside("Outside");


    PlumeTrackerReactor::PlumeTrackerReactor(TeleoReactor::xml_arg_type arg) :
              LstsReactor(arg), 
              m_lastControl(s_control_tl, "Failed"), 
              m_debug_log(s_log->service()),
              m_depth_log(s_log->service())
    {
      b_last_inside_plume = b_inside_plume = -1;
      
      use(s_control_tl);
      use(s_plume_tl);
      use(s_yoyo_tl, true);
      use(s_yoyo_state_tl);
      use(s_depth_tl);
      use(s_reference_tl, true);
      use(s_position_tl);
      
      angle = start_ang;
      state = IDLE;
      
      utils::LogManager::path_type fname = file_name("plumetracker.log");
      m_debug_log.open(fname.c_str());
      fname = file_name("depth.csv");
      m_depth_log.open(fname.c_str());
    }

    void
    PlumeTrackerReactor::handleInit()
    {

    }

    void
    PlumeTrackerReactor::handleTickStart()
    {
    }

    bool
    PlumeTrackerReactor::synchronize()
    {
      m_debug_log<<"++++++++ Plume Tracker ++++++++++"<<"\n";
      TICK cur = getCurrentTick();
      
      if (!m_trex_control)
        return true;
      
      switch (state) 
      {
        case IDLE:
          m_debug_log<<"Idle"<<"\n";
          if (b_inside_plume == 0) 
          {
            goingIn();
            state = OUTSIDE_GOINGIN;
          } 
          else if (b_inside_plume == 1)
          {
            goingOut();
            state = INSIDE_GOINGOUT;
          }
          break;
        case INSIDE_GOINGOUT:
          m_debug_log<<"Going out!"<<"\n";
          m_debug_log<<yoyo_states.size()<<"\n";
          if (b_inside_plume == 0) 
          {
            state = OUTSIDE;
            plume_edge_lat = plume_lat; 
            plume_edge_lon = plume_lon;
          }
          break;
        case OUTSIDE:
          m_debug_log<<"Outside!"<<"\n";
          {
            double dist_from_edge = WGS84::distance(m_lastPosition.lat, m_lastPosition.lon, 0, plume_edge_lat, plume_edge_lon, 0);
            m_debug_log<<"Distance: "<<dist_from_edge<<"\n";
            if (dist_from_edge > outside_plume_dist) 
            {
              state = IDLE;
            }
          }
          break;
        case OUTSIDE_GOINGIN:
          m_debug_log<<"Going in!"<<"\n";
          m_debug_log<<yoyo_states.size()<<"\n";
          m_debug_log<<"Inside plume: "<<b_inside_plume<<"\n";
          if (b_inside_plume == 1)
          {
            yoyo_states.clear();
            state = INSIDE;
          }
          break;
        case INSIDE:
          m_debug_log<<"Inside"<<"\n";
          m_debug_log<<"Doing "<<yoyo_count<<"number of yoyos\n";
          m_debug_log<<yoyo_states.size()<<"\n";
          m_debug_log<<"Inside plume: "<<b_inside_plume<<"\n";
          if (yoyo_states.size() >= yoyo_count*2)
          {
            yoyo_states.clear();
            state = IDLE;
          }
          break;
        default:
          std::cerr<<"ERROR: Should not have gotten to this point"<<"\n";
      }
      return true;
    }
    
    bool 
    PlumeTrackerReactor::goingOut()
    {
      if (angle >= end_ang) {
        m_debug_log<<"Finished!"<<"\n";
        return false;
      }
      m_debug_log<<"Going out. Angle: " << angle <<"\n";
      double angRads = Math::Angles::radians(angle);
      double offsetX = std::cos(angRads) * max_dist;
      double offsetY = std::sin(angRads) * max_dist;
      tracking_lat = river_lat;
      tracking_lon = river_lon;
      WGS84::displace(offsetX, offsetY, &tracking_lat, &tracking_lon);
      
      sendReferenceGoal(tracking_lat, tracking_lon);
      //sendYoYoGoal(tracking_lat, tracking_lon);
      
      return true;
    }

    bool 
    PlumeTrackerReactor::goingIn()
    {
      angle += angle_inc;
      m_debug_log<<"Going in. Angle: "<<angle<<"\n";
      double angRads = Math::Angles::radians(angle);
      double offsetX = std::cos(angRads) * min_dist;
      double offsetY = std::sin(angRads) * min_dist;
      tracking_lat = river_lat;
      tracking_lon = river_lon;
      WGS84::displace(offsetX, offsetY, &tracking_lat, &tracking_lon);
      
      sendYoYoGoal(tracking_lat, tracking_lon);             
      return true;
    }
    
    void 
    PlumeTrackerReactor::sendYoYoGoal(const double& lat, const double& lon)
    {
      m_debug_log<<"Trying to Exec Goal ++++++++++"<<"\n";
      Goal g(s_yoyo_tl, "Exec");
      
      g.restrictAttribute(Variable("latitude", FloatDomain(lat)));
      g.restrictAttribute(Variable("longitude", FloatDomain(lon)));
      g.restrictAttribute(Variable("min_z", FloatDomain(min_depth)));
      g.restrictAttribute(Variable("max_z", FloatDomain(max_depth)));
      g.restrictAttribute(Variable("speed", FloatDomain(yoyo_speed)));

      postGoal(g);
    }
    
    void 
    PlumeTrackerReactor::sendReferenceGoal(const double& lat, const double& lon)
    {
      m_debug_log<<"Trying to Reference Goal ++++++++++"<<"\n";
      Goal g(s_reference_tl, "Going");
      
      g.restrictAttribute(Variable("latitude", FloatDomain(lat)));
      g.restrictAttribute(Variable("longitude", FloatDomain(lon)));
      g.restrictAttribute(Variable("z", FloatDomain(surface_depth)));
      g.restrictAttribute(Variable("speed", FloatDomain(yoyo_speed)));

      postGoal(g);
    }


    void
    PlumeTrackerReactor::notify(TREX::transaction::Observation const &obs)
    {
      if (s_plume_tl == obs.object())
      {
        if(obs.predicate() == s_plume_unknown)
          b_inside_plume = -1;
        else if(obs.predicate() == s_plume_inside)
        {
          b_inside_plume = 1;
          plume_lat = obs.getAttribute("latitude").domain().getTypedSingleton<double,true>();
          plume_lon = obs.getAttribute("longitude").domain().getTypedSingleton<double,true>();
        }
        else if(obs.predicate() == s_plume_outside)
        {
          b_inside_plume = 0;
        }
        
        if (b_last_inside_plume == -1) 
          b_last_inside_plume = b_inside_plume;
      }
#if !SIMULATE_DATA
      else if (s_yoyo_state_tl == obs.object())
      {
        m_debug_log<<"++ Yoyo state: "<<obs.predicate()<<"\n";
        if(yoyo_states.size()>0)
          m_debug_log<<yoyo_states.back()<<"\n";
        if ((obs.predicate() == "Ascending" && (yoyo_states.size()<1 || yoyo_states.back() != "Ascending"))
            || (obs.predicate() == "Descending" && (yoyo_states.size()<1 || yoyo_states.back() != "Descending")))
          yoyo_states.push_back(obs.predicate());
        if(yoyo_states.size()>0)
          m_debug_log<<yoyo_states.back()<<"\n";
      } 
      else if (s_depth_tl == obs.object())
      {
        if (obs.predicate() == "Value")
        {
          double value = obs.getAttribute("value").domain().getTypedSingleton<double,true>();
          m_debug_log<<value<<"\n";
        }
      }
#else
      else if (s_depth_tl == obs.object())
      {
        m_debug_log<<"++ Depth state"<<"\n";
        static double value = 0, last_value = 100, ascending = -1;
        double diff = 0;
        m_debug_log<<"-- Last_value = "<<last_value<<"\n";
        m_debug_log<<"-- Ascending = "<<ascending<<"\n";
        if (obs.predicate() == "Value")
        {
          value = obs.getAttribute("value").domain().getTypedSingleton<double,true>();
          m_debug_log<<"-- Value = "<<value<<"\n";
          diff = last_value - value;
          if (last_value == 100) 
            last_value = value;
          else if (diff > 1) 
          {
            last_value = value;
            if (ascending == 0 || ascending == -1)
            {
              yoyo_states.push_back("Ascending");
              ascending = 1;
            }
          }
          else if (diff < -1)
          {
            last_value = value;
            if (ascending == 1 || ascending == -1)
            {
              yoyo_states.push_back("Decending");
              ascending = 0;
            }
          }
          
          // Keep updating the values for the greatest or lower depth
          if (ascending == 1 && last_value > value)
            last_value = value;
          else if (ascending == 0 && last_value < value)
            last_value = value;
        }
      }
#endif
      else if (s_position_tl == obs.object())
      {
        if (obs.predicate() == "Position")
        {
          m_lastPosition.lat = obs.getAttribute("latitude").domain().getTypedSingleton<double,true>();
          m_lastPosition.lon = obs.getAttribute("longitude").domain().getTypedSingleton<double,true>();
        }
      }
      else if (s_control_tl == obs.object())
      {
        m_lastControl = obs;
        if (m_lastControl.predicate() != "TREX")
        {
          //state = UNKNOWN;
        } else {
          m_trex_control = true;
        }
      }
    }

    PlumeTrackerReactor::~PlumeTrackerReactor()
    {
      // TODO Auto-generated destructor stub
    }
  }
}