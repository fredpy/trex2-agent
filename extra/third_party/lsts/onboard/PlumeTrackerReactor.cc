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
              m_debug_log(s_log->service())
    { 
      use(s_control_tl);
      use(s_plume_tl);
      use(s_yoyo_tl, true);
      use(s_yoyo_state_tl);
      use(s_depth_tl);
      use(s_reference_tl, true);
      use(s_position_tl);
      
      angle = start_ang;
      yoyo_done = false;
      s_past_log = "";
      
      e_exec_state = IDLE;
      e_plume_state = PLUME::UNKNOWN;
      
      utils::LogManager::path_type fname = file_name("plumetracker.log");
      m_debug_log.open(fname.c_str());
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
      // Check if we have control of the AUV
      if (!m_trex_control)
        return true;
      
      ss_debug_log<<"++++++++ Plume Tracker ++++++++++"<<"\n";
      TICK cur = getCurrentTick();
      
      switch (e_exec_state) 
      {
        case IDLE:
          ss_debug_log<<"Currently Idle"<<"\n";
          if (e_plume_state == PLUME::OUTSIDE) 
          {
            goingIn();
            e_exec_state = OUTSIDE_GOINGIN;
          } 
          else if (e_plume_state == PLUME::INSIDE)
          {
            goingOut();
            e_exec_state = INSIDE_GOINGOUT;
          }
          break;
        case INSIDE_GOINGOUT:
          ss_debug_log<<"Going out of Plume!"<<"\n";
          if (e_plume_state == PLUME::OUTSIDE) 
          {
            e_exec_state = OUTSIDE;
            plume_edge_lat = plume_lat; 
            plume_edge_lon = plume_lon;
          }
          break;
        case OUTSIDE:
          ss_debug_log<<"Outside of the Plume!"<<"\n";
          {
            double dist_from_edge = WGS84::distance(m_lastPosition.lat, m_lastPosition.lon, 0, plume_edge_lat, plume_edge_lon, 0);
            ss_debug_log<<"Distance: "<<dist_from_edge<<"\n";
            if (dist_from_edge > outside_plume_dist) 
            {
              e_exec_state = IDLE;
            }
          }
          break;
        case OUTSIDE_GOINGIN:
          ss_debug_log<<"Going into the plume!"<<"\n";
          ss_debug_log<<"Doing "<<yoyo_count<<" yoyos\n";
          ss_debug_log<<"Currently at "<<yoyo_states.size()/2<<"\n";
          if (e_plume_state == PLUME::INSIDE)
          {
            yoyo_states.clear();
            e_exec_state = INSIDE;
          }
          break;
        case INSIDE:
          ss_debug_log<<"Inside the plume"<<"\n";
          ss_debug_log<<"Doing "<<yoyo_count<<" yoyos\n";
          ss_debug_log<<"Currently at "<<yoyo_states.size()/2<<"\n";
          ss_debug_log<<"Are yoyos done? "<<yoyo_done<<"\n";
          if (yoyo_states.size() >= yoyo_count*2 || yoyo_done)
          {
            yoyo_states.clear();
            e_exec_state = IDLE;
          }
          break;
        default:
          std::cerr<<"ERROR: Should not have gotten to this point"<<"\n";
      }
      // Printing unique Debug messages
      uniqueDebugPrint(cur);
      return true;
    }
    
    bool 
    PlumeTrackerReactor::goingOut()
    {
      if (angle >= end_ang) {
        ss_debug_log<<"Finished!"<<"\n";
        return false;
      }
      ss_debug_log<<"Going out. Angle: " << angle <<"\n";
      double angRads = Math::Angles::radians(angle);
      double offsetX = std::cos(angRads) * max_dist;
      double offsetY = std::sin(angRads) * max_dist;
      tracking_lat = river_lat;
      tracking_lon = river_lon;
      WGS84::displace(offsetX, offsetY, &tracking_lat, &tracking_lon);
      
      sendReferenceGoal(tracking_lat, tracking_lon);
      
      return true;
    }

    bool 
    PlumeTrackerReactor::goingIn()
    {
      angle += angle_inc;
      ss_debug_log<<"Going in. Angle: "<<angle<<"\n";
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
      ss_debug_log<<"+++++ Sending YoYo Goal +++++"<<"\n";
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
      ss_debug_log<<"+++++ Sending Reference Goal +++++"<<"\n";
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
          e_plume_state = PLUME::UNKNOWN;
        else if(obs.predicate() == s_plume_inside)
        {
          e_plume_state = PLUME::INSIDE;
          plume_lat = obs.getAttribute("latitude").domain().getTypedSingleton<double,true>();
          plume_lon = obs.getAttribute("longitude").domain().getTypedSingleton<double,true>();
        }
        else if(obs.predicate() == s_plume_outside)
        {
          e_plume_state = PLUME::OUTSIDE;
        }
      }
#if !SIMULATE_DATA
      else if (s_yoyo_state_tl == obs.object())
      {
        ss_debug_log<<"Yoyo state: "<<obs.predicate()<<"\n";
        if ((obs.predicate() == "Ascending" && (yoyo_states.size()<1 || yoyo_states.back() != "Ascending"))
            || (obs.predicate() == "Descending" && (yoyo_states.size()<1 || yoyo_states.back() != "Descending")))
        {
          yoyo_states.push_back(obs.predicate());
          yoyo_done = false;
        } 
        else if (obs.predicate() == "Idle")
        {
          yoyo_done = true;
        }
      }
#else
      else if (s_depth_tl == obs.object())
      {
        ss_debug_log<<"++ Depth state"<<"\n";
        static double value = 0, last_value = 100, ascending = -1;
        double diff = 0;
        ss_debug_log<<"-- Last_value = "<<last_value<<"\n";
        ss_debug_log<<"-- Ascending = "<<ascending<<"\n";
        if (obs.predicate() == "Value")
        {
          value = obs.getAttribute("value").domain().getTypedSingleton<double,true>();
          ss_debug_log<<"-- Value = "<<value<<"\n";
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
        if (m_lastControl.predicate() == "TREX")
        {
          m_trex_control = true;
        }
      }
    }
    
    void 
    PlumeTrackerReactor::uniqueDebugPrint(TICK cur)
    {
      std::string debug_log = ss_debug_log.str();
      if (s_past_log != debug_log) 
      {
        s_past_log = debug_log;
        m_debug_log <<"Time: "<< cur << "\n" << s_past_log <<"\n";
      }
      ss_debug_log.str(std::string());
    }

    PlumeTrackerReactor::~PlumeTrackerReactor()
    {
      // TODO Auto-generated destructor stub
      m_debug_log.close();
    }
  }
}