/*
 * CoordinationReactor.cc
 *
 *  Created on: May 26, 2017
 *      Author: pcooksey
 */

#include "CoordinationReactor.hh"
#include <boost/thread/pthread/mutex.hpp>
#include <boost/graph/graph_concepts.hpp>

namespace
{

  /** @brief TREX log entry point */
  SingletonUse<LogManager> s_log;

  /** @brief Platform reactor declaration */
  TeleoReactor::xml_factory::declare<TREX::LSTS::CoordinationReactor> decl("CoordinationReactor");

}

namespace TREX {
  namespace LSTS {
    
    using namespace CoordinationReactorState;

    // Symbol equality test is faster than string : use global Symbols to improve performances
    // Use timelines
    utils::Symbol const CoordinationReactor::s_control_tl("control");
    utils::Symbol const CoordinationReactor::s_position_tl("estate");
    utils::Symbol const CoordinationReactor::s_reference_tl("reference");
    utils::Symbol const CoordinationReactor::s_plumetracker_tl("plumetracker");
    utils::Symbol const CoordinationReactor::s_plume_tl("plumeindicator");
    utils::Symbol const CoordinationReactor::s_depth_tl("depth");
    
    // Plume detection
    utils::Symbol const CoordinationReactor::s_plume_unknown("Unknown");
    utils::Symbol const CoordinationReactor::s_plume_inside("Inside");
    utils::Symbol const CoordinationReactor::s_plume_outside("Outside");
    
    utils::Symbol const CoordinationReactor::s_shared_tl("coordination");

    CoordinationReactor::CoordinationReactor(TeleoReactor::xml_arg_type arg) :
              LstsReactor(arg), 
              m_lastControl(s_control_tl, "Failed"), 
              m_debug_log(s_log->service()),
              m_depth_log(s_log->service()),
              uuid(boost::uuids::nil_generator()())
    { 
      m_leader_control = parse_attr<bool>(false, TeleoReactor::xml_factory::node(arg),
                                          "leader");
      
      use(s_control_tl);
      use(s_reference_tl, true);
      use(s_position_tl);
      use(s_plumetracker_tl);
      use(s_plume_tl);
      use(s_depth_tl);
      
      if (m_leader_control)
        provide(s_shared_tl);
      else
        use(s_shared_tl, true);
      
      s_past_log = "";
      
      e_exec_state = INITIAL;
      m_stop_sending_goals = false;
      m_tracker_controlled = true;
      
      next_lat = 0;
      next_lon = 0;
      going_lat = 0;
      going_lon = 0;
      
      moving_locations = false;
      
      m_descent_time = boost::posix_time::seconds(0);
      
      utils::LogManager::path_type fname = file_name("coordination.log");
      m_debug_log.open(fname.c_str());
      
      fname = file_name("depth.csv");
      m_depth_log.open(fname.c_str());
    }

    void
    CoordinationReactor::handleInit()
    {

    }

    void
    CoordinationReactor::handleTickStart()
    {
    }

    bool
    CoordinationReactor::synchronize()
    {
      // Check if we have control of the AUV
      if (!m_trex_control)
        return true;
      
      TICK cur = getCurrentTick();
      ss_debug_log<<"["<<cur<<"] ++++++++ Coordination Reactor ++++++++++"<<"\n";
      
      switch (e_exec_state) 
      {
        case INITIAL:
          ss_debug_log<<"Initializing (";
          ss_debug_log<< ((m_leader_control) ? "Leader)\n" : "Follower)\n");
          if (!m_leader_control)
            e_exec_state = JOINING;
          break;
        case JOINING:
          ss_debug_log<<"Joining the coordination (";
          ss_debug_log<< ((m_leader_control) ? "Leader)\n" : "Follower)\n");
          break;
        case GOING:
          ss_debug_log<<"Going (";
          ss_debug_log<< ((m_leader_control) ? "Leader)\n" : "Follower)\n");
          break;
        case EXEC:
          ss_debug_log<<"Currently executing (";
          ss_debug_log<< ((m_leader_control) ? "Leader)\n" : "Follower)\n");
          break;
        default:
          ss_debug_log<<"ERROR: Coordination Reactor should not have gotten to this point"<<"\n";
      }
      // Depending on leadership it will either post observations or goals
      (m_leader_control) ? leaderPostObservation(cur) : followerPostGoal(cur);
      uniqueDebugPrint(cur);
      return true;
    }
    
    void 
    CoordinationReactor::leaderPostObservation(TICK cur)
    {
      // Only the LEADER uses this function
      switch(e_exec_state)
      {
        case INITIAL:
        {
          // Initally post a time and let other AUV's join the team.
          Observation obs = Observation(s_shared_tl, exec_state_names[e_exec_state]);
          boost::posix_time::ptime t = tickToTime(cur);
          std::string ts = boost::posix_time::to_simple_string(t);
          obs.restrictAttribute("Time", StringDomain(ts));
          TeleoReactor::postObservation(obs);
          // Check if the whole team is connected to transition to JOINING
          if (v_team.size() > 0) {
            e_exec_state = JOINING;
          }
        }
          break;
        case JOINING:
        {
          // I am assuming that I only have one other AUV now.
          if (e_plume_state == PLUME::INSIDE)
            leaderPlanningInsideGoingOut();
          else if (e_plume_state == PLUME::OUTSIDE)
            leaderPlanningOutsideGoingIn();;
          
          // Checking if everyone has received the message
          bool all_received = true;
          for (std::vector<Teammate>::iterator it = v_team.begin();
               it != v_team.end();
               ++it)
          {
            if ((*it).received != true)
              all_received = false;
          }
          
          if (all_received)
            e_exec_state = GOING;
          else
            ss_debug_log<<"All team members have not been received Size:"<<v_team.size()<<"\n";
        }
          break;
        case GOING:
        {
          static bool justStartedDecending = true;
          Observation obs = Observation(s_shared_tl, exec_state_names[e_exec_state]);
          boost::posix_time::ptime t = tickToTime(cur);
          std::string ts = boost::posix_time::to_simple_string(t);
          obs.restrictAttribute("Time", StringDomain(ts));
          postUniqueObservation(obs);
          // Checks if it is time to start and that it hasn't already started (m_start_time is not_a_date_time)
          if (m_start_time <= now() && m_start_time != boost::posix_time::not_a_date_time)
          {
            if (e_plume_state == PLUME::INSIDE)
            {
              ss_debug_log<<"--- Starting Plume Tracker now! ---\n";
              ss_debug_log<<"Time: "<<now()<<"\n";
              sendPlumeTrackerGoal();
              m_start_time = boost::posix_time::not_a_date_time;
            }
            else
            {
              if (justStartedDecending)
              {
                ss_debug_log<<"--- Starting Decent now! ---\n";
                m_start_time = now();
                ss_debug_log<<"Start Time: "<<m_start_time<<"\n";
                sendReferenceGoal(next_lat, next_lon, max_depth);
                justStartedDecending = false;
              }
              else if (m_lastPosition.depth <= 0.5)
              {
                m_beginning_descent_time += now() - m_start_time;
                m_start_time = now();
                ss_debug_log<<"Adjusting Start Time: "<<m_start_time<<"\n";
              }
              else if (m_lastPosition.depth <= (max_depth - .5))
              {
                ss_debug_log<<"--- Decending now! ---\n";
                m_descent_time = now() - m_start_time;
                ss_debug_log<<"Descent Time: "<<m_descent_time<<"\n";
              }
              else
              {
                /*
                if (m_beginning_descent_time != boost::posix_time::seconds(0)) 
                {
                  // Multiple by two because 
                  m_descent_time -= m_beginning_descent_time;
                  m_beginning_descent_time = boost::posix_time::seconds(0);
                }
                */
                m_descent_time -= boost::posix_time::seconds(1);
                ss_debug_log<<"--- Maintaining depth for "<<m_descent_time<<"\n";
                if (m_descent_time <= boost::posix_time::seconds(0))
                {
                  ss_debug_log<<"--- Starting Plume Tracker now! ---\n";
                  sendPlumeTrackerGoal();
                  m_start_time = boost::posix_time::not_a_date_time;
                }
              }
            }
          }
          // If plume tracker has taken over control transition and clear old variables
          if (!m_tracker_controlled)
          {
            e_exec_state = EXEC;
            // Clear variables for coordination since they need to be redone when the AUVs surface again
            m_initial_time = boost::posix_time::not_a_date_time;
            v_team.clear();
            justStartedDecending = true;
          }
        }
          break;
        case EXEC:
          if (m_tracker_controlled) {
            e_exec_state = INITIAL;
            // Should only be called once when transitioning. This keeps the AUV circling.
            //sendReferenceGoal(m_lastPosition.lat, m_lastPosition.lon);
          }
          break;
        default:
          ss_debug_log<<"There is a problem this state is incorrect for this reactor\n";
      }
    }
    
    void 
    CoordinationReactor::followerPostGoal(TICK cur)
    {
      // Followers will be posting goals to the leader using this function
      switch(e_exec_state)
      {
        case JOINING:
        {
          //sendReferenceGoal(m_lastPosition.lat, m_lastPosition.lon);
          // If we have our start time we don't need to send a goal
          if (m_start_time != boost::posix_time::not_a_date_time)
          {
            m_debug_log << "Transition to Going \n";
            e_exec_state = GOING;
            break;
          }
          ss_debug_log<<"Joining\n";
          // Join the team of AUV's after receiving the initial time message
          if (m_initial_time != boost::posix_time::not_a_date_time && !moving_locations)
          {
            Goal g(s_shared_tl, exec_state_names[e_exec_state]);
            boost::posix_time::ptime t = tickToTime(cur);
            std::string ts = boost::posix_time::to_simple_string(t);
            g.restrictAttribute(Variable("Time", StringDomain(ts)));
            
            boost::posix_time::time_duration td = t - m_initial_time;
            std::string duration_ts = boost::posix_time::to_simple_string(td);
            g.restrictAttribute(Variable("Latency", StringDomain(duration_ts)));
            
            if (uuid.is_nil())
              uuid = boost::uuids::random_generator()();
            g.restrictAttribute(Variable("ID", StringDomain(boost::uuids::to_string(uuid))));
            
            g.restrictAttribute(Variable("Lat", FloatDomain(m_lastPosition.lat)));
            g.restrictAttribute(Variable("Lon", FloatDomain(m_lastPosition.lon)));
            LstsReactor::postGoal(g);
          }
          else if(moving_locations)
          {
            double dist = WGS84::distance(m_lastPosition.lat, m_lastPosition.lon, 0, going_lat, going_lon, 0);
            if (dist <= 5)
              moving_locations = false;
          }
          else
            ss_debug_log<<"No join message received yet!\n";
        }
          break;
        case GOING:
        {
          ss_debug_log<<"Going\n";
          if (!m_stop_sending_goals)
          {
            ss_debug_log<<"Sending Goal\n";
            Goal g(s_shared_tl, exec_state_names[e_exec_state]);
            g.restrictAttribute(Variable("ID", StringDomain(boost::uuids::to_string(uuid))));
            LstsReactor::postGoal(g);
          }
          ss_debug_log<<"Start: "<<m_start_time<<"- Now: "<<now()<<"\n";
          if (m_start_time <= now() && m_start_time != boost::posix_time::not_a_date_time)
          {
            ss_debug_log<<"--- Starting Plume Tracker now! ---\n";
            sendPlumeTrackerGoal();
            m_start_time = boost::posix_time::not_a_date_time;
          }
          // If plume tracker has taken over control transition and clear old variables
          if (!m_tracker_controlled)
          {
            e_exec_state = EXEC;
            // Clear variables for coordination since they need to be redone when the AUVs surface again
            m_initial_time = boost::posix_time::not_a_date_time;
            m_stop_sending_goals = false;
            v_team.clear();
          }
        }
          break;
        case EXEC:
        {
          static int timeCount = 0;
          ss_debug_log<<"Counter: "<<timeCount<<"\n";
          if (m_tracker_controlled && timeCount<=10)
            timeCount++;
          else if (m_tracker_controlled && timeCount>10) {
            timeCount = 0;
            e_exec_state = JOINING;
            double dist = WGS84::distance(m_lastPosition.lat, m_lastPosition.lon, 0, going_lat, going_lon, 0);
            if (dist > 5 )
              moving_locations = true;
          }
        }
          break;
        default:
          ss_debug_log<<"There is a problem this state is incorrect for this reactor\n";
      }
    }
    
    void 
    CoordinationReactor::handleRequest(const goal_id& goal)
    {
      // Leader handles goals from the followers
      if (!m_trex_control)
      {
        m_debug_log << "TREX is not controlling the vehicle so won't handle this request!";
        return;
      }
      else if (!m_leader_control)
      {
        m_debug_log << "This is not the leader of coordiation so won't handle this request!";
        return;
      }

      // Make a local copy to increase my reference counter instead of accessing the raw pointer directly !!!!!
      goal_id g = goal;
      m_debug_log << "Processing Goal: ";
      if ( g->predicate() == exec_state_names[JOINING])
      {
        m_debug_log << "Joining \n";
        // Calculating the latency for the round trip communications
        boost::posix_time::ptime t = tickToTime(this->getCurrentTick());
        std::string time = g->getAttribute("Time").domain().getStringSingleton();;
        boost::posix_time::ptime followertime = boost::posix_time::time_from_string(time);
        boost::posix_time::time_duration td = t - followertime;
        
        bool add_teammate = true;
        for (std::vector<Teammate>::iterator it = v_team.begin();
               it != v_team.end();
               ++it)
        {
          if ((*it).id == g->getAttribute("ID").domain().getStringSingleton())
            add_teammate = false;
        }
        
        if (add_teammate)
        {
          Teammate teammate;
          teammate.id = g->getAttribute("ID").domain().getStringSingleton();
          teammate.latency = td + boost::posix_time::duration_from_string(g->getAttribute("Latency").domain().getStringSingleton());
          teammate.pos.lat = g->getAttribute("Lat").domain().getTypedSingleton<double,true>();
          teammate.pos.lon = g->getAttribute("Lon").domain().getTypedSingleton<double,true>();
          teammate.received = false;
          v_team.push_back(teammate);
        }
        else
        {
          m_debug_log << "Already added \n";
        }
      }
      else if (g->predicate() == exec_state_names[GOING])
      {
        m_debug_log << "Going \n";
        e_exec_state = GOING;
        std::string id = g->getAttribute("ID").domain().getStringSingleton();
        for (std::vector<Teammate>::iterator it = v_team.begin();
               it != v_team.end();
               ++it)
        {
          if ((*it).id == id)
            (*it).received = true;
          else
            m_debug_log<<"Not equal"<<(*it).id<<" = "<<id<<"\n";
        }
      }
      else
      {
        m_debug_log<< "Request is not valid: " << g->predicate() <<"\n";
      }
    }
    
    void
    CoordinationReactor::notify(TREX::transaction::Observation const &obs)
    {
      // This is only used by the follower
      if(!m_leader_control)
        followerHandleNotify(obs);
      
      // Both the leader and follower receive obs from the timelines they subscribed to
      if (s_position_tl == obs.object())
      {
        if (obs.predicate() == "Position")
        {
          m_lastPosition.lat = obs.getAttribute("latitude").domain().getTypedSingleton<double,true>();
          m_lastPosition.lon = obs.getAttribute("longitude").domain().getTypedSingleton<double,true>();
        }
      }
      else if (s_depth_tl == obs.object())
      {
        if (obs.predicate() == "Value")
        {
          m_lastPosition.depth = obs.getAttribute("value").domain().getTypedSingleton<double,true>();
          m_depth_log << m_lastPosition.depth << "\n";
        }
      }
      else if (s_reference_tl == obs.object())
      {
        if (obs.predicate() == "Going")
        {
          going_lat = obs.getAttribute("latitude").domain().getTypedSingleton<double,true>();
          going_lon = obs.getAttribute("longitude").domain().getTypedSingleton<double,true>();
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
      else if (s_plume_tl == obs.object())
      {
        if(obs.predicate() == s_plume_unknown)
          e_plume_state = PLUME::UNKNOWN;
        else if(obs.predicate() == s_plume_inside)
        {
          e_plume_state = PLUME::INSIDE;
          //plume_lat = obs.getAttribute("latitude").domain().getTypedSingleton<double,true>();
          //plume_lon = obs.getAttribute("longitude").domain().getTypedSingleton<double,true>();
        }
        else if(obs.predicate() == s_plume_outside)
        {
          e_plume_state = PLUME::OUTSIDE;
        }
      }
      else if (s_plumetracker_tl == obs.object())
      {
        if(obs.predicate() == "Controlled")
        {
          m_tracker_controlled = true;
          if(obs.hasAttribute("latitude") && obs.hasAttribute("longitude"))
          {
            next_lat = obs.getAttribute("latitude").domain().getTypedSingleton<double,true>();
            next_lon = obs.getAttribute("longitude").domain().getTypedSingleton<double,true>();
          }
        }
        else
          m_tracker_controlled = false;
      }
    }
    
    void 
    CoordinationReactor::followerHandleNotify(TREX::transaction::Observation const &obs)
    {
      // This is only used by the follower
      // If the tracker is controlled then we will process messages otherwise the two
      // AUVs should not be communicating
      if (s_shared_tl == obs.object() && m_tracker_controlled)
      {
        if (obs.predicate() == exec_state_names[INITIAL])
        {
          // Grab the initial time to send a joining message back
          std::string time = obs.getAttribute("Time").domain().getStringSingleton();
          m_initial_time = boost::posix_time::time_from_string(time);
        } 
        else if (obs.predicate() == exec_state_names[JOINING])
        {
          if(obs.hasAttribute(boost::uuids::to_string(uuid)))
          {
            // Grab starting time for AUV's uuid
            std::string time = obs.getAttribute(boost::uuids::to_string(uuid)).domain().getStringSingleton();
            m_start_time = boost::posix_time::time_from_string(time);
          }
        }
        else if (obs.predicate() == exec_state_names[GOING])
        {
          // Stop sending goals because the joining has been received
          m_stop_sending_goals = true;
        }
      }
    }
    
    void 
    CoordinationReactor::leaderPlanningInsideGoingOut()
    {
      Observation obs = Observation(s_shared_tl, exec_state_names[e_exec_state]);
      for (std::vector<Teammate>::iterator it = v_team.begin();
               it != v_team.end();
               ++it)
      {
        double dist = WGS84::distance(m_lastPosition.lat, m_lastPosition.lon, 0, (*it).pos.lat, (*it).pos.lon, 0);
        // TODO Make the AUV move if it is too far
        if (dist < 1000)
        {
          if (m_start_time == boost::posix_time::not_a_date_time || m_start_time > now())
            m_start_time = now() + ((*it).latency*2) + boost::posix_time::seconds(wait_secs); 
          std::string time = boost::posix_time::to_simple_string(m_start_time + boost::posix_time::seconds(trail_behind_time));
          obs.restrictAttribute((*it).id, StringDomain(time));
        }
        else
          m_debug_log<<"ERROR: Too far away ("<<dist<<")\n";
      }
      TeleoReactor::postObservation(obs);
    }

    void 
    CoordinationReactor::leaderPlanningOutsideGoingIn()
    {
      Observation obs = Observation(s_shared_tl, exec_state_names[e_exec_state]);
      for (std::vector<Teammate>::iterator it = v_team.begin();
               it != v_team.end();
               ++it)
      {
        double dist = WGS84::distance(m_lastPosition.lat, m_lastPosition.lon, 0, (*it).pos.lat, (*it).pos.lon, 0);
        // TODO Make the AUV move if it is too far
        if (dist < 20)
        {
          if (m_start_time == boost::posix_time::not_a_date_time || m_start_time > now())
            m_start_time = now() + ((*it).latency*2) + boost::posix_time::seconds(wait_secs); 
          std::string time = boost::posix_time::to_simple_string(m_start_time);
          obs.restrictAttribute((*it).id, StringDomain(time));
        }
        else
        {
          m_debug_log<<"ERROR: Too far away ("<<dist<<")\n";
          double angRads = Math::Angles::radians(90);
          double offsetX = std::cos(angRads) * 10;
          double offsetY = std::sin(angRads) * 10;
          double moving_lat = (*it).pos.lat;
          double moving_lon = (*it).pos.lon;
          WGS84::displace(offsetX, offsetY, &moving_lat, &moving_lon);
          sendReferenceGoal(moving_lat, moving_lon, 0);
          return;
        }
      }
      TeleoReactor::postObservation(obs);
    }
    
    void 
    CoordinationReactor::sendReferenceGoal(const double& lat, const double& lon, const double& z, const double& speed)
    {
      ss_debug_log<<"+++++ Sending Reference Goal +++++"<<"\n";
      Goal g(s_reference_tl, "Going");
      
      g.restrictAttribute(Variable("latitude", FloatDomain(lat)));
      g.restrictAttribute(Variable("longitude", FloatDomain(lon)));
      g.restrictAttribute(Variable("z", FloatDomain(z)));
      g.restrictAttribute(Variable("speed", FloatDomain(speed)));

      LstsReactor::postGoal(g);
    }
    
    void 
    CoordinationReactor::sendPlumeTrackerGoal()
    {
      ss_debug_log<<"+++++ Sending Plume Tracker Goal +++++"<<"\n";
      if (e_plume_state == PLUME::INSIDE)
      {
        ss_debug_log<<"InsideGoingOut\n";
        Goal g(s_plumetracker_tl, "InsideGoingOut");
        LstsReactor::postGoal(g);
      }
      else if (e_plume_state == PLUME::OUTSIDE)
      {
        ss_debug_log<<"OutsideGoingIn\n";
        Goal g(s_plumetracker_tl, "OutsideGoingIn");
        LstsReactor::postGoal(g);
      }
      else 
      {
        ss_debug_log<<"Not correct!\n";
      }
    }

    
    void 
    CoordinationReactor::uniqueDebugPrint(TICK cur)
    {
      std::string debug_log = ss_debug_log.str();
      if (s_past_log != debug_log) 
      {
        s_past_log = debug_log;
        m_debug_log << s_past_log <<"\n";
      }
      ss_debug_log.str(std::string());
    }
    
    inline boost::posix_time::ptime 
    CoordinationReactor::now()
    {
      return tickToTime(this->getCurrentTick());
    }

    CoordinationReactor::~CoordinationReactor()
    {
      // TODO Auto-generated destructor stub
      m_debug_log.close();
      m_depth_log.close();
    }
  }
}