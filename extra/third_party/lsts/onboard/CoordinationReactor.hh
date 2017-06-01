/*
 * CoordinationReactor.hh
 *
 *  Created on: May 26, 2017
 *      Author: pcooksey
 */

#ifndef COORDINATIONREACTOR_HH_
#define COORDINATIONREACTOR_HH_

# include <trex/transaction/TeleoReactor.hh>
# include <trex/utils/Plugin.hh>
# include <trex/utils/LogManager.hh>
# include <trex/domain/IntegerDomain.hh>
# include <trex/domain/FloatDomain.hh>
# include <trex/domain/StringDomain.hh>
# include <trex/domain/BooleanDomain.hh>
# include <trex/domain/EnumDomain.hh>
# include <trex/utils/asio_fstream.hh>
# include <trex/utils/StringExtract.hh>

#include <boost/uuid/uuid.hpp>            // uuid class
#include <boost/uuid/uuid_generators.hpp> // generators
#include <boost/uuid/uuid_io.hpp>         // streaming operators etc.

# include "../shared/LstsReactor.hh"

# include <DUNE/DUNE.hpp>
# include <DUNE/Math/Angles.hpp>
# include <DUNE/Coordinates/WGS84.hpp>

# include <vector>
# include <sstream>

using namespace TREX::transaction;
using namespace TREX::utils;

using DUNE_NAMESPACES;

namespace TREX {
  /** @brief plume indicator plug-in
   *
   * This namespace is shared by all LSTS reactors/plugins
   * @ingroup lsts
   *
   * @author Philip Cooksey <pcooksey@andrew.cmu.edu>
   */
  namespace LSTS {
    
    typedef struct {
      double lat, lon, depth;
    } Position;
    
    typedef struct {
      std::string id;
      boost::posix_time::time_duration latency;
      Position pos;
      bool received;
    } Teammate;
    
    namespace PLUME 
    { 
      enum PLUME_STATE {UNKNOWN, INSIDE, OUTSIDE};
    };
    
    namespace CoordinationReactorState
    {
      enum EXEC_STATE {INITIAL, JOINING, GOING, EXEC};
      const char *exec_state_names[] = {"Initial", "Joining", "Going", "Exec"};
    }

    class CoordinationReactor : public LstsReactor
    {
    public:
      CoordinationReactor(TeleoReactor::xml_arg_type arg);
      CoordinationReactorState::EXEC_STATE e_exec_state;
      
      TREX::transaction::Observation m_lastControl;
      Position m_lastPosition;
      
      utils::async_ofstream m_debug_log;
      utils::async_ofstream m_depth_log;
      
      void handleInit();
      void handleTickStart();
      bool synchronize();
      void notify(TREX::transaction::Observation const &obs);
      void handleRequest(TREX::transaction::goal_id const &goal);
      void followerHandleNotify(TREX::transaction::Observation const &obs);

      virtual
      ~CoordinationReactor();
      
    private:
      // If TREX is in control
      bool m_trex_control;
      // If this reactor is the leader
      bool m_leader_control;
      bool m_stop_sending_goals;
      bool m_tracker_controlled;
      
      // The next point provided by the PlumeTrackerReactor
      double next_lat, next_lon;
      
      PLUME::PLUME_STATE e_plume_state;
      
      boost::uuids::uuid uuid;
      boost::posix_time::ptime m_initial_time;
      boost::posix_time::ptime m_start_time;
      boost::posix_time::time_duration m_decent_time;
      
      std::stringstream ss_debug_log;
      std::string s_past_log;
      
      std::vector<Teammate> v_team;
      
      static utils::Symbol const s_control_tl;
      static utils::Symbol const s_position_tl;
      static utils::Symbol const s_reference_tl;
      static utils::Symbol const s_plumetracker_tl;
      static utils::Symbol const s_plume_tl;
      
      // Shared timeline between vehicles 
      static utils::Symbol const s_shared_tl;
      
      // Plume detection
      static utils::Symbol const s_plume_unknown;
      static utils::Symbol const s_plume_inside;
      static utils::Symbol const s_plume_outside;
      
      void sendReferenceGoal(const double &lat, const double &lon, const double& z=0, const double& speed=1600);
      void sendPlumeTrackerGoal();
      
      void leaderPostObservation(TICK cur);
      void followerPostGoal(TICK cur);
      void uniqueDebugPrint(TICK cur);
      inline boost::posix_time::ptime now();
      
      void leaderPlanningInsideGoingOut(TREX::transaction::Observation &obs);
      void leaderPlanningOutsideGoingIn(TREX::transaction::Observation &obs);
      
    };


  }
}

#endif /* COORDINATIONREACTOR_HH_ */
