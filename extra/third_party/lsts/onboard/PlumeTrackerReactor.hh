/*
 * PlumeTrackerReactor.hh
 *
 *  Created on: May 22, 2017
 *      Author: pcooksey
 */

#ifndef PLUMETRACKERREACTOR_HH_
#define PLUMETRACKERREACTOR_HH_

# include <trex/transaction/TeleoReactor.hh>
# include <trex/utils/Plugin.hh>
# include <trex/utils/LogManager.hh>
# include <trex/domain/IntegerDomain.hh>
# include <trex/domain/FloatDomain.hh>
# include <trex/domain/StringDomain.hh>
# include <trex/domain/BooleanDomain.hh>
# include <trex/domain/EnumDomain.hh>
# include <trex/utils/asio_fstream.hh>

# include "../shared/LstsReactor.hh"
# include "PlumeTrackingVariables.hh"

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
    
    namespace PLUME 
    { 
      enum PLUME_STATE {UNKNOWN, INSIDE, OUTSIDE};
    };
    
    namespace PlumeTrackerReactorState 
    {
      enum EXEC_STATE {IDLE, CONTROLLED, INSIDE_GOINGOUT, OUTSIDE, OUTSIDE_GOINGIN, INSIDE, SURFACING};
      const char *exec_state_names[] = { "Idle", "Controlled", "InsideGoingOut", "Outside", "OutsideGoingIn", "Inside", "Surfacing" };
    }

    class PlumeTrackerReactor : public LstsReactor
    {
    public:
      PlumeTrackerReactor(TeleoReactor::xml_arg_type arg);
      PlumeTrackerReactorState::EXEC_STATE e_exec_state;
      PLUME::PLUME_STATE e_plume_state;
      
      TREX::transaction::Observation m_lastControl;
      Position m_lastPosition;
      
      utils::async_ofstream m_debug_log;
      
      void handleInit();
      void handleTickStart();
      bool synchronize();
      void notify(TREX::transaction::Observation const &obs);
      void handleRequest(TREX::transaction::goal_id const &goal);

      virtual
      ~PlumeTrackerReactor();
      
    private:
      // If TREX is in control
      bool m_trex_control;
      // If PlumeTracker is in control of itself
      bool m_tracker_control;
      // If yoyo state is IDLE
      bool yoyo_done;
      
      double angle;
      
      double tracking_lat, tracking_lon;
      double plume_lat, plume_lon;
      double plume_edge_lat, plume_edge_lon;
      
      std::vector<Symbol> yoyo_states;
      
      std::stringstream ss_debug_log;
      std::string s_past_log;
      
      static utils::Symbol const s_control_tl;
      static utils::Symbol const s_position_tl;
      static utils::Symbol const s_reference_tl;
      
      static utils::Symbol const s_plume_tl;
      static utils::Symbol const s_yoyo_tl;
      static utils::Symbol const s_yoyo_state_tl;
      static utils::Symbol const s_depth_tl;
      
      // Plume detection
      static utils::Symbol const s_plume_unknown;
      static utils::Symbol const s_plume_inside;
      static utils::Symbol const s_plume_outside;
      
      // Provided timeline
      static utils::Symbol const s_plumetracker_tl;
      
      bool goingOut();
      bool goingIn();
      
      void sendYoYoGoal(const double &lat, const double &lon);
      void sendReferenceGoal(const double &lat, 
                             const double &lon, 
                             const double& z=surface_depth, 
                             const double& speed=yoyo_speed);
      
      void postObservation();
      void uniqueDebugPrint(TICK cur);
      
    };


  }
}

#endif /* PLUMETRACKERREACTOR_HH_ */
