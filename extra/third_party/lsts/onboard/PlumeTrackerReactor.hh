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

using namespace TREX::transaction;
using namespace TREX::utils;

using DUNE_NAMESPACES;

#define SIMULATE_DATA 0


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
      double lat, lon, speed;
    } Position;
    
    enum EXEC_STATE {IDLE, INSIDE_GOINGOUT, OUTSIDE, OUTSIDE_GOINGIN, INSIDE};

    class PlumeTrackerReactor : public LstsReactor
    {
    public:
      PlumeTrackerReactor(TeleoReactor::xml_arg_type arg);
      EXEC_STATE state;
      
      TREX::transaction::Observation m_lastControl;
      Position m_lastPosition;
      
      utils::async_ofstream m_debug_log;
      utils::async_ofstream m_depth_log;
      
      void handleInit();
      void handleTickStart();
      bool synchronize();
      void notify(TREX::transaction::Observation const &obs);

      virtual
      ~PlumeTrackerReactor();
      
    private:
      bool m_trex_control;
      // True(1): inside, False(0): outside, Negative(-1): Unknown
      double b_inside_plume;
      double b_last_inside_plume;
      
      double angle;
      
      double tracking_lat, tracking_lon;
      double plume_lat, plume_lon;
      double plume_edge_lat, plume_edge_lon;
      
      std::vector<Symbol> yoyo_states;
      
      static utils::Symbol const s_trex_pred;
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
      
      bool goingOut();
      bool goingIn();
      
      void sendYoYoGoal(const double &lat, const double &lon);
      void sendReferenceGoal(const double &lat, const double &lon);
      
    };


  }
}

#endif /* PLUMETRACKERREACTOR_HH_ */
