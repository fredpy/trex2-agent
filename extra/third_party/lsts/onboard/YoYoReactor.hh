/*
 * YoYoReactor.hh
 *
 *  Created on: Jun 11, 2013
 *      Author: zp
 */

#ifndef YOYOREACTOR_HH_
#define YOYOREACTOR_HH_

# include <trex/transaction/TeleoReactor.hh>
# include <trex/utils/Plugin.hh>
# include <trex/utils/LogManager.hh>
# include <trex/domain/IntegerDomain.hh>
# include <trex/domain/FloatDomain.hh>
# include <trex/domain/StringDomain.hh>
# include <trex/domain/BooleanDomain.hh>
# include <trex/domain/EnumDomain.hh>

# include "../shared/LstsReactor.hh"

# include <DUNE/DUNE.hpp>
# include <DUNE/Math/Angles.hpp>
# include <DUNE/Coordinates/WGS84.hpp>

using namespace TREX::transaction;
using namespace TREX::utils;

using DUNE_NAMESPACES;


namespace TREX {
  /** @brief yoyo plug-in
   *
   * This namespace is shared by all LSTS reactors/plugins
   * @ingroup lsts
   *
   * @author Jose Pinto <zepinto@gmail.com>
   */
  namespace LSTS {

    typedef struct {
      double lat, lon;
      double z;
      double speed;
    } ReferenceRequest;

    enum EXEC_STATE {IDLE, DESCEND, ASCEND, SURFACE, DONE};

    class YoYoReactor : public LstsReactor
    {
    public:
      YoYoReactor(TeleoReactor::xml_arg_type arg);
      EXEC_STATE state;
      TREX::transaction::Observation m_lastRefState;
      TREX::transaction::Observation m_lastControl;
      TREX::transaction::Observation m_lastReference;
      TREX::transaction::Observation m_lastPosition;

      double m_lat, m_lon, m_minz, m_maxz, m_speed;
      int m_time_at_surface;
      ReferenceRequest m_lastSentRef, m_lastSeenRef;

      void handleInit();
      void handleTickStart();
      bool synchronize();
      void handleRequest(TREX::transaction::goal_id const &g);
      void handleRecall(TREX::transaction::goal_id const &g);
      void notify(TREX::transaction::Observation const &obs);

      void requestReference(double lat, double lon, double speed, double z);

      virtual
      ~YoYoReactor();
      
    private:
      static utils::Symbol const s_trex_pred;
      static utils::Symbol const s_exec_pred;
      
      static utils::Symbol const s_reference_tl;
      static utils::Symbol const s_refstate_tl;
      static utils::Symbol const s_control_tl;
      static utils::Symbol const s_position_tl;
      
      static utils::Symbol const s_yoyo_tl;
    };


  }
}

#endif /* YOYOREACTOR_HH_ */
