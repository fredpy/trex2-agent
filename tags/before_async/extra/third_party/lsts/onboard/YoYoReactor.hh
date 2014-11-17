/*
 * YoYoReactor.hh
 *
 *  Created on: Jun 11, 2013
 *      Author: zp
 */

#ifndef YOYOREACTOR_HH_
#define YOYOREACTOR_HH_

# include <trex/transaction/reactor.hh>
# include <trex/utils/Plugin.hh>
# include <trex/utils/log_manager.hh>
# include <trex/domain/int_domain.hh>
# include <trex/domain/float_domain.hh>
# include <trex/domain/string_domain.hh>
# include <trex/domain/boolean_domain.hh>
# include <trex/domain/enum_domain.hh>

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
      int tick;
    } ReferenceRequest;

    enum EXEC_STATE {IDLE, DESCEND, ASCEND, SURFACE, DONE};

    class YoYoReactor : public LstsReactor
    {
    public:
      YoYoReactor(reactor::xml_arg_type arg);
      EXEC_STATE state;
      TREX::transaction::token m_lastRefState;
      TREX::transaction::token m_lastControl;
      TREX::transaction::token m_lastReference;
      TREX::transaction::token m_lastPosition;

      double m_lat, m_lon, m_minz, m_maxz, m_speed, m_cmdz;
      int m_time_at_surface;
      ReferenceRequest m_lastSentRef, m_lastSeenRef;

      void handle_init();
      void handle_tick_start();
      bool synchronize();
      void handle_request(TREX::transaction::token_id const &g);
      void handle_recall(TREX::transaction::token_id const &g);
      void notify(TREX::transaction::token const &obs);

      void requestReference(double lat, double lon, double speed, double z);

      void printReference(ReferenceRequest req);

      virtual
      ~YoYoReactor();
      
    private:
      static utils::symbol const s_trex_pred;
      static utils::symbol const s_exec_pred;
      
      static utils::symbol const s_reference_tl;
      static utils::symbol const s_refstate_tl;
      static utils::symbol const s_control_tl;
      static utils::symbol const s_position_tl;
      static utils::symbol const s_yoyo_tl;
      static utils::symbol const s_yoyo_state_tl;
    };


  }
}

#endif /* YOYOREACTOR_HH_ */