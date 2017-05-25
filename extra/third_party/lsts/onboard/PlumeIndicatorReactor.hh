/*
 * PlumeIndicatorReactor.hh
 *
 *  Created on: May 18, 2017
 *      Author: pcooksey
 */

#ifndef PLUMEINICATORREACTOR_HH_
#define PLUMEINICATORREACTOR_HH_

# include <trex/transaction/TeleoReactor.hh>
# include <trex/utils/Plugin.hh>
# include <trex/utils/LogManager.hh>
# include <trex/domain/IntegerDomain.hh>
# include <trex/domain/FloatDomain.hh>
# include <trex/domain/StringDomain.hh>
# include <trex/domain/BooleanDomain.hh>
# include <trex/domain/EnumDomain.hh>

# include <vector>
# include <numeric>

# include "../shared/LstsReactor.hh"

# include <DUNE/DUNE.hpp>
# include <DUNE/Math/Angles.hpp>
# include <DUNE/Coordinates/WGS84.hpp>

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
      double lat, lon;
    } Position;
    
    enum STATE {UNKNOWN, INSIDE, OUTSIDE};
    
    const int sample_size = 3;
    const double depth_for_plume = 1;
    const double avg_below = 30;

    class PlumeIndicatorReactor : public LstsReactor
    {
    public:
      PlumeIndicatorReactor(TeleoReactor::xml_arg_type arg);
      STATE state;
      
      TREX::transaction::Observation m_lastControl;
      Position m_lastPosition;
      
      void handleInit();
      void handleTickStart();
      bool synchronize();
      void notify(TREX::transaction::Observation const &obs);

      virtual
      ~PlumeIndicatorReactor();
      
    private:
      std::vector<double> v_salinity, v_temperature, v_depth;
      std::vector<Position> v_positions; 
      double m_last_salinity, m_last_temperature, m_last_depth;
      
      bool m_trex_control;
      
      static utils::Symbol const s_trex_pred;
      static utils::Symbol const s_control_tl;
      
      static utils::Symbol const s_depth_tl;
      static utils::Symbol const s_temperature_tl;
      static utils::Symbol const s_salinity_tl;
      static utils::Symbol const s_position_tl;
      static utils::Symbol const s_plumeindicator_tl;
      
      void getCTDData(TREX::transaction::Observation const &obs, 
                      std::vector<double>& vec, 
                      double& value);
    };


  }
}

#endif /* PLUMEINICATORREACTOR_HH_ */
