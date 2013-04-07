/*
 * ImcAdapter.hh
 *
 *  Created on: Apr 4, 2013
 *      Author: zp
 */

#ifndef IMCADAPTER_HH_
#define IMCADAPTER_HH_

# include <DUNE/DUNE.hpp>
# include <trex/transaction/TeleoReactor.hh>
# include <trex/domain/FloatDomain.hh>
# include <trex/domain/BooleanDomain.hh>
# include <trex/domain/StringDomain.hh>
# include "Platform.hh"

using DUNE_NAMESPACES;
using namespace TREX::transaction;

namespace TREX {
  namespace LSTS {

    /** @brief IMC to TREX translator
     *
     * This class is used to translate incoming IMC data into TREX observations
     *
     * @author Jose Pinto <zepinto@gmail.com>
     * @ingroup lsts
     */
    class ImcAdapter
    {
    public:
      ImcAdapter();
      //@brief Translates VehicleMedium messages into "medium" timeline observations
      Observation vehicleMediumObservation(VehicleMedium * msg);

      //@brief Translates EstimatedState messages into "estate" timeline observations
      Observation estimatedStateObservation(EstimatedState * msg);

      //@brief Translates FollowRefState messages into "frefstate" timeline observations
      Observation followRefStateObservation(FollowRefState * msg);

      //@brief Translates PlanControlState messages into "vstate" timeline observations
      Observation planControlStateObservation(PlanControlState * msg);

      //@brief Translates OperationalLimits messages into "oplimits" timeline observations
      Observation opLimitsObservation(OperationalLimits * msg);

      virtual
      ~ImcAdapter();
    };
  }
}


#endif /* IMCADAPTER_HH_ */
