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


    class ImcAdapter
    {
    public:
      ImcAdapter();
      Observation gpsFixObservation(GpsFix * msg);
      Observation estimatedStateObservation(EstimatedState * msg);
      Observation followRefStateObservation(FollowRefState * msg);
      Observation vehicleStateObservation(VehicleState * msg);

      virtual
      ~ImcAdapter();
    };
  }
}


#endif /* IMCADAPTER_HH_ */
