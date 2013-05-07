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
# include <trex/domain/IntegerDomain.hh>
# include <trex/domain/EnumDomain.hh>
# include <trex/utils/Plugin.hh>
# include <trex/utils/LogManager.hh>
# include <trex/utils/Symbol.hh>

# include "EuropaExtensions.hh"

using DUNE_NAMESPACES;
using namespace TREX::transaction;
using namespace TREX::utils;
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

      bool
      bind(int port);

      bool
      unbind();

      bool
      send(Message * msg, std::string address, int port);

      bool
      startDiscovery();

      void
      setTrexId(int trex_id);

      Message *
      poll(double timeout, bool discovery);

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

      //@brief Translates TrexToken messages into a generic observation
      Observation genericObservation(TrexToken * msg);

      //@brief Translates TrexToken messages into a generic observation
      Observation announceObservation(Announce * msg);

      void asImcMessage(Predicate const &obs, TrexToken * result);

      virtual
      ~ImcAdapter();

    private:
      void variableToImc(Variable const &v, TrexAttribute * attr);

      UDPSocket m_send, m_receive, m_discovery;
      IOMultiplexing m_iom, m_diom;
      uint8_t* m_bfr;
      Concurrency::Mutex m_mutex;
      int m_trex_id;
      bool m_bound;
    };
  }
}


#endif /* IMCADAPTER_HH_ */
