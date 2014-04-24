/*
 * ImcAdapter.hh
 *
 *  Created on: Apr 4, 2013
 *      Author: zp
 */

#ifndef IMCADAPTER_HH_
#define IMCADAPTER_HH_

# include <DUNE/DUNE.hpp>
# include <trex/transaction/reactor.hh>
# include <trex/domain/float_domain.hh>
# include <trex/domain/boolean_domain.hh>
# include <trex/domain/string_domain.hh>
# include <trex/domain/int_domain.hh>
# include <trex/domain/enum_domain.hh>
# include <trex/utils/Plugin.hh>
# include <trex/utils/log_manager.hh>
# include <trex/utils/symbol.hh>
# include <DUNE/Network.hpp>
# include <DUNE/Network/Fragments.hpp>
# include "EuropaExtensions.hh"
# include "ImcMessenger.hh"

using DUNE_NAMESPACES;
using namespace TREX::transaction;
using namespace TREX::utils;

//#define ASYNC_COMMS

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

      bool bindAsynchronous(int port);
      bool bindSynchronous(int port);

      bool bind(int port)
      {
#ifdef ASYNC_COMMS
        return bindAsynchronous(port);
#else
        return bindSynchronous(port);
#endif
      }

      bool unbindAsynchronous();
      bool unbindSynchronous();

      bool unbind()
      {
#ifdef ASYNC_COMMS
        return unbindAsynchronous();
#else
        return unbindSynchronous();
#endif
      }

      bool sendAsynchronous(Message * msg, std::string address, int port);
      bool sendSynchronous(Message * msg, std::string address, int port);

      bool send(Message * msg, std::string address, int port)
      {
#ifdef ASYNC_COMMS
        return sendAsynchronous(msg, address, port);
#else
        return sendSynchronous(msg, address, port);
#endif
      }

      Message * pollAsynchronous();
      Message * pollSynchronous();

      Message * poll()
      {
#ifdef ASYNC_COMMS
        return pollAsynchronous();
#else
        return pollSynchronous();
#endif
      }

      bool
      sendViaIridium(Message * msg, const std::string address, int port);

      void
      setTrexId(int trex_id);

      void setReactorGraph(graph const &g);


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

      //@brief Translates TrexToken messages into a goal
      Goal genericGoal(TrexToken * msg);

      //@brief Translates TrexToken messages into a generic observation
      Observation announceObservation(Announce * msg);

      void asImcMessage(Predicate const &obs, TrexToken * result);

      void fillInExtraGoalAttributes(goal_id &goal, TrexToken * result);

      virtual
      ~ImcAdapter();

    private:
      const int c_imc_header_length;
      const int c_max_iridium_payload_length;
      void variableToImc(var const &v, TrexAttribute * attr);
      void setAttribute(Predicate &pred, TrexAttribute const &attr);
      int m_trex_id;
      UDPSocket sock_send, sock_receive;
      uint8_t* bfr;
      //IOMultiplexing iom;
      DUNE::IO::Poll m_poll;
      ImcMessenger * messenger;
      graph const * m_graph;
    };
  }
}


#endif /* IMCADAPTER_HH_ */
