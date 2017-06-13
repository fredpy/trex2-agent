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
      class tick_proxy {
      public:
        typedef graph::date_type     date_type;
        typedef graph::duration_type duration_type;
        typedef TICK                 tick_type;
        
        virtual ~tick_proxy() {}
        
        virtual tick_type current_tick() =0;
        virtual date_type tick_to_date(tick_type const &tck) =0;
        virtual tick_type date_to_tick(date_type const &date) =0;
        virtual std::string date_str(tick_type const &tck) =0;
        virtual std::string duration_str(tick_type const &tck) =0;
        virtual tick_type as_date(std::string const &date) =0;
        virtual tick_type as_duration(std::string const &date) =0;
        
        
        std::string date_str(IntegerDomain::bound const &bound) {
          return date_str(bound.value());
        }
        std::string duration_str(IntegerDomain::bound const &bound) {
          return duration_str(bound.value());
        }
        
        virtual utils::log::stream log(utils::Symbol const &kind) =0;
      
      protected:
        tick_proxy() {}
        
      };
      
      
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

      void
      setPlatformId(int platf_id);

      void setReactorGraph(graph const &g);
      void set_proxy(tick_proxy *p) {
        m_cvt.reset(p);
      }


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

      //@brief Trygve Modification. Declaring method for ctdObservation
      Observation ctdObservation(Conductivity * msg_c, Temperature * msg_t, Depth * msg_d, Salinity * msg_s);

      //@brief Translates TrexToken messages into a generic observation
      Observation genericObservation(TICK &date, TrexToken * msg);

      //@brief Translates TrexToken messages into a goal
      Goal genericGoal(TrexToken * msg, bool restrict_to_future=true);

      //@brief Translates TrexToken messages into a generic observation
      Observation announceObservation(Announce * msg);

      void asImcMessage(TICK date, Predicate const &obs, TrexToken * result);

      void fillInExtraGoalAttributes(goal_id &goal, TrexToken * result);

      virtual
      ~ImcAdapter();
      
      tick_proxy &time_conv() const {
        return *m_cvt;
      }
      utils::log::stream log(utils::Symbol const &kind=utils::log::null) {
        return time_conv().log(kind);
      }
      
      
    private:
      const int c_imc_header_length;
      const int c_max_iridium_payload_length;
      bool variableToImc(Variable const &v, TrexAttribute * attr);
      void setAttribute(Predicate &pred, TrexAttribute const &attr);
      static int m_trex_id, m_platf_id, m_iridium_req;
      UDPSocket sock_send, sock_receive;
      uint8_t* bfr;
      //IOMultiplexing iom;
      DUNE::IO::Poll m_poll;
      ImcMessenger * messenger;
      
      UNIQ_PTR<tick_proxy> m_cvt;
      //graph const * m_graph;
    };
  }
}


#endif /* IMCADAPTER_HH_ */
