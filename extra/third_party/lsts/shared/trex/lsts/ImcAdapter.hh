/*
 * ImcAdapter.hh
 *
 *  Created on: Apr 4, 2013
 *      Author: zp
 */
#pragma once
#include <trex/domain/BooleanDomain.hh>
#include <trex/domain/EnumDomain.hh>
#include <trex/domain/FloatDomain.hh>
#include <trex/domain/IntegerDomain.hh>
#include <trex/domain/StringDomain.hh>
#include <trex/transaction/TeleoReactor.hh>
#include <trex/utils/LogManager.hh>
#include <trex/utils/Plugin.hh>
#include <trex/utils/Symbol.hh>

#define DUNE_HEADER <DUNE/DUNE.hpp>
#include "bits/dune_include.hh"
#define DUNE_HEADER <DUNE/Network.hpp>
#include "bits/dune_include.hh"
#define DUNE_HEADER <DUNE/Network/Fragments.hpp>
#include "bits/dune_include.hh"

#include "EuropaExtensions.hh"
#include "ImcMessenger.hh"

//#define ASYNC_COMMS

namespace TREX::LSTS {

/** @brief IMC to TREX translator
 *
 * This class is used to translate incoming IMC data into TREX observations
 *
 * @author Jose Pinto <zepinto@gmail.com>
 * @ingroup lsts
 */
class ImcAdapter {
public:
  class tick_proxy {
  public:
    typedef transaction::graph::date_type date_type;
    typedef transaction::graph::duration_type duration_type;
    typedef transaction::TICK tick_type;

    virtual ~tick_proxy() {}

    virtual tick_type current_tick() = 0;
    virtual date_type tick_to_date(tick_type const &tck) = 0;
    virtual tick_type date_to_tick(date_type const &date) = 0;
    virtual std::string date_str(tick_type const &tck) = 0;
    virtual std::string duration_str(tick_type const &tck) = 0;
    virtual tick_type as_date(std::string const &date) = 0;
    virtual tick_type as_duration(std::string const &date) = 0;

    std::string date_str(transaction::IntegerDomain::bound const &bound) {
      return date_str(bound.value());
    }
    std::string duration_str(transaction::IntegerDomain::bound const &bound) {
      return duration_str(bound.value());
    }

    virtual utils::log::stream log(utils::Symbol const &kind) = 0;

  protected:
    tick_proxy() {}
  };

  ImcAdapter();

  bool bindAsynchronous(int port);
  bool bindSynchronous(int port);

  bool bind(int port) {
#ifdef ASYNC_COMMS
    return bindAsynchronous(port);
#else
    return bindSynchronous(port);
#endif
  }

  bool unbindAsynchronous();
  bool unbindSynchronous();

  bool unbind() {
#ifdef ASYNC_COMMS
    return unbindAsynchronous();
#else
    return unbindSynchronous();
#endif
  }

  bool sendAsynchronous(DUNE::IMC::Message *msg, std::string address, int port);
  bool sendSynchronous(DUNE::IMC::Message *msg, std::string address, int port);

  bool send(DUNE::IMC::Message *msg, std::string address, int port) {
#ifdef ASYNC_COMMS
    return sendAsynchronous(msg, address, port);
#else
    return sendSynchronous(msg, address, port);
#endif
  }

  DUNE::IMC::Message *pollAsynchronous();
  DUNE::IMC::Message *pollSynchronous();

  DUNE::IMC::Message *poll() {
#ifdef ASYNC_COMMS
    return pollAsynchronous();
#else
    return pollSynchronous();
#endif
  }

  bool sendViaIridium(DUNE::IMC::Message *msg, const std::string address, int port);

  void setTrexId(int trex_id);

  void setPlatformId(int platf_id);

  void setReactorGraph(transaction::graph const &g);
  void set_proxy(tick_proxy *p) { m_cvt.reset(p); }

  //@brief Translates VehicleMedium messages into "medium" timeline observations
  transaction::Observation vehicleMediumObservation(DUNE::IMC::VehicleMedium *msg);

  //@brief Translates EstimatedState messages into "estate" timeline
  //observations
  transaction::Observation estimatedStateObservation(DUNE::IMC::EstimatedState *msg);

  //@brief Translates FollowRefState messages into "frefstate" timeline
  //observations
  transaction::Observation followRefStateObservation(DUNE::IMC::FollowRefState *msg);

  //@brief Translates PlanControlState messages into "vstate" timeline
  //observations
  transaction::Observation planControlStateObservation(DUNE::IMC::PlanControlState *msg);

  //@brief Translates OperationalLimits messages into "oplimits" timeline
  //observations
  transaction::Observation opLimitsObservation(DUNE::IMC::OperationalLimits *msg);

  //@brief Translates TrexToken messages into a generic observation
  transaction::Observation genericObservation(transaction::TICK &date, DUNE::IMC::TrexToken *msg);

  //@brief Translates TrexToken messages into a goal
  transaction::Goal genericGoal(DUNE::IMC::TrexToken *msg, bool restrict_to_future = true);

  //@brief Translates TrexToken messages into a generic observation
  transaction::Observation announceObservation(DUNE::IMC::Announce *msg);

  void asImcMessage(transaction::TICK date, transaction::Predicate const &obs, DUNE::IMC::TrexToken *result);

  void fillInExtraGoalAttributes(transaction::goal_id &goal, DUNE::IMC::TrexToken *result);

  virtual ~ImcAdapter();

  tick_proxy &time_conv() const { return *m_cvt; }
  utils::log::stream log(utils::Symbol const &kind = utils::log::null) {
    return time_conv().log(kind);
  }

private:
  const int c_imc_header_length;
  const int c_max_iridium_payload_length;
  bool variableToImc(transaction::Variable const &v, DUNE::IMC::TrexAttribute *attr);
  void setAttribute(transaction::Predicate &pred, DUNE::IMC::TrexAttribute const &attr);
  static int m_trex_id, m_platf_id, m_iridium_req;
  DUNE::Network::UDPSocket sock_send, sock_receive;
  uint8_t *bfr;
  // IOMultiplexing iom;
  DUNE::IO::Poll m_poll;
  ImcMessenger *messenger;

  std::unique_ptr<tick_proxy> m_cvt;
  // graph const * m_graph;
};
} // namespace TREX::LSTS
