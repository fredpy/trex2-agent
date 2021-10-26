/*
 * ImcMessenger.hh
 *
 *  Created on: 30 September 2013
 *      Author: zp
 */
#pragma once

#include <atomic>

#include <trex/domain/BooleanDomain.hh>
#include <trex/domain/EnumDomain.hh>
#include <trex/domain/FloatDomain.hh>
#include <trex/domain/IntegerDomain.hh>
#include <trex/domain/StringDomain.hh>
#include <trex/transaction/TeleoReactor.hh>
#include <trex/utils/LogManager.hh>
#include <trex/utils/Plugin.hh>
#include <trex/utils/Symbol.hh>

#include <boost/thread/thread.hpp>

#include "EuropaExtensions.hh"

#define DUNE_HEADER <DUNE/DUNE.hpp>
#include "bits/dune_include.hh"


namespace TREX::LSTS {

class ImcMessenger {
public:
  using message_type = DUNE::IMC::Message;


  ImcMessenger();
  ImcMessenger(ImcMessenger const &) =delete;
  virtual ~ImcMessenger();

  void startListening(int bindPort);
  void stopListening();
  bool inboxEmpty();
  message_type *receive();
  void post(message_type *msg, int port, std::string addr);


private:
  struct SendRequest {
    message_type *msg;
    int port;
    std::string addr;
  };

  std::unique_ptr<std::thread> receiver{}, sender;
  std::atomic_bool m_listening{false}, m_sending{false};

  std::queue<message_type *> inbox;
  std::queue<SendRequest> outbox;
  DUNE::Concurrency::Mutex send_mutex, receive_mutex;
};
} // namespace TREX::LSTS
