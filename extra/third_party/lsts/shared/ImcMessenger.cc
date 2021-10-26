/*
 * ImcMessenger.cc
 *
 *  Created on: Apr 4, 2013
 *      Author: zp
 */

#include "trex/lsts/ImcMessenger.hh"
#include <array>

using TREX::LSTS::ImcMessenger;

ImcMessenger::ImcMessenger() : m_sending{true} {
  sender = std::make_unique<std::thread>([this]() {
    DUNE::Network::UDPSocket sock;

    while (m_sending.load()) {
      std::this_thread::sleep_for(std::chrono::milliseconds{50});
      {
        std::unique_lock l{send_mutex};
        if (!outbox.empty()) {
          SendRequest next = outbox.front();
          std::unique_ptr<message_type> del_msg{next.msg}; // ensure we delete this message no matter what
          outbox.pop();
          l.unlock();
          DUNE::Utils::ByteBuffer bb;
          try {
            DUNE::IMC::Packet::serialize(next.msg, bb);
            sock.write(bb.getBuffer(), next.msg->getSerializationSize(),
                       DUNE::Network::Address(next.addr.c_str()), next.port);
          } catch (std::runtime_error &e) {
            std::cerr << "ERROR: " << e.what() << "\n";
          }
        }
      }
    }
  });
}

ImcMessenger::~ImcMessenger() {
  m_sending.store(false);
  sender->join();
  stopListening();
}

void ImcMessenger::startListening(int bindPort) {
  stopListening();
  m_listening.store(true);
  receiver = std::make_unique<std::thread>([=] {
    DUNE::Network::UDPSocket sock;
    DUNE::IO::Poll poll;
    std::array<std::uint8_t, 65535> bfr;

    sock.bind(bindPort, DUNE::Network::Address::Any, true);
    poll.add(sock);

    while (m_listening.load()) {
      if (poll.poll(sock, 100)) {
        DUNE::Network::Address addr;
        std::uint16_t rv = sock.read(bfr.data(), bfr.size(), &addr);
        DUNE::IMC::Message *msg =
            DUNE::IMC::Packet::deserialize(bfr.data(), rv);
        {
          std::unique_lock l{receive_mutex};
          inbox.push(msg);
        }
      }
    }
  });
  std::cout << "listening for messages on " << bindPort << std::endl;
}

void ImcMessenger::stopListening() {
  if (auto tmp = std::move(receiver); tmp) {
    m_listening.store(false);
    tmp->join();
  }
}

bool ImcMessenger::inboxEmpty() {
  std::unique_lock l{receive_mutex};
  return inbox.empty();
}

ImcMessenger::message_type *ImcMessenger::receive() {
  std::unique_lock l{receive_mutex};
  if( inbox.empty() ) {
    return nullptr;
  }
  auto ret = inbox.front();
  inbox.pop();
  return ret;
}

void ImcMessenger::post(message_type *msg, int port, std::string addr) {
  SendRequest req{msg->clone(), port, std::move(addr)};
  {
    std::unique_lock l{send_mutex};
    outbox.push(req);
  }
}

