/*
 * ImcMessenger.cc
 *
 *  Created on: Apr 4, 2013
 *      Author: zp
 */

#include "ImcMessenger.hh"

namespace TREX {
  namespace LSTS {

    void receiverThread(int port, std::queue<Message *> *inbox)
    {
      UDPSocket sock;
      IOMultiplexing iom;
      uint8_t* bfr = new uint8_t[65535];

      sock.bind(port, Address::Any, true);
      sock.addToPoll(iom);

      while (true)
      {
        if (iom.poll(100))
        {
          Address addr;
          uint16_t rv = sock.read((char*)bfr, 65535, &addr);
          IMC::Message * msg = IMC::Packet::deserialize(bfr, rv);
          inbox->push(msg);
        }
      }
    }

    void senderThread(std::queue<SendRequest> *outbox)
    {
      UDPSocket sock;
      while (true)
      {
        if (outbox->empty())
        {
          boost::this_thread::sleep(boost::posix_time::milliseconds(50));
          continue;
        }
        SendRequest req = outbox->front();
        outbox->pop();

        std::cout << req.msg << std::endl;
        DUNE::Utils::ByteBuffer bb;
        try
        {
          IMC::Packet::serialize(req.msg, bb);

          sock.write((const char*)bb.getBuffer(), (req.msg)->getSerializationSize(),
                     Address(req.addr.c_str()), req.port);
        }
        catch (std::runtime_error& e)
        {
          std::cerr << "ERROR: " << ": " << e.what() << "\n";
        }
        delete req.msg;
      }
    }

    ImcMessenger::ImcMessenger()
    {
      receiver = NULL;
      sender = new boost::thread(senderThread, &outbox);
    }

    void
    ImcMessenger::startListening(int bindPort)
    {
      stopListening();
      receiver = new boost::thread(receiverThread, bindPort, &inbox);
      std::cout << "listening for messages on " << bindPort << std::endl;
    }

    void
    ImcMessenger::stopListening()
    {
      if (receiver != NULL)
        receiver->interrupt();
    }

    bool
    ImcMessenger::inboxEmpty()
    {
      return inbox.empty();
    }


    Message *
    ImcMessenger::receive()
    {
      Message * ret = NULL;
      if (!inbox.empty())
      {
        ret = inbox.front();
        inbox.pop();
      }
      return ret;
    }

    void
    ImcMessenger::post(Message * msg, int port, std::string addr)
    {
      SendRequest req;
      req.msg = msg;
      req.port = port;
      req.addr = addr;
      outbox.push(req);
    }

    ImcMessenger::~ImcMessenger()
    {
      stopListening();
    }
  }
}

