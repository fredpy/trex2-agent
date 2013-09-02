/*
 * ImcMessenger.cc
 *
 *  Created on: Apr 4, 2013
 *      Author: zp
 */

#include "ImcMessenger.hh"

namespace TREX {
  namespace LSTS {

    void receiverThread(int port, std::queue<Message *> *inbox, Concurrency::Mutex * mutex)
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
          mutex->lock();
          inbox->push(msg);
          mutex->unlock();
        }
      }
    }

    void senderThread(std::queue<SendRequest> *outbox, Concurrency::Mutex * mutex)
    {
      UDPSocket sock;
      while (true)
      {
        boost::this_thread::sleep(boost::posix_time::milliseconds(50));

        mutex->lock();
        bool empty = outbox->empty();
        mutex->unlock();

        if (!empty)
        {
          mutex->lock();
          SendRequest req = outbox->front();
          outbox->pop();
          mutex->unlock();

          //std::cout << req.msg << std::endl;
          //std::cout << (req.msg)->getSerializationSize() << std::endl;
          DUNE::Utils::ByteBuffer bb;
          try
          {
            IMC::Packet::serialize(req.msg, bb);

            sock.write((const char*)bb.getBuffer(), (req.msg)->getSerializationSize(),
                       Address(req.addr.c_str()), req.port);

            delete req.msg;
          }
          catch (std::runtime_error& e)
          {
            std::cerr << "ERROR: " << ": " << e.what() << "\n";
          }
        }
      }
    }

    ImcMessenger::ImcMessenger()
    {
      receiver = NULL;
      sender = new boost::thread(senderThread, &outbox, &send_mutex);
    }

    void
    ImcMessenger::startListening(int bindPort)
    {
      stopListening();
      receiver = new boost::thread(receiverThread, bindPort, &inbox, &receive_mutex);
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
      bool empty = false;
      receive_mutex.lock();
      empty = inbox.empty();
      receive_mutex.unlock();
      return empty;
    }


    Message *
    ImcMessenger::receive()
    {
      Message * ret = NULL;
      receive_mutex.lock();
      if (!inbox.empty())
      {
        ret = inbox.front();
        inbox.pop();
      }
      receive_mutex.unlock();
      return ret;
    }

    void
    ImcMessenger::post(Message * msg, int port, std::string addr)
    {
      SendRequest req;
      req.msg = msg->clone();
      req.port = port;
      req.addr = addr;
      send_mutex.lock();
      outbox.push(req);
      send_mutex.unlock();
    }

    ImcMessenger::~ImcMessenger()
    {
      stopListening();
    }
  }
}

