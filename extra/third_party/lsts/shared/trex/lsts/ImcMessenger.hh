/*
 * ImcMessenger.hh
 *
 *  Created on: 30 September 2013
 *      Author: zp
 */

#ifndef IMCMESSENGER_HH_
#define IMCMESSENGER_HH_

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

# include <boost/thread/thread.hpp>

# include "EuropaExtensions.hh"

using DUNE_NAMESPACES;
using namespace TREX::transaction;
using namespace TREX::utils;

typedef struct {
  Message * msg;
  int port;
  std::string addr;
} SendRequest;

namespace TREX {
  namespace LSTS {

    class ImcMessenger
    {
    public:
      ImcMessenger();

      void startListening(int bindPort);
      void stopListening();
      bool inboxEmpty();
      Message * receive();
      void post(Message * msg, int port, std::string addr);

      virtual
      ~ImcMessenger();

    private:
      boost::thread * receiver, * sender;
      std::queue<Message *> inbox;
      std::queue<SendRequest> outbox;
      Concurrency::Mutex send_mutex, receive_mutex;
    };
  }
}


#endif /* IMCMESSENGER_HH_ */
