/*
 * PositionUpdater.hpp
 *
 *  Created on: May 6, 2013
 *      Author: zp
 */

#ifndef POSITIONUPDATER_HPP_
#define POSITIONUPDATER_HPP_

# include <DUNE/DUNE.hpp>
# include "ImcAdapter.hh"
# include <trex/transaction/TeleoReactor.hh>
# include <trex/utils/Plugin.hh>
using namespace TREX::transaction;
using namespace TREX::utils;

using DUNE_NAMESPACES;

namespace TREX {
  namespace LSTS {
    class PositionUpdater: public TeleoReactor
    {
    public:
      PositionUpdater(TeleoReactor::xml_arg_type arg);
      virtual
      ~PositionUpdater();

    private:

      int m_bind_port, m_send_port;
      std::string m_send_address;
      ImcAdapter m_adapter;
      std::map<std::string, Announce *> m_receivedAnnounces;
      bool synchronize();
      void handleInit();
      void handleTickStart();
    };
  }
}

#endif /* POSITIONUPDATER_HPP_ */
