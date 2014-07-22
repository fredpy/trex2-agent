/*
 * PositionUpdater.hpp
 *
 *  Created on: May 6, 2013
 *      Author: zp
 */

#ifndef POSITIONUPDATER_HPP_
#define POSITIONUPDATER_HPP_

# include <DUNE/DUNE.hpp>
# include "../shared/ImcAdapter.hh"
# include <trex/transaction/reactor.hh>
# include <trex/utils/Plugin.hh>
# include "../shared/LstsReactor.hh"

using namespace TREX::transaction;
using namespace TREX::utils;

using DUNE_NAMESPACES;

namespace TREX {
  namespace LSTS {
    class PositionUpdater: public LstsReactor
    {
    public:
      PositionUpdater(reactor::xml_arg_type arg);
      virtual
      ~PositionUpdater();

    private:

      int m_bind_port;
      ImcAdapter m_adapter, m_discovery;
      std::map<std::string, Announce *> m_receivedAnnounces;
      bool synchronize();
      void handle_init();
      void handle_tick_start();
    };
  }
}

#endif /* POSITIONUPDATER_HPP_ */
