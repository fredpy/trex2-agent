/*
 * PositionUpdater.cpp
 *
 *  Created on: May 6, 2013
 *      Author: zp
 */

#include "PositionUpdater.hh"
# include "../shared/LstsUtils.hh"
using namespace TREX::transaction;
using namespace TREX::utils;
using namespace TREX::LSTS;
using DUNE_NAMESPACES;

namespace
{
  /** @brief PositionUpdater reactor declaration */
  reactor::declare<PositionUpdater> decl("PositionUpdater");
}


namespace TREX {
  void
  initPlugin()
  {

  }

  namespace LSTS {
    PositionUpdater::PositionUpdater(reactor::xml_arg_type arg) :
      LstsReactor(arg)
    {
      m_bind_port = parse_attr<int>(-1, xml(arg),
                                    "bindport");
    }

    void
    PositionUpdater::handle_init()
    {

      if (m_bind_port != -1)
        m_adapter.bind(m_bind_port);

      //m_adapter.startDiscovery();
    }

    void
    PositionUpdater::handle_tick_start()
    {
      Message * msg = NULL;
      while((msg = m_adapter.poll()) != NULL)
      {
        if (msg->getId() == Announce::getIdStatic())
        {
          Announce * ann = (Announce *) dynamic_cast<Announce *>(msg);

          if (m_receivedAnnounces[ann->sys_name] != NULL)
            delete m_receivedAnnounces[ann->sys_name];

          m_receivedAnnounces[ann->sys_name] = ann;
        }
        else
          delete msg;
      }
    }

    bool
    PositionUpdater::synchronize()
    {
      std::map<std::string, Announce *>::iterator it;

      for (it = m_receivedAnnounces.begin(); it != m_receivedAnnounces.end(); it++)
      {
        token obs = m_adapter.announceObservation(it->second);
        postUniqueObservation(obs);
      }
      return true;
    }

    PositionUpdater::~PositionUpdater()
    {
      m_adapter.unbind();
    }
  }
}
