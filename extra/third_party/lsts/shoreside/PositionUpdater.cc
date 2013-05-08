/*
 * PositionUpdater.cpp
 *
 *  Created on: May 6, 2013
 *      Author: zp
 */

#include "PositionUpdater.hh"
using namespace TREX::transaction;
using namespace TREX::utils;
using namespace TREX::LSTS;
using DUNE_NAMESPACES;

namespace
{
  /** @brief PositionUpdater reactor declaration */
  TeleoReactor::xml_factory::declare<PositionUpdater> decl("PositionUpdater");
}


namespace TREX {
  void
  initPlugin()
  {

  }

  namespace LSTS {
    PositionUpdater::PositionUpdater(TeleoReactor::xml_arg_type arg) :
      LstsReactor(arg)
    {
      m_bind_port = parse_attr<int>(-1, TeleoReactor::xml_factory::node(arg),
                                    "bindport");
    }

    void
    PositionUpdater::handleInit()
    {
      if (m_bind_port != -1)
        m_adapter.bind(m_bind_port);

      m_adapter.startDiscovery();
    }

    void
    PositionUpdater::handleTickStart()
    {
      Message * msg = NULL;
      while((msg = m_adapter.poll(0, true)) != NULL)
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
        Observation obs = m_adapter.announceObservation(it->second);
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
