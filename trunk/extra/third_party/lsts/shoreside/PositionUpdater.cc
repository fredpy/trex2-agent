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

namespace TREX {
  namespace LSTS {
    PositionUpdater::PositionUpdater(TeleoReactor::xml_arg_type arg) :
      TeleoReactor(arg, false)
    {
      m_bind_port = parse_attr<int>(6002, TeleoReactor::xml_factory::node(arg),
                                    "bindport");
      m_send_port = parse_attr<int>(6002, TeleoReactor::xml_factory::node(arg),
                                    "sendport");

      m_send_address = parse_attr<std::string>("255.255.255.255",
                                               TeleoReactor::xml_factory::node(arg),
                                               "sendip");
    }

    void
    PositionUpdater::handleInit()
    {
      m_adapter.bind(m_bind_port);
    }

    void
    PositionUpdater::handleTickStart()
    {
      Message * msg = NULL;
      while((msg = m_adapter.poll(0)) != NULL)
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
      //  Observation obs = m_adapter.announceObservation(it->second);

      }
      return true;
    }

    PositionUpdater::~PositionUpdater()
    {
      m_adapter.unbind();
    }


  }
}
