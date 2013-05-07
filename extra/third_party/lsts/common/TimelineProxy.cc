/*
 * TimelineProxy.cc
 *
 *  Created on: May 7, 2013
 *      Author: zp
 */

#include "TimelineProxy.hh"

using namespace TREX::LSTS;
using namespace TREX::transaction;
using namespace TREX::utils;
using DUNE_NAMESPACES;

namespace
{

  /** @brief TimelineReporter reactor declaration */
  TeleoReactor::xml_factory::declare<TimelineProxy> decl("TimelineProxy");

}

TimelineProxy::TimelineProxy(TeleoReactor::xml_arg_type arg)
:TeleoReactor(arg)
{
  m_destport = parse_attr<int>(6001, TeleoReactor::xml_factory::node(arg),
                               "destport");
  m_destaddr = parse_attr<std::string>("127.0.0.1",
                                       TeleoReactor::xml_factory::node(arg),
                                       "destaddr");
  m_localport = parse_attr<int>(6006, TeleoReactor::xml_factory::node(arg),
                               "localport");

  m_goalProxy = parse_attr<bool>(true, TeleoReactor::xml_factory::node(arg),
                                 "goal_forwarding");

  m_timeline = parse_attr<std::string>("", TeleoReactor::xml_factory::node(arg),
                                 "timeline");

}

TimelineProxy::~TimelineProxy()
{
  m_adapter.unbind();
}

void
TimelineProxy::handleInit()
{
  if (m_goalProxy)
  {
    provide(m_timeline, true, true);
  }
  else
  {
    use(m_timeline);
  }

  m_adapter.bind(m_localport);
}

void
TimelineProxy::handleRequest(goal_id const &g)
{
  if (!m_goalProxy)
  {
    syslog(log::warn) << "Cannot post goals to this observation-forwarding proxy";
    return;
  }
  if (g.get()->object().str() != m_timeline)
  {
    syslog(log::warn) << "Cannot handle requests on " << g.get()->object().str();
    return;
  }

  TrexOperation op;
  op.goal_id = g->object().str();

  TrexToken tok;
  m_adapter.asImcMessage(*g.get(), &tok);
  op.token.set(&tok);
  op.op = TrexOperation::OP_POST_GOAL;
  m_adapter.send(&op, m_destaddr, m_destport);
}

void
TimelineProxy::notify(Observation const &obs)
{
  if (m_goalProxy)
  {
    syslog(log::warn) << "Cannot post observations to this goal-forwarding proxy";
    return;
  }
  if (obs.object().str() != m_timeline)
  {
    syslog(log::warn) << "Cannot forward observations on " << obs.object().str();
    return;
  }

  TrexOperation op;
  TrexToken tok;

  m_adapter.asImcMessage(Observation(obs), &tok);
  op.token.set(&tok);
  op.op = TrexOperation::OP_POST_TOKEN;
  m_adapter.send(&op, m_destaddr, m_destport);
}

bool
TimelineProxy::synchronize()
{

  Message * msg;
  while ((msg = m_adapter.poll(0, false)) != NULL)
  {
    if (msg->getId() == TrexOperation::getIdStatic())
    {
      TrexOperation * top = dynamic_cast<TrexOperation *>(msg);
      switch(top->op)
      {
        case (TrexOperation::OP_POST_TOKEN):
            postObservation(m_adapter.genericObservation(top->token.get()), true);
        break;
        case (TrexOperation::OP_POST_GOAL):
            //TODO
            break;
        case (TrexOperation::OP_RECALL_GOAL):
            //TODO
            break;
      }
    }

    delete msg;
  }

  return true;
}
