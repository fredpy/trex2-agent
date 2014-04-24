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
  reactor::factory::declare<TimelineProxy> decl("TimelineProxy");

}

TimelineProxy::TimelineProxy(reactor::xml_arg_type arg)
:reactor(arg)
{
  m_destport = parse_attr<int>(6001, reactor::factory::node(arg),
                               "destport");
  m_destaddr = parse_attr<std::string>("127.0.0.1",
                                       reactor::factory::node(arg),
                                       "destaddr");
  m_localport = parse_attr<int>(6006, reactor::factory::node(arg),
                                "localport");

  m_goalProxy = parse_attr<bool>(true, reactor::factory::node(arg),
                                 "goal_forwarding");

  m_timeline = parse_attr<std::string>("", reactor::factory::node(arg),
                                       "timeline");

  m_useIridium = parse_attr<bool>(false, reactor::factory::node(arg),
                                        "iridium");
}

TimelineProxy::~TimelineProxy()
{
  m_adapter.unbind();
}

void
TimelineProxy::handle_init()
{
  if (m_goalProxy)
  {
    provide(m_timeline, true, true);
  }
  else
  {
    use(m_timeline);
  }

  m_adapter.setReactorGraph(this->get_graph());
  m_adapter.bind(m_localport);

  if (m_goalProxy)
  {
    syslog(log::warn) << "listening for " << m_timeline << " goals at port " << m_localport;
    std::cout << "listening for " << m_timeline << " goals at port " << m_localport << std::endl;
  }
  else
  {
    syslog(log::warn) << "listening for " << m_timeline << " observations at port " << m_localport;
    std::cout << "listening for " << m_timeline << " observations at port " << m_localport << std::endl;
  }
}

void
TimelineProxy::handle_request(goal_id const &g)
{

  syslog(log::info) << "handleRequest(" << g <<")";

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
  std::ostringstream ss;
  ss << g;
  op.goal_id = ss.str();

  TrexToken tok;
  m_adapter.asImcMessage(*g.get(), &tok);
  op.token.set(&tok);
  op.op = TrexOperation::OP_POST_GOAL;
  m_adapter.send(&op, m_destaddr, m_destport);
  std::cerr << "just sent this: " << std::endl;
  op.toText(std::cerr);
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

  if (m_useIridium)
  {
    m_adapter.sendViaIridium(&op, m_destaddr, m_destport);
    std::cerr << "just sent this via iridium: " << std::endl;
  }
  else
  {
    m_adapter.send(&op, m_destaddr, m_destport);
    std::cerr << "just sent this via IMC: " << std::endl;
  }

  op.toText(std::cerr);
}

bool
TimelineProxy::synchronize()
{

  //std::cout << "I'm here" << std::endl;

  Message * msg;
  while ((msg = m_adapter.poll()) != NULL)
  {
    if (msg->getId() == TrexOperation::getIdStatic())
    {
      TrexOperation * top = dynamic_cast<TrexOperation *>(msg);

     // std::cerr << "just received this: " << std::endl;
     // top->toText(std::cerr);
      switch(top->op)
      {
        case (TrexOperation::OP_POST_TOKEN):
              post_observation(m_adapter.genericObservation(top->token.get()), true);
        break;
        case (TrexOperation::OP_POST_GOAL):
            {
          goal_id gid = post_goal(m_adapter.genericGoal(top->token.get()));
          m_goals[top->goal_id] = gid;
            }
        break;
        case (TrexOperation::OP_RECALL_GOAL):
              goal_map::iterator found = m_goals.find(top->goal_id);
        if (found != m_goals.end())
        {
          post_recall(found->second);
          m_goals.erase(found);
        }
        break;
      }
    }

    delete msg;
  }

  return true;
}
