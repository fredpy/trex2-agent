/*
 * TimelineReporter.cc
 *
 *  Created on: Apr 7, 2013
 *      Author: zp
 */

#include "TimelineReporter.hh"

using namespace TREX::LSTS;
using namespace TREX::transaction;
using namespace TREX::utils;
using DUNE_NAMESPACES;

namespace
{

  /** @brief TimelineReporter reactor declaration */
  TeleoReactor::xml_factory::declare<TimelineReporter> decl("TimelineReporter");

}

namespace TREX {
  void
  initPlugin()
  {
  }
}

TimelineReporter::TimelineReporter(TeleoReactor::xml_arg_type arg)
:TeleoReactor(arg), graph::timelines_listener(arg), aborted(false)
{
  m_hostport = parse_attr<int>(-1, TeleoReactor::xml_factory::node(arg),
                               "hostport");
  m_hostaddr = parse_attr<std::string>("127.0.0.1",
                                       TeleoReactor::xml_factory::node(arg),
                                       "hostaddr");
  m_output = parse_attr<bool>(false,
                              TeleoReactor::xml_factory::node(arg),
                              "output");
}

TimelineReporter::~TimelineReporter() {
	// TODO Auto-generated destructor stub
}

void
TimelineReporter::declared(details::timeline const &timeline)
{
  if (m_output)
    std::cout << "Timeline has been declared: " << timeline.name() << std::endl;

  //syslog(log::warn) << "Timeline has been declared: " << timeline.name();
  if( !isExternal(timeline.name()) )
	  use(timeline.name(), false, true);
}

void
TimelineReporter::undeclared(details::timeline const &timeline)
{
  if (m_output)
    std::cout << "Timeline has been undeclared: " << timeline.name() << std::endl;
  //syslog(log::warn) << "Timeline has been undeclared: " << timeline.name();
  unuse(timeline.name());
}

void TimelineReporter::newPlanToken(goal_id const &g)
{
  std::string gname = g->object().str();
  std::string gpred = g->predicate().str();

  if (m_output)
    std::cout << "GOAL: " << (*g) << std::endl;

  //syslog(log::warn) << "newPlanToken(" << gname << " , " << gpred << ")";
}

void TimelineReporter::cancelledPlanToken(goal_id const &g)
{
   std::string gname = g->object().str();
   std::string gpred = g->predicate().str();

   if (m_output)
     std::cout << "RECALL: " << (*g) << std::endl;

   //syslog(log::warn) << "cancelledPlanToken(" << gname << " , " << gpred << ")";
}

void TimelineReporter::notify(Observation const &obs)
{
  std::list<TREX::utils::Symbol> attrs;

  TrexOperation op;
  TrexToken token;

  token.predicate = obs.predicate().str();
  token.timeline = obs.object().str();

  op.op = TrexOperation::OP_POST_TOKEN;

  obs.listAttributes(attrs);
  std::list<TREX::utils::Symbol>::iterator it;
  for (it = attrs.begin(); it != attrs.end(); it++)
  {
    TrexAttribute attr;
    Variable v = obs.getAttribute(*it);
    Symbol type = v.domain().getTypeName();

    if (type.str() == "float") {
      attr.attr_type = TrexAttribute::TYPE_FLOAT;
    }
    else if (type.str() == "int") {
      attr.attr_type = TrexAttribute::TYPE_INT;
    }
    else if (type.str() == "bool") {
      attr.attr_type = TrexAttribute::TYPE_BOOL;
    }
    else if (type.str() == "string") {
      attr.attr_type = TrexAttribute::TYPE_STRING;
    }
    else if (type.str() == "enum") {
      attr.attr_type = TrexAttribute::TYPE_ENUM;
    }

    attr.name = (*it).str();

    if (v.domain().isSingleton()) {
      attr.max = v.domain().getStringSingleton();
      attr.min = v.domain().getStringSingleton();
    }
    //FIXME add support for interval domains
    //else if (v.domain().isInterval())
    //{

    //}
    else  {// if (v.domain().isFull()) {
      attr.max = "";
      attr.min = "";
    }

    token.attributes.push_back(attr);
  }
  if (m_output)
  {

    std::cout << "[" << getCurrentTick() << "] " << obs << std::endl;
  }
  op.token.set(token);
  if (m_hostport != -1)
    m_adapter.send(&op, m_hostaddr, m_hostport);
}


