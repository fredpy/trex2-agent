/*
 * TimelineReporter.cc
 *
 *  Created on: Apr 7, 2013
 *      Author: zp
 */

#include "TimelineReporter.hh"
# include "Platform.hh"
using namespace TREX::LSTS;
using namespace TREX::transaction;
using DUNE_NAMESPACES;

namespace
{

	/** @brief TimelineReporter reactor declaration */
	TeleoReactor::xml_factory::declare<TimelineReporter> decl("TimelineReporter");

}

TimelineReporter::TimelineReporter(TeleoReactor::xml_arg_type arg)
:TeleoReactor(arg), aborted(false)
{

}

TimelineReporter::~TimelineReporter() {
	// TODO Auto-generated destructor stub
}

void TimelineReporter::newPlanToken(goal_id const &g)
{
  Goal * goal = g.get();

  std::string gname = (goal->object()).str();
  std::string gpred = (goal->predicate()).str();

  std::cerr << "newPlanToken(" << gname << " , " << gpred << ")\n";
}

void TimelineReporter::notify(Observation const &obs)
{
  std::list<TREX::utils::Symbol> attrs;
  //TrexObservation msg;

  TrexOperation op;
  TrexToken token;

  token.predicate = obs.predicate().str();
  token.timeline = obs.object().str();
  std::cout << token.timeline << "." << token.predicate <<"{";

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

    std::cout << v;
  }
  std::cout << "}\n";
  std::cout.flush();

  op.token.set(token);

  Platform *r = m_env->getPlatformReactor();
  r->sendMsg(op);
}


