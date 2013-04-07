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

  TrexObservation msg;
  msg.predicate = obs.predicate().str();
  msg.timeline = obs.object().str();
  obs.listAttributes(attrs);
  std::list<TREX::utils::Symbol>::iterator it;
  std::stringstream ss;
  for (it = attrs.begin(); it != attrs.end(); it++)
  {
    if (it != attrs.begin())
      ss <<";";
    ss << obs.getAttribute(*it);
  }

  msg.attributes=ss.str();

  Platform *r = m_env->getPlatformReactor();
  r->sendMsg(msg);
}


