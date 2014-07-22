/*
 * SafetyBug.cc
 *
 *  Created on: Jun 21, 2012
 *      Author: zp
 */

#include "extra/third_party/lsts/onboard/SafetyBug.hh"
# include "extra/third_party/lsts/onboard/Platform.hh"
using namespace TREX::LSTS;
using namespace TREX::transaction;
using DUNE_NAMESPACES;


namespace
{

	/** @brief SafetyBug reactor declaration */
	TeleoReactor::xml_factory::declare<SafetyBug> decl("SafetyBug");

}

SafetyBug::SafetyBug(TREX::transaction::TeleoReactor::xml_arg_type arg)
:TeleoReactor(arg), aborted(false)
{

}

SafetyBug::~SafetyBug() {
	// TODO Auto-generated destructor stub
}

void SafetyBug::newPlanToken(TREX::transaction::goal_id const &g)
{
  Goal * goal = g.get();

  std::string gname = (goal->object()).str();
  std::string gpred = (goal->predicate()).str();

  syslog(utils::log::info) << "newPlanToken(" << gname << " , " << gpred << ")\n";
}

void SafetyBug::notify(TREX::transaction::Observation const &obs)
{
  if (obs.predicate() == "Failed" && !aborted)
  {
    Platform *r = m_env->getPlatformReactor();
    if( NULL!=r ) {
      Abort ab;
      r->sendMsg(ab);
      r->reportErrorToDune("Sent abort due to " + obs.object().str() + " failure.");
    }
    syslog(utils::log::error)<< "Sent abort due to " << obs.object()
			         << " failure.";
    aborted = true;
  }
}


