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
	reactor::declare<SafetyBug> decl("SafetyBug");

}

SafetyBug::SafetyBug(TREX::transaction::reactor::xml_arg_type arg)
:reactor(arg), aborted(false)
{

}

SafetyBug::~SafetyBug() {
	// TODO Auto-generated destructor stub
}

void SafetyBug::new_plan_token(TREX::transaction::token_id const &g)
{
  std::string gname = (g->object()).str();
  std::string gpred = (g->predicate()).str();

  syslog(utils::log::info) << "newPlanToken(" << gname << " , " << gpred << ")\n";
}

void SafetyBug::notify(TREX::transaction::token const &obs)
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


