/*
 * SafetyBug.cc
 *
 *  Created on: Jun 21, 2012
 *      Author: zp
 */

#include "extra/lsts/SafetyBug.hh"
# include "Platform.hh"
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

void SafetyBug::notify(TREX::transaction::Observation const &obs)
{
	if (obs.predicate() == "Failed" && !aborted)
	{
		Abort ab;
		m_env->getPlatformReactor()->sendMsg(ab);
		m_env->getPlatformReactor()->reportErrorToDune("Sent abort due to " + obs.object().str() + " failure.");
		syslog("ERROR") << "Sent abort due to " << obs.object() << " failure.";
		aborted = true;
	}
}


