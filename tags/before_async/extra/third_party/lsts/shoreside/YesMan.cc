/*
 * YesMan.cpp
 *
 *  Created on: May 9, 2013
 *      Author: Margarida
 */

#include "extra/third_party/lsts/shoreside/YesMan.hh"

using namespace TREX::LSTS;
using namespace TREX::transaction;
namespace
{
  /** @brief PositionUpdater reactor declaration */
  reactor::declare<YesMan> decl("YesMan");
}

#define SPOT_SIM_TL "spotSim" // internal
#define TREX_TL "drifterFollow" // external

YesMan::YesMan(reactor::xml_arg_type arg)
	:reactor(arg){
	  post_observation(token(SPOT_SIM_TL, "None"));
}

YesMan::~YesMan() { }

bool YesMan::synchronize() {
	return true;
}

/**
 * Getting the ___ from Rest
 */
void YesMan::handle_request(token_id const &g) {
	if( g->object()==SPOT_SIM_TL ) {
		TREX::transaction::token observation(*g);
		post_observation(observation);
	}
}

