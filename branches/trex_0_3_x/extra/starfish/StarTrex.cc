/** @file "StarTrex.cc"
 * @brief startrex reactor implementation
 *
 * @author WilliamTan
 * @ingroup startrex
 */
/** @defgroup startrex The StarTrex plug-in
 * @brief A bridge client between the STARFISH AUV and Trex reactor
 *
 * 
 *
 * @author WilliamTan
 * @ingroup plugins
 */
/*********************************************************************
 * Software License Agreement (BSD License)
 * 
 *  Copyright (c) 2011, MBARI.
 *  All rights reserved.
 * 
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 * 
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the TREX Project nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 * 
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */
#include <iostream>

#include <trex/domain/FloatDomain.hh>
#include <trex/domain/EnumDomain.hh>
#include <trex/domain/IntegerDomain.hh>


#include "StarTrex.hh"
#include "Constraints.hh"

using namespace TREX::utils;
using namespace TREX::transaction;
using namespace TREX::startrex;
using namespace starfish::europa;

namespace {

/** @brief Auv reactor declaration */
TeleoReactor::xml_factory::declare<Auv> decl("Auv");

}

Symbol const Auv::vState("vehicleState");
Symbol const Auv::vWP_Bhv("wp");

Symbol const Auv::vehicleState("Holds");
Symbol const Auv::wp_active("Active");
Symbol const Auv::wp_inactive("Inactive");

Symbol const Auv::undefined("UNDEFINED");
Symbol const Auv::success("SUCCESS");
Symbol const Auv::aborted("ABORTED");
Symbol const Auv::preempted("PREEMPTED");

Auv::Auv(TeleoReactor::xml_arg_type arg)
:TeleoReactor(arg, false),
 m_on(parse_attr<bool>(false, TeleoReactor::xml_factory::node(arg),
		 "state")),
		 m_verbose(parse_attr<bool>(false, TeleoReactor::xml_factory::node(arg),
				 "verbose")),
				 m_firstTick(true) {
	syslog()<<"I want to own "<<vState;
	provide(vState, false); // declare the light timeline ... no goal accepted here
	syslog()<<"I want to own "<<vWP_Bhv;
	provide(vWP_Bhv); // declare the switch timeline

	syslog()<<"All done";
}

Auv::~Auv() {}

void Auv::bridgeSetup(Rpc* rpc) {
	//printf("b called \n");
	//get configuration if there is any and provide rpc to the component
	if (rpc == NULL) {
		cfg = NULL;
		//printf("Rpc is null \n");
		return;
	}
	else
	{
		//free(cfg);
		cfg = new Configuration(rpc,"StarTrex");
	}

	//printf("here\n");
	//initialize all the sentuator's to send commands to the chucky
	sPos = cfg->getSentuator("Position");
	if (sPos == NULL) syslog()<<"Position sensor not found";
	sAlti = cfg->getSentuator("Altitude");
	if (sAlti == NULL) syslog()<<"Altitude sensor not found";
	sDepth = cfg->getSentuator("Depth");
	if (sDepth == NULL) syslog()<<"Depth sensor not found";
	sAtt = cfg->getSentuator("Attitude");
	if (sAtt == NULL) syslog()<<"Attitude sensor not found";
	sEng = cfg->getSentuator("EngineRoom");
	if (sEng == NULL) syslog()<<"EngineRoom Actuator not found";
	sHelm = cfg->getSentuator("Helmsman");
	if (sHelm == NULL) syslog()<<"Helmsman actuator not found";
	sDiv = cfg->getSentuator("DivingOfficer");
	if (sDiv == NULL) syslog()<<"DivingOfficer Actuator not found";
}

void Auv::getVehicleStates() {
	Measurement* m = NULL;
	//printf("getState \n");
	if (sPos != NULL) {
		m = sPos->get(MT_POSITION,0);
		xpos = m->get(MQ_XPOS);
		ypos = m->get(MQ_YPOS);
		syslog()<<"xpos is "<<xpos<<" ypos is "<<ypos;
	}

	if (sAlti != NULL) {
		m = sAlti->get(MT_ALTITUDE,0);
		alt = m->get(MQ_ALTITUDE);
		syslog()<<"alti is "<<alt;
	}

	if (sDepth != NULL) {
		m = sDepth->get(MT_DEPTH,0);
		depth = m->get(MQ_DEPTH);
		syslog()<<"depth is "<<depth;
	}

	if (sAtt != NULL) {
		m = sAtt->get(MT_ATTITUDE,0);
		roll = m->get(MQ_ROLL);
		yaw = m->get(MQ_YAW);
		bearing = m->get(MQ_BEARING,0);
		syslog()<<"roll is "<<roll/PI*180<<" yaw is "<<yaw/PI*180<<" bearing="<<bearing;
	}

}

void Auv::handleInit() {

	vCmdState = wp_inactive;
	roll = yaw = bearing = depth = xpos = ypos = ux = uy = uw = alt = xSP = ySP = zSP = thrustSP = bearingSP = 0.0;
	syslog()<<"Trying establish connection with STARFISH";

	//////////////////////////////////////////////
	//handle starfish stuffs
	ic = new ICommsSim(0xB3);
	MsgSvcAddr* addr = ic->getAddress();
	if (addr == NULL) {
		printf("Could not open IComms interface\n");
		exit;
	}
	String* s = addr->toString();
	printf("Running safety officer on %s \n",s->chars());
	delete s;
	delete addr;
	rpc = new Rpc(ic);
	bridgeSetup(rpc);

	syslog()<<"Init completed ! ";
}


/*void Auv::setValue(bool val) {
	Symbol light_v, switch_v;
	m_on = val;

	if( m_on ) {
		light_v = onPred;
		switch_v = downPred;
	} else {
		light_v = offPred;
		switch_v = upPred;
	}
	m_light_state.reset(new Observation(lightObj, light_v));
	postObservation(*m_light_state);
	Observation switch_state(switchObj, switch_v);
	postObservation(switch_state);
}*/


bool Auv::synchronize() {
	//get starfish states
	if (sPos == NULL)
		bridgeSetup(rpc);
	getVehicleStates();

	//post position observation
	Observation posObs(vState,vehicleState);
	posObs.restrictAttribute("x",FloatDomain(xpos));
	posObs.restrictAttribute("y",FloatDomain(ypos));
	posObs.restrictAttribute("depth",FloatDomain(depth));
	postObservation(posObs);

	bool need_post = m_verbose;

	if( m_firstTick ) {
		//post vehicle state
		Observation cmd_state(vWP_Bhv,vCmdState);
		cmd_state.restrictAttribute("status",EnumDomain(undefined));
		postObservation(cmd_state);

		need_post = false;
		m_firstTick = false;
		m_nextCmd = getCurrentTick()+1;
	} else {
		TICK cur = getCurrentTick();

		if( m_nextCmd<=cur ) {
			if( !m_pending.empty() && vCmdState == wp_inactive)
				if( m_pending.front()->startsAfter(cur) ) {
					// it can start after cur
					if( m_pending.front()->startsBefore(cur) ) {
						//record current executing goal
						currentGoal = m_pending.front();
						syslog()<<"goal id bef="<<currentGoal; //debug
						// it can also starts before cur => it can be set to cur
						vCmdState = m_pending.front()->predicate();
						Variable const* var = &(*m_pending.front())["x"];
						float x = var->domain().getTypedSingleton<float,true>(); //check for existence and singleton
						var = &(*m_pending.front())["y"];
						float y = var->domain().getTypedSingleton<float,true>();
						var = &(*m_pending.front())["depth"];
						float z = var->domain().getTypedSingleton<float,true>();
						setWP(x,y,z);
						var = &(*m_pending.front())["thrust"];
						int thrust = var->domain().getTypedSingleton<int,true>();
						thrustSP = thrust;
						//calculate the timing for the next command.
						m_nextCmd = cur+m_pending.front()->getDuration().lowerBound().value();
						m_pending.pop_front();
						syslog()<<"goal id aft="<<currentGoal; //debug
						//post state
						vCmdState = wp_active;
						// only post when there is a change of cmd state. !!!
						//post vehicle state
						Observation cmd_state(vWP_Bhv,vCmdState);
						postObservation(cmd_state);
					}
				} else {
					// too late to execute => remove it
					m_pending.pop_front();
				}
		}
	}

	//perform vehicle control
	VCS();

	// always succeed
	return true;
}

void Auv::handleRequest(goal_id const &g) {
	if( g->predicate()==wp_active || g->predicate()==wp_inactive ) {

		// I insert it on my list
		IntegerDomain::bound lo = g->getStart().lowerBound();
		if( lo.isInfinity() ) {
			m_pending.push_front(g);
		} else {
			std::list<goal_id>::iterator i = m_pending.begin();
			TICK val = lo.value();
			for(; m_pending.end()!=i; ++i ) {
				if( (*i)->startsAfter(val) )
					break;
			}
			m_pending.insert(i, g);
		}
	}
}

void Auv::handleRecall(goal_id const &g) {
	syslog()<<"goal id handleRecall="<<g<<" cur goalid="<<currentGoal; //debug
	//if the recall is for the current executing goal.
	if( currentGoal == g) {
		vCmdState = wp_inactive;
		// only post when there is a change of cmd state. !!!
		//post vehicle state
		Observation cmd_state(vWP_Bhv,vCmdState);
		cmd_state.restrictAttribute("status",EnumDomain(preempted));
		postObservation(cmd_state);
		stop();
		return;
	}
	else
	{
		std::list<goal_id>::iterator i = m_pending.begin();
		for( ; m_pending.end()!=i; ++i ) {
			if( *i==g ) {
				m_pending.erase(i);
				break;
			}
		}
	}
}

void Auv::VCS() {

	if(vCmdState==wp_active) {
		if(navigate()) {
			vCmdState = wp_inactive;
			// only post when there is a change of cmd state. !!!
			//post vehicle state
			Observation cmd_state(vWP_Bhv,vCmdState);
			cmd_state.restrictAttribute("status",EnumDomain(success));
			postObservation(cmd_state);
		}
	}
	else {
		stop();
	}
}

bool Auv::navigate() {
	//log current position and the wp
	syslog()<<"curPos x="<<xpos<<" y="<<ypos<<" z="<<depth;
	syslog()<<"WP x="<<xSP<<" y="<<ySP<<" z="<<zSP;

	//check distance
	float dist = dist2D(xpos,ypos,xSP,ySP);
	if (dist <= WAYPTRADIUS)
		return true;

	bearingSP = calDesiredBearing(xpos,ypos,xSP,ySP);

	//issue navigation command
	setThrust(thrustSP);
	setDepth(zSP);
	setBearing(bearingSP);

	return false;
}

void Auv::stop() {
	syslog()<<"Vehicle Stop signal received, stopping...";
	setThrust(0);
	setDepth(VAL_DISABLE);
	setBearing(VAL_DISABLE);
}

void Auv::setWP(float x, float y, float z) {
	xSP = x;
	ySP = y;
	zSP = z;
}

void Auv::setThrust(float t) {
	if(sEng != NULL)
		sEng->set(AT_THRUST,t);
	else
		syslog()<<"sEng is NULL";
}

void Auv::setDepth(float d) {
	if(sDiv != NULL)
		sDiv->set(AT_DEPTH,d);
	else
		syslog()<<"sDiv is NULL";
}

void Auv::setBearing(float b) {
	if(sHelm != NULL)
		sHelm->set(AT_BEARING,b);
	else
		syslog()<<"sHelm is NULL";
}


float Auv::dist2D(float xStart, float yStart, float xEnd, float yEnd) {
	return sqrt(pow((xStart-xEnd),2)+pow((yStart-yEnd),2));
}

float Auv::calDesiredBearing(float xStart, float yStart, float xEnd, float yEnd) {
	float dx = xEnd-xStart;
	float dy = yEnd-yStart;
	float angle = 0;

	if(dy==0.0&&dx==0.0)
		angle = 0.0;
	else
		angle = -(atan2(dy,dx)-PIdiv2);

	if(angle<0) angle+=2*PI;
	return angle*180/PI;
}

