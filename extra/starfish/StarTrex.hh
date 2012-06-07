/** @file "AuvSwitch.hh"
 *  @brief lightswitch plug-in utilities
 *
 *  This file defines the utilities provided by the lightswitch plug-in. 
 *  
 *  @ingroup lightswitch
 *  @author Frederic Py <fpy@mbari.org>
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
#ifndef H_StarTrex
# define H_StarTrex

# include <trex/transaction/TeleoReactor.hh>
#include <dsaav/dsaav.h>
#include <enum2str.h>

namespace TREX {
/** @brief lightswitch plug-in utilities
 *
 * This namespace embeds the classes and functions provided by the
 * lightswitch plug-in
 *
 * @ingroup lightswitch
 *
 * @author Frederic Py <fpy@mbari.org>
 */
namespace startrex {

/** @brief Auv reactor definition
 *
 * This class implements a very simple reactor that emulates a light
 * switch. It provides a @c light timeline that can be either on
 * (@c Auv.On) or off (@c Auv.Off)
 *
 * @author Frederic Py <fpy@mbari.org>
 * @ingroup lightswitch
 */
class Auv :public TREX::transaction::TeleoReactor {
public:
	/** @brief XML constructor
	 * @param arg An XML node definition
	 *
	 * This constructor is called to generate a Auv reactor
	 * based on an XML node. The expected XML format is the following:
	 * @code
	 * <Auv name="<name>" latency="<int>" lookahead="<int>" state="<bool>"
	 *        verbose="<bool>" />
	 * @endcode
	 * Where :
	 * @li @c state is a boolean value indicating which is the initial
	 *     state of the @c light timeline (default is @c true)
	 * @li @c verbose is a boolean used to indicate whther we should
	 *     repeat the light state at every tick or not (default is @c false)
	 */
	Auv(TREX::transaction::TeleoReactor::xml_arg_type arg);
	/** @brief Destructor */
	~Auv();


private:
	//STARFISH
	static const float PI         = 3.141593;
	static const float PIdiv2     = 1.570796;
	static const float WAYPTRADIUS = 10;

	IComms* ic;
	Rpc* rpc;
	Configuration* cfg;
	Sentuator *sPos, *sAlti, *sDepth, *sAtt;
	Sentuator *sEng, *sHelm, *sDiv;
	float roll,yaw,bearing,depth,xpos,ypos,ux,uy,uw,alt;
	float xSP,ySP,zSP,thrustSP,bearingSP;
	void bridgeSetup(Rpc* rpc);
	void getVehicleStates();
	void setThrust(float t);
	void setDepth(float d);
	void setBearing(float b);
	void setWP(float x, float y, float z);
	void VCS();
	bool navigate();
	void stop();
	float dist2D(float xStart, float yStart, float xEnd, float yEnd);
	float calDesiredBearing(float xStart, float yStart, float xEnd, float yEnd);


	//trex
	bool synchronize();
	void handleInit();
	void handleRequest(TREX::transaction::goal_id const &g);
	void handleRecall(TREX::transaction::goal_id const &g);

	/** @brief State of the timeline */
	bool m_on, m_verbose;
	TREX::transaction::TICK m_nextCmd;
	//used to keep track of current executing goal. Since goal will be
	//popped out of the list when they are being executed, a handleRecall
	//call back will not be able to stop current executing goal. By using
	//this variable, current executing goal can be aborted.
	TREX::transaction::goal_id currentGoal;
	/** @brief Is the state already posted as observation ? */
	bool m_firstTick;

	//void setValue(bool val);

	std::list<TREX::transaction::goal_id> m_pending;
	std::auto_ptr<TREX::transaction::Observation> m_light_state;

	//name for the vehicle timelines
	static TREX::utils::Symbol const vState; //observe
	static TREX::utils::Symbol const vWP_Bhv; //internal to vehicle (allow goals to be posted)

	static TREX::utils::Symbol const vehicleState;
	static TREX::utils::Symbol const wp_active;
	static TREX::utils::Symbol const wp_inactive;

	//enums
	static TREX::utils::Symbol const undefined;
	static TREX::utils::Symbol const success;
	static TREX::utils::Symbol const aborted;
	static TREX::utils::Symbol const preempted;

	TREX::utils::Symbol vCmdState;

};

}
}

#endif // H_StarTrex
