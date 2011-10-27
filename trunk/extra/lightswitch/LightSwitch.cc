/** @file "LightSwitch.cc"
 * @brief lightswitch plugin implementation
 *
 * @author Frederic Py <fpy@mbari.org>
 * @ingroup lightswitch
 */
/** @defgroup lightswitch The lightswitch plug-in
 * @brief A simple plug-in providing a light-switch reactor
 *
 * This group embeds all the utilities provided by the lightswitch
 * plug-in. This is a simple example of a TREX plug-in that provides the
 * Light reactor
 *
 * @author Frederic Py <fpy@mbari.org>
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

#include <trex/utils/Plugin.hh>
#include <trex/utils/LogManager.hh>

#include <trex/domain/FloatDomain.hh>
#include <trex/domain/EnumDomain.hh>

#include "LightSwitch.hh"

using namespace TREX::utils;
using namespace TREX::transaction;
using namespace TREX::lightswitch;

namespace {

  /** @brief TREX log entry point */
  SingletonUse<LogManager> s_log;

  /** @brief Light reactor declaration */
  TeleoReactor::xml_factory::declare<Light> decl("Light");
  
}

namespace TREX {
  
  /** @brief Plug-in initialisation
   *
   * This function is called by TREX after loading the lighswitch plug-in.
   * It manage the initialisation of this plug-in
   *
   * @ingroup lightswitch
   */
  void initPlugin() {
    ::s_log->syslog("plugin.lightswitch")<<"LightSwitch loaded."<<std::endl;
    // ::decl;
  }

} // TREX

Symbol const Light::onPred("On");

Symbol const Light::offPred("Off");

Symbol const Light::lightObj("light");

Symbol const Light::upPred("Up");

Symbol const Light::downPred("Down");

Symbol const Light::brokenPred("Broken");

Symbol const Light::switchObj("switch");


Light::Light(TeleoReactor::xml_arg_type arg)
  :TeleoReactor(arg, false), 
   m_on(parse_attr<bool>(false, TeleoReactor::xml_factory::node(arg),
			 "state")),
   m_firstTick(true) {
  syslog()<<"I want to own "<<lightObj;
  provide(lightObj); // declare the light timeline
  syslog()<<"I want to own "<<switchObj;
  provide(switchObj); // declare the switch timeline 
  syslog()<<"I am done";
}

Light::~Light() {}

void Light::setValue(bool val) {
  Symbol light_v, switch_v;
  m_on = val;

  if( m_on ) {
    light_v = onPred;
    switch_v = downPred;
  } else {
    light_v = offPred;
    switch_v = upPred;
  }
  Observation light_state(lightObj, light_v);
  light_state.restrictAttribute("foo", FloatDomain(2.4));

  postObservation(light_state);
  Observation switch_state(switchObj, switch_v);
  switch_state.restrictAttribute("amp", TREX::transaction::EnumDomain("foo"));
  postObservation(switch_state);
} 


bool Light::synchronize() {
  if( m_firstTick ) {
    setValue(m_on);
    m_firstTick = false;
    m_nextSwitch = getCurrentTick()+1;
  } else {
    TICK cur = getCurrentTick();

    if( m_nextSwitch<=cur ) {
      while( !m_pending.empty() )
	if( m_pending.front()->startsAfter(cur) ) {
	  // it can start after cur
	  if( m_pending.front()->startsBefore(cur) ) {
	    
	    // it can also starts before cur => it can be set to cur 
	    if( brokenPred!=m_pending.front()->predicate() )
	      setValue(downPred==m_pending.front()->predicate());
	    else {
	      Observation obs(*m_pending.front());
	      postObservation(obs);
	    }
	    m_nextSwitch = cur+m_pending.front()->getDuration().lowerBound().value();
	    m_pending.pop_front();
	  }
	  // if we reached this point that mesans that the goal
	  // necessarily starts in the future (or has been
	  // processed)
	  break;
	} else {
	  // too late to execute => remove it
	  m_pending.pop_front();
	}
    }
  }
  // always succeed
  return true;
}

void Light::handleRequest(goal_id const &g) {
  if( g->predicate()==upPred || g->predicate()==downPred || g->predicate()==brokenPred ) {
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

void Light::handleRecall(goal_id const &g) {
  std::list<goal_id>::iterator i = m_pending.begin();
  for( ; m_pending.end()!=i; ++i ) {
    if( *i==g ) {
      m_pending.erase(i);
      break;
    }
  }
}      
