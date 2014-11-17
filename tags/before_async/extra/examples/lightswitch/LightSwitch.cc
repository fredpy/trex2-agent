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
#include <trex/utils/log_manager.hh>

#include <trex/domain/boolean_domain.hh>
#include <trex/domain/float_domain.hh>
#include <trex/domain/enum_domain.hh>

#include "LightSwitch.hh"

using namespace TREX::utils;
using namespace TREX::transaction;
using namespace TREX::lightswitch;

namespace {

  /** @brief TREX log entry point */
  singleton::use<log_manager> s_log;

  /** @brief Light reactor declaration */
  reactor::declare<Light> decl("Light");
  
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
    ::s_log->syslog("plugin.lightswitch", info)<<"LightSwitch loaded."<<std::endl;
    // ::decl;
  }

} // TREX

symbol const Light::onPred("On");

symbol const Light::offPred("Off");

symbol const Light::lightObj("light");

symbol const Light::upPred("Up");

symbol const Light::downPred("Down");

symbol const Light::brokenPred("Broken");

symbol const Light::switchObj("switch");


Light::Light(reactor::xml_arg_type arg)
  :reactor(arg, false),
   m_on(parse_attr<bool>(false, reactor::xml(arg),
			 "state")),
   m_verbose(parse_attr<bool>(false, reactor::xml(arg),
			      "verbose")),
   m_firstTick(true) {
  syslog(null, info)<<"I want to own "<<lightObj;
  provide(lightObj, false); // declare the light timeline ... no goal accepted here
  syslog(null, info)<<"I want to own "<<switchObj;
  provide(switchObj); // declare the switch timeline
  syslog(null, info)<<"I am done";
}

Light::~Light() {}

void Light::setValue(bool val) {
  symbol light_v, switch_v;
  m_on = val;

  if( m_on ) {
    light_v = onPred;
    switch_v = downPred;
  } else {
    light_v = offPred;
    switch_v = upPred;
  }
  m_light_state.reset(new token(lightObj, light_v));
  //syslog()<<"Posting "<<*m_light_state;
  post_observation(*m_light_state);
  token switch_state(switchObj, switch_v);
  switch_state.restrict_attribute("foo", boolean_domain(m_on));
  //syslog()<<"Posting "<<switch_state;
  post_observation(switch_state);
} 


bool Light::synchronize() {
  bool need_post = m_verbose;

  //syslog()<<"Synchronization :\n\t- first tick: "<<m_firstTick<<"\n\t- state: "<<m_on;
  if( m_firstTick ) {
    setValue(m_on);
    need_post = false;
    m_firstTick = false;
    m_nextSwitch = current_tick()+1;
  } else {
    TICK cur = current_tick();

    //syslog()<<"Testing next goal switch "<<m_nextSwitch<<"<="<<cur;
    if( m_nextSwitch<=cur ) {
      while( !m_pending.empty() ){
//        syslog()<<"Checking "<<m_pending.front()->object()<<"."
//        <<m_pending.front()->predicate()<<".start="<<m_pending.front()->getStart();
	if( m_pending.front()->starts_after(cur) ) {
	  // it can start after cur
	  if( m_pending.front()->starts_before(cur+1) ) {
	    
	    // it can also starts before cur => it can be set to cur 
	    if( brokenPred!=m_pending.front()->predicate() ) {
//              syslog()<<"Switching to "<<(downPred==m_pending.front()->predicate());
	      setValue(downPred==m_pending.front()->predicate());
	      need_post = false;
	    } else {
	      token obs(*m_pending.front());
//              syslog()<<"Posting "<<obs<<" as an observation";
	      post_observation(obs);
	    }
	    m_nextSwitch = cur+m_pending.front()->duration().lower_bound().value();
	    m_pending.pop_front();
	  } // else {
//            syslog()<<"This goal is still in the future ("<<m_pending.front()->getStart()
//            <<">"<<cur<<").";
//          }
	  // if we reached this point that mesans that the goal
	  // necessarily starts in the future (or has been
	  // processed)
	  break;
	} else {
//          syslog()<<"This goal shoudl have been already started ("
//          <<m_pending.front()->getStart()<<"<"<<cur<<").";
	  // too late to execute => remove it
	  m_pending.pop_front();
          
	}
      }
    }
  }
  // check if I still need to post the ligh state
  if( need_post ) 
    post_observation(*m_light_state);
  // always succeed
  return true;
}

void Light::handle_request(token_id const &g) {
  if( g->predicate()==upPred || g->predicate()==downPred || g->predicate()==brokenPred ) {
    // I insert it on my list
    int_domain::bound lo = g->start().lower_bound();
    if( lo.is_infinity() ) {
//      syslog()<<"Adding "<<*g<<" to front of pending goals";
      m_pending.push_front(g);
    } else {
      std::list<token_id>::iterator i = m_pending.begin();
//      size_t pos = 1;
      TICK val = lo.value();
      for(; m_pending.end()!=i; ++i ) {
	if( (*i)->starts_after(val) )
	  break;
//        ++pos;
      }
//      syslog()<<"Adding "<<*g<<" at position "<<pos<<" of pending goals";
      m_pending.insert(i, g);
    }
  }// else {
//    syslog()<<"Ignoring goal "<<*g;
//  }
}

void Light::handle_recall(token_id const &g) {
  std::list<token_id>::iterator i = m_pending.begin();
  for( ; m_pending.end()!=i; ++i ) {
    if( *i==g ) {
      m_pending.erase(i);
      break;
    }
  }
}      