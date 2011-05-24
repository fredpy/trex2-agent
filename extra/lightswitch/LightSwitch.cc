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
#include <iostream>

#include "Plugin.hh"
#include "LogManager.hh"
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

Symbol const Light::switchObj("switch");


Light::Light(TeleoReactor::xml_arg_type arg)
  :TeleoReactor(arg, false), 
   m_on(parse_attr<bool>(false, TeleoReactor::xml_factory::node(arg),
			 "state")),
   m_firstTick(true) {
  provide(lightObj); // declare the light timeline
  provide(switchObj); // declare the switch timeline 
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
  postObservation(Observation(lightObj, light_v));
  postObservation(Observation(switchObj, switch_v));
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
	    setValue(downPred==m_pending.front()->predicate());
	    m_nextSwitch = cur+m_pending.front()->getDuration().lowerBound().value();
	    m_pending.pop_front();
	  }
	  // if we reched this point that mesans that the goal
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
  if( g->predicate()==upPred || g->predicate()==downPred ) {
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
