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
    ::s_log->syslog("lightswitch")<<"LightSwitch loaded."<<std::endl;
    // ::decl;
  }

} // TREX

Symbol const Light::onPred("On");

Symbol const Light::offPred("Off");

Symbol const Light::lightObj("light");

Light::Light(TeleoReactor::xml_arg_type arg)
  :TeleoReactor(arg, false), 
   m_on(parse_attr<bool>(false, TeleoReactor::xml_factory::node(arg),
			 "state")),
   m_posted(false) {
  provide(lightObj); // declare the timeline
}

Light::~Light() {}

bool Light::synchronize() {
  if( !m_posted ) {
    Symbol state;
    if( m_on ) 
      state = onPred;
    else
      state = offPred;
    Observation obs(lightObj, state);
    postObservation(obs);
    m_posted = true;
  } else {
    while( !m_pending.empty() ) {
      TICK cur = getCurrentTick();
      if( m_pending.front()->startsAfter(cur) ) {
	if( m_pending.front()->startsBefore(cur) ) {
	  Observation obs(*(m_pending.front()));
	  m_pending.pop_front();
	  postObservation(obs);
	}
	break;
      } else {
	m_pending.pop_front();
      }
    }
  }
  return true;
}

void Light::handleRequest(goal_id const &g) {
  if( g->predicate()==onPred || g->predicate()==offPred ) {
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
