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

#include <trex/domain/EnumDomain.hh>
#include <trex/domain/StringDomain.hh>

#include "AgentLocation.hh"

using namespace TREX::utils;
using namespace TREX::transaction;
using namespace TREX::AgentLocation;

Symbol const AgentLocation::AtPred("At");

Symbol const AgentLocation::GoPred("Go");

Symbol const AgentLocation::AgentLocationObj("location");

AgentLocation::AgentLocation(TeleoReactor::xml_arg_type arg)
  :TeleoReactor(arg, false)
{
  syslog(null, info)<<"I want to own "<<AgentLocationObj;
  provide(AgentLocationObj);
  syslog(null, info)<<"I am done";
}

AgentLocation::~AgentLocation() {}

void AgentLocation::setAt(std::string location) {
    m_state.reset(new Observation(AgentLocationObj, AtPred));
    m_state->restrictAttribute("loc", EnumDomain(location));
    postObservation(*m_state);
}

void AgentLocation::setGo(std::string origin, std::string destination){
    m_state.reset(new Observation(AgentLocationObj, GoPred));
    m_state->restrictAttribute("from", EnumDomain(origin));
    m_state->restrictAttribute("to", EnumDomain(destination));
    postObservation(*m_state);
}

void AgentLocation::handleInit(){
    m_state.reset(new Observation(AgentLocationObj, AtPred));
    m_state->restrictAttribute("loc", EnumDomain("Home"));
    postObservation(*m_state);
    m_nextSwitch = getCurrentTick() + 1;
}

bool AgentLocation::synchronize() {
    TICK cur = getCurrentTick();

    if( m_nextSwitch<=cur )
    {
      while(!m_pending.empty())
      {
        if( m_pending.front()->startsAfter(cur) ) {
          if( m_pending.front()->startsBefore(cur+1) ) {
            if(AtPred==m_pending.front()->predicate()) {
                setAt(m_pending.front()->getAttribute("loc").domain().getStringSingleton());
            } else {
                setGo(m_pending.front()->getAttribute("from").domain().getStringSingleton(),
                      m_pending.front()->getAttribute("to").domain().getStringSingleton());
            }
            m_nextSwitch = cur+m_pending.front()->getDuration().lowerBound().value();
            m_pending.pop_front();
          }
          // if we reached this point that means that the goal
          // necessarily starts in the future (or has been processed)
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

void AgentLocation::handleRequest(goal_id const &g) {
  if( g->predicate()==AtPred || g->predicate()==GoPred ) {
    // I insert it on my list
    IntegerDomain::bound lo = g->getStart().lowerBound();
    if( lo.isInfinity() ) {
      m_pending.push_front(g);
    } else {
      std::list<goal_id>::iterator i = m_pending.begin();
      TICK val = lo.value();
      for(; m_pending.end()!=i; ++i )
      {
        if( (*i)->startsAfter(val) )
            break;
      }
      m_pending.insert(i, g);
    }
  }
}

void AgentLocation::handleRecall(goal_id const &g) {
    std::list<goal_id>::iterator i = m_pending.begin();
    for( ; m_pending.end()!=i; ++i )
    {
        if( *i==g )
        {
            m_pending.erase(i);
            break;
        }
    }
}
