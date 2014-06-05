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

#include <trex/domain/enum_domain.hh>
#include <trex/domain/string_domain.hh>

#include "AgentLocation.hh"

using namespace TREX::utils;
using namespace TREX::transaction;
using namespace TREX::AgentLocation;

symbol const AgentLocation::AtPred("At");

symbol const AgentLocation::GoPred("Go");

symbol const AgentLocation::AgentLocationObj("auvLocation");

AgentLocation::AgentLocation(reactor::xml_arg_type arg)
  :reactor(arg, false)
{
  syslog(null, info)<<"I want to own "<<AgentLocationObj;
  provide(AgentLocationObj);
  syslog(null, info)<<"I am done";
}

AgentLocation::~AgentLocation() {}

void AgentLocation::setAt(std::string location) {
  m_state.reset(new token(AgentLocationObj, AtPred));
  m_state->restrict_attribute("loc", enum_domain(location));
  post_observation(*m_state);
}

void AgentLocation::setGo(std::string origin, std::string destination){
    m_state.reset(new token(AgentLocationObj, GoPred));
    m_state->restrict_attribute("from", enum_domain(origin));
    m_state->restrict_attribute("to", enum_domain(destination));
    post_observation(*m_state);
}

void AgentLocation::handle_init(){
    m_state.reset(new token(AgentLocationObj, AtPred));
    m_state->restrict_attribute("loc", enum_domain("Ship"));
    post_observation(*m_state);
    m_nextSwitch = current_tick() + 1;
}

bool AgentLocation::synchronize() {
    TICK cur = current_tick();

    if( m_nextSwitch<=cur )
    {
      while(!m_pending.empty())
      {
        if( m_pending.front()->starts_after(cur) ) {
          if( m_pending.front()->starts_before(cur+1) ) {
            if(AtPred==m_pending.front()->predicate()) {
                setAt(m_pending.front()->attribute("loc").domain().get_singleton_as_string());
            } else {
                setGo(m_pending.front()->attribute("from").domain().get_singleton_as_string(),
                      m_pending.front()->attribute("to").domain().get_singleton_as_string());
            }
            m_nextSwitch = cur+m_pending.front()->duration().lower_bound().value();
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

void AgentLocation::handle_request(token_id const &g) {
  if( g->predicate()==AtPred || g->predicate()==GoPred ) {
    // I insert it on my list
    int_domain::bound lo = g->start().lower_bound();
    if( lo.is_infinity() ) {
      m_pending.push_front(g);
    } else {
      std::list<token_id>::iterator i = m_pending.begin();
      TICK val = lo.value();
      for(; m_pending.end()!=i; ++i )
      {
        if( (*i)->starts_after(val) )
            break;
      }
      m_pending.insert(i, g);
    }
  }
}

void AgentLocation::handle_recall(token_id const &g) {
    std::list<token_id>::iterator i = m_pending.begin();
    for( ; m_pending.end()!=i; ++i )
    {
        if( *i==g )
        {
            m_pending.erase(i);
            break;
        }
    }
}
