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
#include "WitreReactor.hh"

using namespace TREX::witre;
using namespace TREX::agent;
using namespace TREX::transaction;

/*
 * class TREX::witre::WitreReactor
 */

// structors 

//WitreReactor::WitreReactor(Server &server, Agent &a)
//:Agent::AgentProxy(a), graph::timelines_listener(a), m_server(server) {
//}
//
//WitreReactor::~WitreReactor() {
//}
//
//// TREX execution callbacks 
//
//void WitreReactor::handleInit() {
//  graph::timelines_listener::initialize();
//}
//
//void WitreReactor::handleTickStart() {
//  
//}
//
//void WitreReactor::notify(Observation const &obs) {
//  
//}
//
//bool WitreReactor::synchronize() {
//  return true;
//}
//
//void WitreReactor::newPlanToken(goal_id const &t) {
//  
//}
//
//void WitreReactor::cancelledPlanToken(goal_id const &t) {
//  
//}
//
//// TREX timelines callbacks
//
//void WitreReactor::declared(details::timeline const &tl) {
//  if( !isExternal(tl.name()) ) {
//    use(tl.name(), true, true);
//    m_timelines.insert(tl.name());
//    m_server.new_timeline(getAgentName(), tl.name());
//  }
//}
//
//void WitreReactor::undeclared(details::timeline const &tl) {
//  m_server.graph_updated(getAgentName());  
//}
//
//void WitreReactor::connected(Relation const &r) {
//  if( this!=&(r.client()) ) {
//    m_server.graph_updated(getAgentName());    
//  }
//}
//
//void WitreReactor::disconnected(Relation const &r) {
//  if( this!=&(r.client()) ) {
//    m_server.graph_updated(getAgentName());    
//  }  
//}
//
//
//
