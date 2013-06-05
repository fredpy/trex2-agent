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
#include "DrifterTracker.hh"

using namespace mbari;

/*
 * class mbari::MessageHandler
 */
MessageHandler::MessageHandler(MessageHandler::xml_arg const &arg, std::string const &pfx)
  :m_exchange(TREX::utils::parse_attr<std::string>(factory::node(arg), 
						   "exchange")),
   m_route(TREX::utils::parse_attr<std::string>("", factory::node(arg), "route")),
   m_tracker(*(arg.second)),
   m_prefix(TREX::utils::parse_attr<std::string>(pfx, factory::node(arg), "prefix"))
 {}

bool MessageHandler::provide(std::string const &timeline, bool control) {
  m_tracker.provide(timeline, control);
  if( m_tracker.isInternal(timeline) ) {
    if( control )
      m_tracker.goalHandler(timeline, this);
    return true;
  }
  return false;
}

void MessageHandler::notify(TREX::transaction::Observation const &obs) {
  m_tracker.postObservation(obs, true);
}

TREX::transaction::TICK MessageHandler::now() const {
  return m_tracker.getCurrentTick();
}

MessageHandler::date_type MessageHandler::tickToTime(TREX::transaction::TICK date) const {
  return m_tracker.tickToTime(date);
}

MessageHandler::duration_type MessageHandler::tickDuration() const {
  return m_tracker.tickDuration();
}

TREX::utils::log::stream
MessageHandler::syslog(TREX::utils::Symbol const &kind) {
   return m_tracker.syslog(route(), kind);
}
