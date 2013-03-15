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
#include <trex/europa/TrexThreatDecisionPoint.hh>
#include <trex/europa/Assembly.hh>

#pragma warning (push : 0)
// europa has a lot of warnings: lets make it more silent
# include <PLASMA/Token.hh>
# include <PLASMA/TokenVariable.hh>
#pragma warning (pop)



using namespace  TREX::europa;
using EUROPA::ObjectId;
using EUROPA::TokenId;

/*
 * class TREX::europa::TrexThreatDecisionPoint
 */ 

// structors

TrexThreatDecisionPoint::TrexThreatDecisionPoint
  (EUROPA::DbClientId const &client, EUROPA::TokenId const &tokenToOrder,
   EUROPA::TiXmlElement const &configData, EUROPA::LabelStr const &explanation)
  :EUROPA::SOLVERS::ThreatDecisionPoint(client, tokenToOrder, 
					configData, explanation) {
  if( !client->isGlobalVariable(Assembly::CLOCK_VAR) )
    throw EuropaException("Unable to find clock variable \""
			  +Assembly::CLOCK_VAR.toString()+"\"");
  m_clock = client->getGlobalVariable(Assembly::CLOCK_VAR);
  m_tokenToOrder->incRefCount();
}

TrexThreatDecisionPoint::~TrexThreatDecisionPoint() {
  m_tokenToOrder->decRefCount();
}

// manipulators

void TrexThreatDecisionPoint::handleInitialize() {
  ThreatDecisionPoint::handleInitialize();
  std::vector<std::pair<ObjectId, std::pair<TokenId, TokenId> > > past;

  if( m_tokenToOrder->start()->lastDomain().getUpperBound()>=now() ) { 
    EUROPA::eint min_dur = m_tokenToOrder->duration()->lastDomain().getLowerBound();
    // Prune all the solutions taht would put this token in the past
    std::vector<std::pair<ObjectId, std::pair<TokenId, TokenId> > >::iterator i = m_choices.begin();


    while( m_choices.end()!=i ) {
      if( m_tokenToOrder==i->second.first ) {
        if( m_tokenToOrder!=i->second.second ) {
          // m_tokenToOrder inserted before i->second.second
          EUROPA::eint max_start = i->second.second->start()->lastDomain().getUpperBound();
          if( max_start<=std::numeric_limits<EUROPA::eint>::infinity() ) {
            max_start = max_start-min_dur;
            if( max_start < now() ) {
              debugMsg("trex:threat", "remove (*"<<m_tokenToOrder->getKey()<<"*<"
                       <<i->second.second->getKey()<<")\n\tthe latter start="
                       <<i->second.second->start()->lastDomain().toString()
                       <<" is before "<<now());
              past.push_back(*i);
              i = m_choices.erase(i);
              continue;
            }
          }
        }
      } /* 
	   This code is no good ... need to tinker it but for now 
	   we disable it

	   else {
	   // m_tokenToOrder is inserted after i->second.first 
	   // check that i->second.first ends after now()
	   if( i->second.first->end()->lastDomain().getUpperBound()<now() ) {
	   debugMsg("trex:threat", "remove ("<<i->second.first->getKey()<<"<*"
	   <<m_tokenToOrder->getKey()<<"*)\n\tthe former end="
	   <<i->second.second->end()->lastDomain().toString()
	   <<" is before "<<now());
	   i = m_choices.erase(i);
	   continue;
	   }	
	   }
       */
      debugMsg("trex:threat", "keeping choice ("<<i->second.first->getKey()<<"<"
          <<i->second.second->getKey()
          <<")\n\t- first from : "<<i->second.first->start()->lastDomain().toString()
          <<" to "<<i->second.first->end()->lastDomain().toString()
          <<")\n\t- second from : "<<i->second.second->start()->lastDomain().toString()
          <<" to "<<i->second.second->end()->lastDomain().toString());
      ++i;
    }
  } else {
    debugMsg("trex:threat", m_tokenToOrder->getKey()<<" start="
             <<m_tokenToOrder->start()->lastDomain().toString()
             <<" is before "<<now()<<"\n\tdo keep its options open.");
    // Maybe we should exclude all choices that do not put start 
    // in the past ? ... it should have been already done by the 
    // other guy
  }
  // invert the order or merging decisions
  std::reverse(m_choices.begin(), m_choices.end());
  m_choices.insert(m_choices.end(), past.begin(), past.end());

  m_choiceCount = m_choices.size();
  debugMsg("trex:threat", "Number of choices for "<<m_tokenToOrder->getKey()<<": "<<m_choiceCount);
}

// observers

EUROPA::eint TrexThreatDecisionPoint::now() const {
  return m_clock->lastDomain().getLowerBound();
}

std::string TrexThreatDecisionPoint::toString() const {
  // check the border coniditions
  if( m_choices.empty() )
    return "THREAT:EMPTY "+m_tokenToOrder->toString();
  if( m_index>=m_choiceCount )
    return "THREAT:EXHAUSTED "+m_tokenToOrder->toString();
  // Now let ThreatDecisionPoint do its buggy work
  return EUROPA::SOLVERS::ThreatDecisionPoint::toString();
}

std::string TrexThreatDecisionPoint::toShortString() const {
  // check the border coniditions 
  if( m_choices.empty() )
    return "THR:EMPTY("+m_tokenToOrder->toString()+")";
  if( m_index>=m_choiceCount )
    return "THR:EXHAUSTED+("+m_tokenToOrder->toString()+")";
  // Now let ThreatDecisionPoint do its buggy work
  return EUROPA::SOLVERS::ThreatDecisionPoint::toShortString();
}
