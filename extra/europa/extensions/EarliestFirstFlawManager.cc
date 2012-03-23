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

#include "EarliestFirstFlawManager.hh"

#include <PLASMA/Token.hh>
#include <PLASMA/TokenVariable.hh>

using namespace TREX::europa;

EarliestFirstFlawManager::EarliestFirstFlawManager(EUROPA::TiXmlElement const &cfg)
  :EUROPA::SOLVERS::OpenConditionManager(cfg) {}
      

bool EarliestFirstFlawManager::betterThan(EUROPA::EntityId const &a, 
					  EUROPA::EntityId const &b,
					  EUROPA::LabelStr &explanation) {
  if( b.isNoId() ) {
    explanation = "b.noId";
    return true;
  } 
  if( a.isNoId() ) {
    explanation = "a.noId";
    return false;
  }
  EUROPA::TokenId t_a(a), t_b(b);
  EUROPA::IntervalIntDomain 
    start_a = t_a->start()->lastDomain(), 
    start_b = t_b->start()->lastDomain(); 

  if( start_a.getLowerBound() < start_b.getLowerBound() ) {
    explanation = "a.lb<b";
    return true;
  } 
  if( start_b.getLowerBound() < start_a.getLowerBound() ) {
    explanation = "b.lb<a";
    return false;
  }
  if( t_a->isFact() ) {
    if( !t_b->isFact() ) {
      explanation = "a.isFact";
      return true;
    }
  } else if( t_b->isFact() ) {
    explanation = "b.isFact";
    return false;
  }
  
  bool ret = start_a.getUpperBound() < start_b.getUpperBound();
  
  if( ret ) {
    explanation = "a.ub<b";
  } else { 
    explanation = "b.ub<=a";
  }
  
  return ret;
}

