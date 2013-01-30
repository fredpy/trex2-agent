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
#include "trex/europa/TimeConstraints.hh"
#include "trex/europa/Assembly.hh"

#include <PLASMA/Domain.hh>



using namespace TREX::europa;


/*
 * class TREX::europa::TickFromDate
 */
 
// structors 

TickFromDate::TickFromDate(EUROPA::LabelStr  const &name, 
                           EUROPA::LabelStr const &propagatorName,
                           EUROPA::ConstraintEngineId const &cstrEngine, 
                           std::vector<EUROPA::ConstrainedVariableId> const &variables)
  :ReactorConstraint(name, propagatorName, cstrEngine, variables) {
  if( m_variables.size()!=TickFromDate::NARGS )
    throw EuropaException("Constraint "+name.toString()+" accept exactly 2 arguments");
  m_tick = &getCurrentDomain(m_variables[TickFromDate::TICK]);
  m_date = &getCurrentDomain(m_variables[TickFromDate::DATE]);
}

// manipulators
 
void TickFromDate::handleExecute() {
  Assembly &a = assembly();
  
  EUROPA::edouble t_lo, t_hi, d_lo, d_hi, tmp;
  bool up_lo = true, up_hi = true;
  
  m_date->getBounds(d_lo, d_hi);
  m_tick->getBounds(t_lo, t_hi);
  
  if( d_hi < std::numeric_limits<EUROPA::edouble>::infinity() ) {
    tmp = a.date_to_tick(d_hi);
    if( tmp < t_hi ) {
      t_hi = tmp;
      up_hi = false;
    }
  }
  if( std::numeric_limits<EUROPA::edouble>::minus_infinity() < d_lo ) {
    tmp = a.date_to_tick(d_lo);
    if( t_lo < tmp ) {
      t_lo = tmp;
      up_lo = false;
    }
  }
  
  if( m_tick->intersect(t_lo, t_hi)
     && m_tick->isEmpty() )
    return;
  if( up_hi && t_hi < std::numeric_limits<EUROPA::eint>::infinity() )
    d_hi = a.tick_to_date(t_hi);
  
  if( up_lo && std::numeric_limits<EUROPA::eint>::minus_infinity() < t_lo )
    d_lo = a.tick_to_date(t_lo);
  m_date->intersect(d_lo, d_hi);
}
