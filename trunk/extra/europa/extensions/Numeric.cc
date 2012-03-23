/*********************************************************************
 * Software License Agreement (BSD License)
 * 
 *  Copyright (c) 2012, MBARI.
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
#include "Numeric.hh"

#include <PLASMA/ConstrainedVariable.hh>


using namespace TREX::europa;

AbsValConstraint::AbsValConstraint(EUROPA::LabelStr const &name,
				   EUROPA::LabelStr const &propagatorName,
				   EUROPA::ConstraintEngineId const &cstrEngine,
				   std::vector<EUROPA::ConstrainedVariableId> const &vars)
:EUROPA::Constraint(name, propagatorName, cstrEngine, vars),
 m_abs(getCurrentDomain(vars[1])), 
 m_val(getCurrentDomain(vars[0])) {
  checkError(vars.size()==2, "Exactly 2 parameters required.");
}


void AbsValConstraint::handleExecute() {
  EUROPA::edouble a_lb, a_ub, v_lb, v_ub;
  // restrict m_abs based on m_val
  m_val.getBounds(v_lb, v_ub);
  
  if( v_lb>=0.0 ) {
    a_lb = v_lb;
    a_ub = v_ub;
  } else {
    if( v_ub>=0.0 )
      a_lb = 0.0;
    else 
      a_lb = std::min(std::abs(v_lb), std::abs(v_ub));
    a_ub = std::max(std::abs(v_lb), std::abs(v_ub));
  }
  m_abs.intersect(a_lb, a_ub);

  // constrain m_val based on m_abs
  m_abs.getBounds(a_lb, a_ub);

  if( a_lb==0.0 && m_val.isMember(-a_ub) && m_val.isMember(a_ub) )
    m_val.intersect(-a_ub, a_ub);
  if( ( m_val.isMember(a_lb) || m_val.isMember(a_ub) ) &&
      ( m_val.isMember(-a_lb) || m_val.isMember(-a_ub) ) )
    return;
  if( m_val.isMember(a_lb) || m_val.isMember(a_ub) )
    m_val.intersect(a_lb, a_ub);
  if( m_val.isMember(-a_lb) || m_val.isMember(-a_ub) )
    m_val.intersect(-a_ub, -a_lb);     
}
