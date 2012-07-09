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
#include <PLASMA/Domains.hh>


using namespace TREX::europa;

/*
 * class TREX::europa::AbsValConstraint
 */

AbsValConstraint::AbsValConstraint(EUROPA::LabelStr const &name,
				   EUROPA::LabelStr const &propagatorName,
				   EUROPA::ConstraintEngineId const &cstrEngine,
				   std::vector<EUROPA::ConstrainedVariableId> const &vars)
:EUROPA::Constraint(name, propagatorName, cstrEngine, vars),
 m_abs(getCurrentDomain(vars[0])), 
 m_val(getCurrentDomain(vars[1])) {
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

/*
 * class TREX::europa::SqrtConstraint
 */

SqrtConstraint::SqrtConstraint(EUROPA::LabelStr const &name,
                               EUROPA::LabelStr const &propagatorName,
                               EUROPA::ConstraintEngineId const &cstrEngine,
                               std::vector<EUROPA::ConstrainedVariableId> const &vars)
:EUROPA::Constraint(name, propagatorName, cstrEngine, vars),
m_sqrt(getCurrentDomain(vars[0])), 
m_val(getCurrentDomain(vars[1])) {
  checkError(vars.size()==2, "Exactly 2 parameters required.");
}


void SqrtConstraint::handleExecute() {
  static EUROPA::IntervalDomain const positive(0.0, std::numeric_limits<EUROPA::edouble>::infinity());
  static EUROPA::edouble const acceptable_epsilon(1e-4);

  if( m_sqrt.isOpen() && m_val.isOpen() ) {
    m_sqrt.intersect(positive);
    m_val.intersect(positive);
    return;
  } else if( m_val.isSingleton() ) {
    EUROPA::edouble val = m_val.getSingletonValue(), root;
    
    if( val<0.0 ) {
      m_val.intersect(positive);
      return;
    }
    root = std::sqrt(val);
    if( val>=std::numeric_limits<EUROPA::edouble>::infinity() )
      m_sqrt.intersect(root, std::numeric_limits<EUROPA::edouble>::infinity());
    else 
      m_sqrt.intersect(root, root);
    return;
  } else if( m_sqrt.isSingleton() ) {
    EUROPA::edouble root = m_sqrt.getSingletonValue(), val;
    
    if( root<0.0 ) {
      m_sqrt.intersect(positive);
      return;
    }
    val = root*root;
    if( val>=std::numeric_limits<EUROPA::edouble>::infinity() )
      m_val.intersect(root, std::numeric_limits<EUROPA::edouble>::infinity());
    else 
      m_val.intersect(val, val);
    return;
  }
  EUROPA::edouble v_min, v_max, r_min, r_max;
  
  for(bool done=false; !done; ) {
    done = true;
    m_val.getBounds(v_min, v_max);
    m_sqrt.getBounds(r_min, r_max);
    
    if( v_min<=0.0 )
      v_min = 0.0;
    if( r_min<=0.0 )
      r_min = 0.0;
    
    // compute upper bound of sqrt
    EUROPA::edouble max_r, min_r;
    if( v_max>=std::numeric_limits<EUROPA::edouble>::infinity() )
      max_r = v_max;
    else 
      max_r = std::sqrt(v_max);
    if( r_max-acceptable_epsilon > max_r )
      r_max = max_r;
    
    // compute lower bound of sqrt
    min_r = std::sqrt(v_min);
    if( r_min+acceptable_epsilon < min_r )
      r_min = min_r;
    if( m_sqrt.intersect(r_min, r_max) && m_sqrt.isEmpty() )
      return;
    
    EUROPA::edouble max_v, min_v;
    
    // compute upper bound of the square
    max_v = r_max*r_max;
    if( v_max-acceptable_epsilon > max_v )
      v_max = max_v;
    // compute the lower bound of the square
    min_v = r_min*r_min;
    if( v_min+acceptable_epsilon < min_v )
      v_min = min_v;
    
    if( m_val.intersect(v_min, v_max) ) {
      if( m_val.isEmpty() )
        return;
      // need to propagate the new domain to m_sqrt
      done = false;
    }
  }
}

/*
 * class TREX::europa::CeillConstraint
 */

CeilConstraint::CeilConstraint(EUROPA::LabelStr const &name,
                                 EUROPA::LabelStr const &propagatorName,
                                 EUROPA::ConstraintEngineId const &cstrEngine,
                                 std::vector<EUROPA::ConstrainedVariableId> const &vars)
:EUROPA::Constraint(name, propagatorName, cstrEngine, vars),
  m_ceil(getCurrentDomain(vars[0])), 
  m_val(getCurrentDomain(vars[1])) {
  checkError(vars.size()==2, "Exactly 2 parameters required.");
}


void CeilConstraint::handleExecute() {
  EUROPA::edouble c_lb, c_ub, v_lb, v_ub;
  // restrict m_abs based on m_val
  m_val.getBounds(v_lb, v_ub);
  
  if( v_lb<=std::numeric_limits<EUROPA::edouble>::minus_infinity() )
    c_lb = std::numeric_limits<EUROPA::eint>::minus_infinity();
  else
    c_lb = std::ceil(v_lb); 
  if( v_ub>=std::numeric_limits<EUROPA::edouble>::infinity() )
    c_ub = std::numeric_limits<EUROPA::eint>::infinity();
  else
    c_ub = std::ceil(v_ub);
  
  m_ceil.intersect(c_lb, c_ub);
  
  // constrain m_val based on m_abs
  m_ceil.getBounds(c_lb, c_ub);
  
  if( c_lb<=std::numeric_limits<EUROPA::eint>::minus_infinity() )
    v_lb = std::numeric_limits<EUROPA::edouble>::minus_infinity();
  else if( (c_lb-1.0)<v_lb )
    v_lb = c_lb-1.0;
  if( c_ub>=std::numeric_limits<EUROPA::eint>::infinity() )
    v_ub = std::numeric_limits<EUROPA::edouble>::infinity();
  else if( v_ub>c_ub )
    v_ub = c_ub;
  m_val.intersect(v_lb, v_ub);
}

/*
 * class TREX::europa::FloorConstraint
 */

FloorConstraint::FloorConstraint(EUROPA::LabelStr const &name,
                               EUROPA::LabelStr const &propagatorName,
                               EUROPA::ConstraintEngineId const &cstrEngine,
                               std::vector<EUROPA::ConstrainedVariableId> const &vars)
:EUROPA::Constraint(name, propagatorName, cstrEngine, vars),
m_floor(getCurrentDomain(vars[0])), 
m_val(getCurrentDomain(vars[1])) {
  checkError(vars.size()==2, "Exactly 2 parameters required.");
}


void FloorConstraint::handleExecute() {
  EUROPA::edouble f_lb, f_ub, v_lb, v_ub;
  // restrict m_abs based on m_val
  m_val.getBounds(v_lb, v_ub);
  
  if( v_lb<=std::numeric_limits<EUROPA::edouble>::minus_infinity() )
    f_lb = std::numeric_limits<EUROPA::eint>::minus_infinity();
  else
    f_lb = std::floor(v_lb);
  if( v_ub>=std::numeric_limits<EUROPA::edouble>::infinity() )
    f_ub = std::numeric_limits<EUROPA::eint>::infinity();
  else
    f_ub = std::floor(v_ub);
  
  m_floor.intersect(f_lb, f_ub);
  
  // constrain m_val based on m_abs
  m_floor.getBounds(f_lb, f_ub);
  
  if( f_lb<=std::numeric_limits<EUROPA::eint>::minus_infinity() )
    v_lb = std::numeric_limits<EUROPA::edouble>::minus_infinity();
  else if( f_lb>v_lb )
    v_lb = f_lb;
  if( f_ub>=std::numeric_limits<EUROPA::eint>::infinity() )
    v_ub = std::numeric_limits<EUROPA::edouble>::infinity();
  else if( v_ub>(f_ub+1.0) )
    v_ub = f_ub+1.0;
  m_val.intersect(v_lb, v_ub);
}


// Europa messes with me ...
#ifdef max
# undef max 
#endif // max

#ifdef min
# undef min 
#endif // max

/*
 * class TREX:europa::MaxConstraint
 */
MaxConstraint::MaxConstraint(EUROPA::LabelStr const &name,
			     EUROPA::LabelStr const &propagatorName,
			     EUROPA::ConstraintEngineId const &cstrEngine,
			     std::vector<EUROPA::ConstrainedVariableId> 
			     const &vars)
 :EUROPA::Constraint(name, propagatorName, cstrEngine, vars) {
  //  TODO need to check that arguments are correct
}

void MaxConstraint::handleExecute() {
  EUROPA::Domain &result = getCurrentDomain(m_variables[0]);
  EUROPA::edouble lb, ub, cur_max;
  cur_max = result.getUpperBound();
  
  for(int i=1; i<m_variables.size(); ++i) {
    EUROPA::Domain &arg = getCurrentDomain(m_variables[i]);
    EUROPA::edouble ilb, iub;
    arg.getBounds(ilb, iub);
    arg.intersect(ilb, cur_max);
    if( arg.isEmpty() )
      return;
    if( 1==i ) {
      lb = ilb;
      ub = iub;
    } else {
      using std::max;
      lb = max(lb, ilb);
      ub = max(ub, iub);
    }
  }
  result.intersect(lb, ub);
}

/*
 * class TREX:europa::MinConstraint
 */
MinConstraint::MinConstraint(EUROPA::LabelStr const &name,
			     EUROPA::LabelStr const &propagatorName,
			     EUROPA::ConstraintEngineId const &cstrEngine,
			     std::vector<EUROPA::ConstrainedVariableId> 
			     const &vars)
 :EUROPA::Constraint(name, propagatorName, cstrEngine, vars) {
  //  TODO need to check that arguments are correct
}

void MinConstraint::handleExecute() {
  EUROPA::Domain &result = getCurrentDomain(m_variables[0]);
  EUROPA::edouble lb, ub, cur_min;
  cur_min = result.getLowerBound();

  for(int i=1; i<m_variables.size(); ++i) {
    EUROPA::Domain &arg = getCurrentDomain(m_variables[i]);
    EUROPA::edouble ilb, iub;
    arg.getBounds(ilb, iub);
    arg.intersect(cur_min, iub);
    if( arg.isEmpty() )
      return;
    if( 1==i ) {
      lb = ilb;
      ub = iub;
    } else {
      using std::min;
      lb = min(lb, ilb);
      ub = min(ub, iub);
    }
  }
  result.intersect(lb, ub);
}



















