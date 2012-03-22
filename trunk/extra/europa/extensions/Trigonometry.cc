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
#include "Trigonometry.hh"

#include <PLASMA/ConstrainedVariable.hh>

#include <boost/numeric/interval.hpp>

namespace {
  
  using namespace boost::numeric;
  using namespace interval_lib;

  typedef save_state<rounded_transc_opp<EUROPA::edouble::basis_type> > my_rnd;
  typedef checking_base<EUROPA::edouble::basis_type> my_check;
  typedef interval<EUROPA::edouble::basis_type, policies<my_rnd, my_check> > boost_flt;
  
  boost_flt convert(EUROPA::Domain const &dom) {
    EUROPA::edouble lo, hi;
    EUROPA::edouble::basis_type b_lo = boost_flt::whole().lower(), 
               b_hi = boost_flt::whole().upper();
    dom.getBounds(lo, hi);
    
    if( lo>std::numeric_limits<EUROPA::edouble>::minus_infinity() )
      b_lo = cast_basis(lo);
    if( hi<std::numeric_limits<EUROPA::edouble>::infinity() )
      b_hi = cast_basis(hi);
    return boost_flt(b_lo, b_hi);
  }
  
  inline boost_flt deg_to_rad(boost_flt const &dom) {
    return pi<boost_flt>()*dom/180.0;
  }
  
}

using namespace TREX::europa;

/*
 * class CosineConstraint
 */

CosineConstraint::CosineConstraint(EUROPA::LabelStr const &name,
				   EUROPA::LabelStr const &propagatorName,
				   EUROPA::ConstraintEngineId const &cstrEngine,
				   std::vector<EUROPA::ConstrainedVariableId> const &vars)
:EUROPA::Constraint(name, propagatorName, cstrEngine, vars),
 m_angle(getCurrentDomain(vars[0])), 
 m_cos(getCurrentDomain(vars[1])) {
   checkError(vars.size()==2, "Exactly 2 parameters required.");
}

void CosineConstraint::handleExecute() {
  boost_flt b_angle(convert(m_angle)), b_cos = cos(deg_to_rad(b_angle));
  EUROPA::edouble c_lo(b_cos.lower()), c_hi(b_cos.upper());
  m_cos.intersect(c_lo, c_hi);
}

/*
 * class SineConstraint
 */

SineConstraint::SineConstraint(EUROPA::LabelStr const &name,
			       EUROPA::LabelStr const &propagatorName,
			       EUROPA::ConstraintEngineId const &cstrEngine,
			       std::vector<EUROPA::ConstrainedVariableId> const &vars) 
:EUROPA::Constraint(name, propagatorName, cstrEngine, vars),
m_angle(getCurrentDomain(vars[0])), 
m_sin(getCurrentDomain(vars[1])) {
  checkError(vars.size()==2, "Exactly 2 parameters required.");
}

void SineConstraint::handleExecute() {
  boost_flt b_angle(convert(m_angle)), b_sin = sin(deg_to_rad(b_angle));
  EUROPA::edouble s_lo(b_sin.lower()), s_hi(b_sin.upper());
  m_sin.intersect(s_lo, s_hi);
}
