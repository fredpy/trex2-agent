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

// include plasma header as system files in order to disable warnings
# define TREX_PP_SYSTEM_FILE <PLASMA/ConstrainedVariable.hh>
# include <trex/europa/bits/system_header.hh>

#include "interv_patch.hh"

namespace {
  
  using namespace boost::numeric;
  using namespace interval_lib;

  typedef EUROPA::edouble::basis_type eur_dbl;
  typedef rounding_control<eur_dbl> rnd_ctrl;
  typedef TREX::europa::my_rounded_arith_opp<eur_dbl, rnd_ctrl> rnd_arith;
  typedef rounded_transc_opp<eur_dbl, rnd_arith> rnd_transc;
  typedef save_state<rnd_transc> my_rnd;
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
  inline boost_flt rad_to_deg(boost_flt const &dom) {
    return 180.0*dom/pi<boost_flt>();
  }
  
  
}

using namespace TREX::europa;


/*
 * class RadDeg
 */

InCircle::InCircle(EUROPA::LabelStr const &name,
		   EUROPA::LabelStr const &propagatorName,
		   EUROPA::ConstraintEngineId const &cstrEngine,
		   std::vector<EUROPA::ConstrainedVariableId> const &vars)
:EUROPA::Constraint(name, propagatorName, cstrEngine, vars),
 m_deg(getCurrentDomain(vars[0])), 
 m_val(getCurrentDomain(vars[1])) {
  checkError(vars.size()==2, "Exactly 2 parameters required.");
}

void InCircle::handleExecute() {
  // First restrict the bounds of m_val
  m_deg.intersect(-180.0, 180.0);
  EUROPA::edouble d_hi, d_lo, v_hi, v_lo;
  
  m_deg.getBounds(d_lo, d_hi);
  m_val.getBounds(v_lo, v_hi);

  if( v_hi< std::numeric_limits<EUROPA::edouble>::infinity() ) {
    EUROPA::edouble::basis_type mod = fmod(EUROPA::cast_basis(v_hi), 360.0);
    if( mod<=-180.0 )
      mod += 360.0;
    else if( mod>180.0 )
      mod-= 360.0;
    // Mod is now in the range [-180, 180]
    if( mod>d_hi )
      v_hi -= mod-d_hi;   
  }
  if( v_lo > std::numeric_limits<EUROPA::edouble>::minus_infinity() ) {
    EUROPA::edouble::basis_type mod = fmod(EUROPA::cast_basis(v_lo), 360.0);
    if( mod<=-180.0 )
      mod += 360.0;
    else if( mod>180.0 )
      mod-= 360.0;
    // Mod is now in the range [-180, 180]
    if( mod<d_lo )
      v_lo += d_lo-mod;   
  }
  m_val.intersect(v_lo, v_hi);

  boost_flt val = convert(m_val);
  
  if( width(val)<360.0 ) {
    val = fmod(val, 360.0);
    if( val.lower()>180.0 ) {
      val -= 360.0;
      intersect(m_deg, val.lower(), val.upper(), 1e-6);
    } else if( val.upper()<=-180.0 ) {
      val += 360.0;
      intersect(m_deg, val.lower(), val.upper(), 1e-6);
    } else if( val.lower()>-180.0 && val.upper()<=180.0 )
	intersect(m_deg, val.lower(), val.upper(), 1e-6);
  }
  
}

/*
 * class RadDeg
 */

RadDeg::RadDeg(EUROPA::LabelStr const &name,
	       EUROPA::LabelStr const &propagatorName,
	       EUROPA::ConstraintEngineId const &cstrEngine,
	       std::vector<EUROPA::ConstrainedVariableId> const &vars)
:EUROPA::Constraint(name, propagatorName, cstrEngine, vars),
 m_deg(getCurrentDomain(vars[1])), 
 m_rad(getCurrentDomain(vars[0])) {
  checkError(vars.size()==2, "Exactly 2 parameters required.");
}

void RadDeg::handleExecute() {
  boost_flt b_rad(deg_to_rad(convert(m_deg))), b_deg(rad_to_deg(convert(m_rad)));
  
  intersect(m_rad, b_rad.lower(), b_rad.upper(), 1.0e-8);
  intersect(m_deg, b_deg.lower(), b_deg.upper(), 1.0e-6);
}

/*
 * class CosineConstraint
 */

CosineConstraint::CosineConstraint(EUROPA::LabelStr const &name,
				   EUROPA::LabelStr const &propagatorName,
				   EUROPA::ConstraintEngineId const &cstrEngine,
				   std::vector<EUROPA::ConstrainedVariableId> const &vars)
:EUROPA::Constraint(name, propagatorName, cstrEngine, vars),
 m_angle(getCurrentDomain(vars[1])), 
 m_cos(getCurrentDomain(vars[0])) {
   checkError(vars.size()==2, "Exactly 2 parameters required.");
}

void CosineConstraint::handleExecute() {
  if( m_angle.intersect(-180.0, 180.0) && m_angle.isEmpty() ) 
    return;
  if( m_cos.intersect(-1.0, 1.0) && m_cos.isEmpty() ) 
    return;
  if( m_angle.isSingleton() ) {
    EUROPA::edouble::basis_type tmp = cos(M_PI*EUROPA::cast_basis(m_angle.getSingletonValue())/180.0);
    intersect(m_cos, tmp, tmp, 1e-6);
  } else if( m_cos.isSingleton() ) {
    EUROPA::edouble::basis_type tmp = acos(EUROPA::cast_basis(m_cos.getSingletonValue()));
    tmp *= 180.0/M_PI;
  
    intersect(m_angle, -tmp, tmp, 1e-6);
    if( m_angle.getUpperBound()+1e-6 < tmp )
      intersect(m_angle, -tmp, -tmp, 1e-6);
    else if( 1e-6-tmp < m_angle.getLowerBound() )
      intersect(m_angle, tmp, tmp, 1e-6);
  } else {
    boost_flt b_angle(deg_to_rad(convert(m_angle))), b_cos;
    
    b_cos = sin((pi<boost_flt>()/2.0)-b_angle);
    
    EUROPA::edouble::basis_type c_lo(fmax(-1.0, b_cos.lower())),
      c_hi(fmin(1.0L, b_cos.upper()));
    
    intersect(m_cos, c_lo, c_hi, 1e-6);
  }
}

/*
 * class SineConstraint
 */

SineConstraint::SineConstraint(EUROPA::LabelStr const &name,
			       EUROPA::LabelStr const &propagatorName,
			       EUROPA::ConstraintEngineId const &cstrEngine,
			       std::vector<EUROPA::ConstrainedVariableId> const &vars) 
:EUROPA::Constraint(name, propagatorName, cstrEngine, vars),
m_angle(getCurrentDomain(vars[1])), 
m_sin(getCurrentDomain(vars[0])) {
  checkError(vars.size()==2, "Exactly 2 parameters required.");
}

void SineConstraint::handleExecute() {
  if( m_angle.intersect(-180.0, 180.0) && m_angle.isEmpty() ) 
    return;
  if( m_sin.intersect(-1.0, 1.0) && m_sin.isEmpty() ) 
    return;
  if( m_angle.isSingleton() ) {
    EUROPA::edouble::basis_type tmp = sin(M_PI*EUROPA::cast_basis(m_angle.getSingletonValue())/180.0);
    intersect(m_sin, tmp, tmp, 1e-6);
  } else if( m_sin.isSingleton() ) {
    EUROPA::edouble::basis_type tmp = asin(EUROPA::cast_basis(m_sin.getSingletonValue()));
    tmp *= 180.0/M_PI;
    EUROPA::edouble a_lo, a_hi;

    if( tmp>1e-6 ) {
      a_lo = tmp;
      a_hi = 180.0-tmp;

      if( m_angle.getUpperBound()+1e-6 < a_hi )
	a_hi = a_lo;
      if( a_lo+1e-6 < m_angle.getLowerBound() )
	a_lo = a_hi;
      intersect(m_angle, a_lo, a_hi, 1e-6);
    } else if( tmp<1e-6 ) {
      a_lo = -180.0 - tmp;
      a_hi = tmp;
      
      if( m_angle.getUpperBound()+1e-6 < a_hi )
	a_hi = a_lo;
      if( a_lo+1e-6 < m_angle.getLowerBound() )
	a_lo = a_hi;
      intersect(m_angle, a_lo, a_hi, 1e-6);
      
    } else {
      a_lo = -180.0;
      a_hi = 180.0;
      
      if( m_angle.getLowerBound()-1e-6 > a_lo ) {
	if( m_angle.getLowerBound()<=1e-6 )
	  a_lo = 0.0;
	else 
	  a_lo = 180.0;
      }
      if( m_angle.getUpperBound()+1e-6 < a_hi ) {
	if( m_angle.getUpperBound()>=-1e-6 )
	  a_hi = 0.0;
	else
	  a_hi = -180.0;
      }
      intersect(m_angle, a_lo, a_hi, 1e-6);
    } 
  } else {
    boost_flt b_angle(deg_to_rad(convert(m_angle))), b_sin;
    b_sin = sin(b_angle);

    EUROPA::edouble s_lo(fmax(-1.0L, b_sin.lower())), 
      s_hi(fmin(1.0L, b_sin.upper()));

    intersect(m_sin, s_lo, s_hi, 1e-6);
  }
}
