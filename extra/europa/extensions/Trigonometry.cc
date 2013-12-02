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

#include <boost/numeric/interval.hpp>

namespace {
  
  bool
  intersect(EUROPA::Domain& dom, EUROPA::edouble::basis_type lb, EUROPA::edouble::basis_type ub,
	    EUROPA::edouble::basis_type decimal_places)
  {
    EUROPA::edouble const p_inf = std::numeric_limits<EUROPA::edouble>::infinity();
    EUROPA::edouble const n_inf = std::numeric_limits<EUROPA::edouble>::minus_infinity();
    EUROPA::edouble const prec = decimal_places;
      

    if( lb>ub || lb>=p_inf || ub<=n_inf ) {
      dom.empty();
      return true;
    } else {
      EUROPA::edouble lo = n_inf, hi = p_inf;

      // std::cerr<<dom.toString()<<" * ["<<lb<<", "<<ub<<"]"<<std::flush;

      if( lb>lo ) {
	lo = lb;
	if( lo > dom.getUpperBound() )
	  lo -= prec;
      }
      if( ub<hi ) {
	hi = ub;
	if( hi < dom.getLowerBound() )
	  hi += prec;
      }
      bool ret = dom.intersect(lo, hi);

      // if( dom.isEmpty() )
      // 	std::cerr<<" EMPTY"<<std::endl;
      // else
      // 	std::cerr<<" = "<<dom.toString()<<std::endl;

      return ret;
    }
  }

  bool intersect(EUROPA::Domain &dom, EUROPA::edouble lb, EUROPA::edouble ub, 
		 EUROPA::edouble::basis_type decimal_places) {
    
    EUROPA::edouble const prec = decimal_places;
    
    if( lb>dom.getUpperBound() &&
	lb < std::numeric_limits<EUROPA::edouble>::infinity() )
      lb -= prec;
    if( ub<dom.getLowerBound() &&
	ub > std::numeric_limits<EUROPA::edouble>::minus_infinity() )
      ub += prec;
	
    // std::cerr<<dom.toString()<<" * ["<<lb<<", "<<ub<<"]"<<std::endl;
    return dom.intersect(lb, ub);
  }


  /*
   * A replacement to boost::rounded_arith_opp as it has a bug under clang :
   *   - the to_int call made in the boost version needed to be replaced by 
   *     Rounding::to_int to allow the compiler know that this static method 
   *     is coming from the base class 
   */
  template<class T, class Rounding>
  struct my_rounded_arith_opp: Rounding {
    void init() { this->upward(); }
# define BOOST_DN(EXPR) \
this->downward(); \
T r = this->force_rounding(EXPR); \
this->upward(); \
return r
# define BOOST_NR(EXPR) \
this->to_nearest(); \
T r = this->force_rounding(EXPR); \
this->upward(); \
return r
# define BOOST_UP(EXPR) return this->force_rounding(EXPR)
# define BOOST_UP_NEG(EXPR) return -this->force_rounding(EXPR)
    template<class U> T conv_down(U const &v) { BOOST_UP_NEG(-v); }
    template<class U> T conv_up  (U const &v) { BOOST_UP(v); }
    T add_down(const T& x, const T& y) { BOOST_UP_NEG((-x) - y); }
    T sub_down(const T& x, const T& y) { BOOST_UP_NEG(y - x); }
    T mul_down(const T& x, const T& y) { BOOST_UP_NEG(x * (-y)); }
    T div_down(const T& x, const T& y) { BOOST_UP_NEG(x / (-y)); }
    T add_up  (const T& x, const T& y) { BOOST_UP(x + y); }
    T sub_up  (const T& x, const T& y) { BOOST_UP(x - y); }
    T mul_up  (const T& x, const T& y) { BOOST_UP(x * y); }
    T div_up  (const T& x, const T& y) { BOOST_UP(x / y); }
    T median  (const T& x, const T& y) { BOOST_NR((x + y) / 2); }
    T sqrt_down(const T& x)
    { BOOST_NUMERIC_INTERVAL_using_math(sqrt); BOOST_DN(sqrt(x)); }
    T sqrt_up  (const T& x)
    { BOOST_NUMERIC_INTERVAL_using_math(sqrt); BOOST_UP(sqrt(x)); }
    T int_down(const T& x) { return -Rounding::to_int(-x); }
    T int_up  (const T& x) { return Rounding::to_int(x); }
# undef BOOST_DN
# undef BOOST_NR
# undef BOOST_UP
# undef BOOST_UP_NEG
  };

  
  using namespace boost::numeric;
  using namespace interval_lib;

  typedef EUROPA::edouble::basis_type eur_dbl;
  typedef rounding_control<eur_dbl> rnd_ctrl;
  typedef my_rounded_arith_opp<eur_dbl, rnd_ctrl> rnd_arith;
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
      intersect(m_deg, val.lower(), val.upper(), 1e-8);
    } else if( val.upper()<=-180.0 ) {
      val += 360.0;
      intersect(m_deg, val.lower(), val.upper(), 1e-8);
    } else if( val.lower()>-180.0 && val.upper()<=180.0 )
	intersect(m_deg, val.lower(), val.upper(), 1e-8);
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
  
  intersect(m_rad, b_rad.lower(), b_rad.upper(), 1.0e-10);
  intersect(m_deg, b_deg.lower(), b_deg.upper(), 1.0e-8);
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
  boost_flt b_angle(deg_to_rad(convert(m_angle))), b_cos;
  // cos on ubuntu 12.10 appears to be buggy ... 
  // we use cos(angle) = sin(pi/2 - angle) instead
  b_cos = sin((pi<boost_flt>()/2.0)-b_angle);
  
  EUROPA::edouble::basis_type c_lo(fmax(-1.0L, b_cos.lower())), 
    c_hi(fmin(1.0L, b_cos.upper()));
  intersect(m_cos, c_lo, c_hi, 1.0e-10);
  if( m_cos.isSingleton() ) {
    double angle = 180.0*acos(EUROPA::cast_basis(m_cos.getSingletonValue()))/M_PI;
    intersect(m_angle, -angle, angle, 1.0e-8);
    if( m_angle.getUpperBound()<angle )
      intersect(m_angle, -angle, -angle, 1.0e-8);
    else if( m_angle.getLowerBound()>-angle ) 
      intersect(m_angle, angle, angle, 1.0e-8);
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
  boost_flt b_angle(convert(m_angle)), b_sin = sin(deg_to_rad(b_angle));
  EUROPA::edouble s_lo(fmax(-1.0L, b_sin.lower())), 
    s_hi(fmin(1.0L, b_sin.upper()));
  intersect(m_sin, s_lo, s_hi, 1.0e-10);
  if( m_sin.isSingleton() ) {
    // domain based was to hard lets do the singleton case instead
    EUROPA::edouble a_lo = asin(EUROPA::cast_basis(m_sin.getSingletonValue())), a_hi;
    a_lo = 180.0*a_lo/M_PI;
    if( a_lo>0.0 ) {
      a_hi = 180.0-a_lo;

      intersect(m_angle, a_lo, a_hi, 1.0e-8);
      // Check if I can specify it 
      if( m_angle.getUpperBound()<a_hi )
	intersect(m_angle, a_lo, a_lo, 1.0e-8);
      else if( m_angle.getLowerBound()>a_lo )
	intersect(m_angle, a_hi, a_hi, 1.0e-8);
    } else if( a_lo<0.0 ) {
      std::swap(a_lo, a_hi);
      a_lo = -180.0-a_hi;
      
      intersect(m_angle, a_lo, a_hi, 1.0e-8);
      // Check if I can specify it 
      if( m_angle.getUpperBound()<a_hi )
	intersect(m_angle, a_lo, a_lo, 1.0e-8);
      else if( m_angle.getLowerBound()>a_lo )
	intersect(m_angle, a_hi, a_hi, 1.0e-8);
    } else {
      intersect(m_angle, -180.0, 180.0, 1.0e-8);
      if( m_angle.getLowerBound()>0.0 )
	intersect(m_angle, 180.0, 180.0, 1.0e-8);
      else if( m_angle.getUpperBound()<0.0 )
	intersect(m_angle, -180.0, -180.0, 1.0e-8);
      else {
	if( m_angle.getUpperBound()<180.0 )
	  intersect(m_angle, -180.0, 0.0, 1.0e-8);
	if( m_angle.getLowerBound()>-180.0 )
	  intersect(m_angle, 0.0, 180.0, 1.0e-8);
      }
    } 
  }
}
