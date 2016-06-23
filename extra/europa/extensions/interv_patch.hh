#ifndef H_trex_europa_interv_patch
# define H_trex_europa_interv_patch

#include <boost/numeric/interval.hpp>

namespace TREX {
  namespace europa {


    inline bool
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

    inline bool intersect(EUROPA::Domain &dom, EUROPA::edouble lb, EUROPA::edouble ub, 
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
# define BOOST_DN(EXPR)				\
      this->downward();				\
      T r = this->force_rounding(EXPR);		\
      this->upward();				\
      return r
# define BOOST_NR(EXPR)				\
      this->to_nearest();			\
      T r = this->force_rounding(EXPR);		\
      this->upward();				\
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
    
  }
}


#endif
