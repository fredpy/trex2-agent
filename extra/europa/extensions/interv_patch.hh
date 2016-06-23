#ifndef H_trex_europa_interv_patch
# define H_trex_europa_interv_patch

#include <boost/numeric/interval.hpp>

namespace TREX {
  namespace europa {
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
