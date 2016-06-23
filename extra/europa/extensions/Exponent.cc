#include "Exponent.hh"

// include plasma header as system files in order to disable warnings
# define TREX_PP_SYSTEM_FILE <PLASMA/ConstrainedVariable.hh>
# include <trex/europa/bits/system_header.hh>
# define TREX_PP_SYSTEM_FILE <PLASMA/Domains.hh>
# include <trex/europa/bits/system_header.hh>

# include "interv_patch.hh"

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
}

using namespace TREX::europa;
namespace eu=EUROPA;

Exponent::Exponent(eu::LabelStr const &name,
		   eu::LabelStr const &propagatorName,
		   eu::ConstraintEngineId const &cstrEngine,
		   std::vector<eu::ConstrainedVariableId> const &vars)
  :eu::Constraint(name, propagatorName,cstrEngine, vars),
   m_exp(getCurrentDomain(vars[0])),
   m_log(getCurrentDomain(vars[1])) {
  checkError(vars.size()==2, "Exactly 2 parameters required.");
}

void Exponent::handleExecute() {
  static eu::IntervalDomain const positive(0.0, std::numeric_limits<eu::edouble>::infinity());

  if( m_exp.intersect(positive) && m_exp.isEmpty() )
    return;

  if( m_exp.isSingleton() ) {
    eu::edouble::basis_type tmp = std::log(cast_basis(m_exp.getSingletonValue()));
    intersect(m_log, tmp, tmp, 1e-6);
  } else if( m_log.isSingleton() ) {
    eu::edouble::basis_type tmp = std::exp(cast_basis(m_log.getSingletonValue()));
    intersect(m_exp, tmp, tmp, 1e-6);
  }
  // TODO do the interval math
}


