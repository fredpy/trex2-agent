#include "EuropaExtensions.hh"

#include <PLASMA/CFunctions.hh>

#include <trex/europa/EuropaPlugin.hh>
#include <trex/europa/Assembly.hh>

#include <Dune/Math/Angles.hpp>
#include <Dune/Coordinates/WGS84.hpp>

/**
 * DIST == ll_dist(lat1, lon1, lat2, lon2)
 * ll_offset(LAT, LON, ?NORTHING, ?EASTING)
 */
namespace TREX {
  namespace LSTS {

    using namespace EUROPA;
    using namespace boost::polygon::operators;
    namespace poly = boost::polygon;

    DECLARE_FUNCTION_TYPE(RadDeg, to_rad,
        "deg_to_rad", FloatDT, 1);

    DECLARE_FUNCTION_TYPE(LatLonDist, ll_distance,
        "ll_dist", FloatDT, 4);

    DECLARE_FUNCTION_TYPE(InsideOpLimits, sane_pos,
            "sane_pos", FloatDT, 4);
  }
}

namespace {

  bool intersect(EUROPA::Domain& dom,
      EUROPA::edouble lb, EUROPA::edouble ub,
      double precision_error) {
    if( ub > dom.getUpperBound()
        && ub < std::numeric_limits<EUROPA::edouble>::infinity() )
      ub -= precision_error;

    if( lb < dom.getLowerBound()
        && std::numeric_limits<EUROPA::edouble>::minus_infinity() < lb )
      lb += precision_error;

    return dom.intersect(lb, ub);
  }


  class LSTSPlugin :public TREX::europa::EuropaPlugin {
  public:
    void registerComponents(TREX::europa::Assembly const &assembly) {
      TREX_REGISTER_CONSTRAINT(assembly,TREX::LSTS::LatLonToOffset,
                               ll_offset, trex);
      TREX_REGISTER_CONSTRAINT(assembly,TREX::LSTS::LatLonDist,
          ll_dist, trex);
      TREX_REGISTER_CONSTRAINT(assembly, TREX::LSTS::RadDeg,
          deg_to_rad, trex);
      TREX_REGISTER_CONSTRAINT(assembly, TREX::LSTS::InsideOpLimits,
                sane_pos, trex);

      declareFunction(assembly, new TREX::LSTS::RadDegFunction());
      declareFunction(assembly, new TREX::LSTS::LatLonDistFunction());
      declareFunction(assembly, new TREX::LSTS::InsideOpLimitsFunction());
    }
  };

  LSTSPlugin europa_extensions;

  double const deg_precision(0.000001);
  double const dist_precision(0.5);

} // ::


using namespace TREX::LSTS;
using Dune::Math::Angles;
using Dune::Coordinates::WGS84;

// statics

Dune::IMC::HomeRef *LatLonToOffset::s_home = NULL;
Dune::IMC::OperationalLimits *InsideOpLimits::s_oplimits = NULL;

void LatLonToOffset::set_home(Dune::IMC::HomeRef *home) { 
  s_home = home;
  debugMsg("lsts:ll_offset", "Home updated to ("<<Angles::degrees(home->lat)<<", "<<Angles::degrees(home->lon)<<")");
}

void InsideOpLimits::set_oplimits(Dune::IMC::OperationalLimits *oplimits) {
  s_oplimits = oplimits;
}


//LATLONDIST STARTS ...

LatLonDist::LatLonDist(EUROPA::LabelStr const &name,
    EUROPA::LabelStr const &propagator,
    EUROPA::ConstraintEngineId const &cstrEngine,
    std::vector<EUROPA::ConstrainedVariableId> const &vars)
:EUROPA::Constraint(name, propagator, cstrEngine, vars),
 m_dist(getCurrentDomain(m_variables[LatLonDist::DIST])),
 m_lat1(getCurrentDomain(m_variables[LatLonDist::LAT1])),
 m_lon1(getCurrentDomain(m_variables[LatLonDist::LON1])),
 m_lat2(getCurrentDomain(m_variables[LatLonDist::LAT2])),
 m_lon2(getCurrentDomain(m_variables[LatLonDist::LON2]))
{}

void LatLonDist::handleExecute() {
  double lat1, lon1, lat2, lon2;

  if( m_dist.isSingleton() ) {
    // While generally incomplete this approach avoid to
    // compute the same distance other and other again ... and 
    // it should work as long as nobody constraints dist 
    // otherwise
    return; 
  }

  if (m_lat1.isSingleton())
    lat1 = cast_basis(m_lat1.getSingletonValue());
  else
    return;
  if (m_lon1.isSingleton())
    lon1 = cast_basis(m_lon1.getSingletonValue());
  else
    return;

  if (m_lat2.isSingleton())
    lat2 = cast_basis(m_lat2.getSingletonValue());
  else
    return;
  if (m_lon2.isSingleton())
    lon2 = cast_basis(m_lon2.getSingletonValue());
  else
    return;

  EUROPA::edouble 
  dist = WGS84::distance(lat1, lon1, 0, lat2, lon2, 0);

  intersect(m_dist, dist, dist, 0.5);
}

//LATLONDIST ENDS ...


//INSIDEOPLIMITS STARTS ...

InsideOpLimits::InsideOpLimits(EUROPA::LabelStr const &name,
    EUROPA::LabelStr const &propagator,
    EUROPA::ConstraintEngineId const &cstrEngine,
    std::vector<EUROPA::ConstrainedVariableId> const &vars)
:EUROPA::Constraint(name, propagator, cstrEngine, vars),
 m_inside(getCurrentDomain(m_variables[InsideOpLimits::INSIDE])),
 m_lat(getCurrentDomain(m_variables[InsideOpLimits::LAT])),
 m_lon(getCurrentDomain(m_variables[InsideOpLimits::LON])),
 m_depth(getCurrentDomain(m_variables[InsideOpLimits::DEPTH]))
{
}

void InsideOpLimits::handleExecute() {

  double lat, lon, depth;
  bool inside = true;
  double x, y;

  if (s_oplimits == NULL)
    return;
  if (m_lat.isSingleton())
    lat = cast_basis(m_lat.getSingletonValue());
  else
    return;
  if (m_lon.isSingleton())
    lon = cast_basis(m_lon.getSingletonValue());
  else
    return;

  if (m_depth.isSingleton())
    depth = cast_basis(m_depth.getSingletonValue());

  if (s_oplimits->mask & Dune::IMC::OperationalLimits::OPL_MAX_DEPTH)
    inside = inside && depth < s_oplimits->max_depth;

  if (s_oplimits->mask & Dune::IMC::OperationalLimits::OPL_AREA)
  {

    WGS84::displacement(s_oplimits->lat, s_oplimits->lon, 0, lat, lon, 0, &x, &y);

    Angles::rotate(s_oplimits->orientation, true, x, y);

    double d2limits =
        std::max(std::fabs(x) - 0.5 * s_oplimits->length, std::fabs(y) - 0.5 * s_oplimits->width);

   // std::cerr << "d2limits: " << d2limits << "\n";
    inside = d2limits < 0;
  }

  //s_oplimits->toText(std::cerr);
  //std::cerr << "inside: " << inside << "\n";

  m_inside.intersect(!!inside, !!inside);

  return;
}

//LATLONDIST ENDS ...

// structors

LatLonToOffset::LatLonToOffset(EUROPA::LabelStr const &name,
    EUROPA::LabelStr const &propagator,
    EUROPA::ConstraintEngineId const &cstrEngine,
    std::vector<EUROPA::ConstrainedVariableId> const &vars)
:EUROPA::Constraint(name, propagator, cstrEngine, vars),
 m_lat(getCurrentDomain(m_variables[LatLonToOffset::LAT])), // lat in radian
 m_lon(getCurrentDomain(m_variables[LatLonToOffset::LON])), // lon in radian
 m_northing(getCurrentDomain(m_variables[LatLonToOffset::NORTH])), // nothing in meter
 m_easting(getCurrentDomain(m_variables[LatLonToOffset::EAST])) { // easting in meter
}
// manipulators
void LatLonToOffset::handleExecute() {
  double lat, lon, n, e;
  if( NULL!=s_home ) {
    if( m_lat.isSingleton() && m_lon.isSingleton() ) {
      lat = cast_basis(m_lat.getSingletonValue());
      lon = cast_basis(m_lon.getSingletonValue());
      debugMsg("lsts:ll_offset", "<n,e> = displacement(["<<s_home->lat<<", "
          <<s_home->lon<<"], ["<<lat<<", "<<lon<<"])");
      Dune::Coordinates::WGS84::displacement(s_home->lat, s_home->lon, 0, lat, lon, 0, &n, &e);
      debugMsg("lsts:ll_offset", "<n,e> = <"<<n<<", "<<e<<">");
      if( intersect(m_northing, n, n, dist_precision)
          && m_northing.isEmpty() ) {
        debugMsg("trex:ll_offset", "n="<<n<<" resulted on an empty northing.");
        return;
      }
      if( intersect(m_easting, e, e, dist_precision)
          && m_easting.isEmpty() ) {
        debugMsg("trex:ll_offset", "e="<<e<<" resulted on an empty easting.");
        return;
      }
    } else {
      if( m_northing.isSingleton() && m_easting.isSingleton() ) {
        n = cast_basis(m_northing.getSingletonValue());
        e = cast_basis(m_easting.getSingletonValue());
        lat = s_home->lat;
        lon = s_home->lon;
        debugMsg("lsts:ll_offset", "[lat,lon] = displace(["<<lat<<", "<<lon<<"], <"<<n<<", "<<e<<">)");
        Dune::Coordinates::WGS84::displace(n, e, &lat, &lon);
        debugMsg("lsts:ll_offset", "[lat,lon] = ["<<lat<<", "<<lon<<"]");
        if( m_lat.intersect(lat, lat)
            && m_lat.isEmpty() ) {
          debugMsg("trex:ll_offset", "lat="<<lat<<" resulted on an empty latitude.");
          return;
        }
        if( m_lon.intersect(lon, lon)
            && m_lon.isEmpty() ) {
          debugMsg("trex:ll_offset", "lon="<<lon<<" resulted on an empty latitude.");
          return;
        }
      }
    }
  }
}

/*
 * class TREX::LSTS::DegRad
 */

// structors

RadDeg::RadDeg(EUROPA::LabelStr const &name,
    EUROPA::LabelStr const &propagator,
    EUROPA::ConstraintEngineId const &cstrEngine,
    std::vector<EUROPA::ConstrainedVariableId> const &vars)
:EUROPA::Constraint(name, propagator, cstrEngine, vars),
 m_deg(getCurrentDomain(m_variables[RadDeg::DEGREES])),
 m_rad(getCurrentDomain(m_variables[RadDeg::RADIANS])) {
}

// manipulators

void RadDeg::handleExecute() {
  EUROPA::edouble d_lo, d_hi, r_lo, r_hi, tmp;

  m_rad.getBounds(r_lo, r_hi);
  m_deg.getBounds(d_lo, d_hi);

  // this boolean will indicate if I need to restrict rad
  bool t_hi = true, t_lo = true;

  if( r_hi < std::numeric_limits<EUROPA::edouble>::infinity() ) {
    tmp = Dune::Math::Angles::degrees(cast_basis(r_hi));
    if( tmp<=d_hi ) {
      // upper bound restricted => no need to compute rad upperbound
      t_hi = false;
      d_hi = tmp;
    }
  }
  if( std::numeric_limits<EUROPA::edouble>::minus_infinity() < r_lo ) {
    tmp = Dune::Math::Angles::degrees(cast_basis(r_lo));
    if( d_lo<=tmp ) {
      // lowerbound restricted => no need to compute red lower bound
      t_lo = false;
      d_lo = tmp;
    }
  }
  // update deg
  if( intersect(m_deg, d_lo, d_hi, deg_precision) &&
      m_deg.isEmpty() )
    return;

  if( t_hi && d_hi < std::numeric_limits<EUROPA::edouble>::infinity() )
    r_hi = Dune::Math::Angles::radians(cast_basis(d_hi));
  if( t_lo && std::numeric_limits<EUROPA::edouble>::minus_infinity() < d_lo )
    r_lo = Dune::Math::Angles::radians(cast_basis(d_lo));
  m_rad.intersect(r_lo, r_hi);
}

