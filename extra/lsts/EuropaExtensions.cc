#include "EuropaExtensions.hh"

#include <PLASMA/CFunctions.hh>

#include <trex/europa/EuropaPlugin.hh>
#include <trex/europa/Assembly.hh>

#include <Dune/Math/Angles.hpp>
#include <Dune/Coordinates/WGS84.hpp>

# include <boost/polygon/polygon.hpp>

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

      declareFunction(assembly, new TREX::LSTS::RadDegFunction());
      declareFunction(assembly, new TREX::LSTS::LatLonDistFunction());

    }
  };

  LSTSPlugin europa_extensions;

  double const deg_precision(0.000001);
  double const dist_precision(0.5);

} // ::


using namespace TREX::LSTS;
using Dune::Math::Angles;
using Dune::Coordinates::WGS84;

/*
 * class TREX::LSTS::LatLonToOffset
 */ 

// statics

Dune::IMC::HomeRef *LatLonToOffset::s_home = NULL;
Dune::IMC::OperationalLimits *InsideOpLimits::s_oplimits = NULL;

void LatLonToOffset::set_home(Dune::IMC::HomeRef *home) { 
  s_home = home;
  debugMsg("lsts:ll_offset", "Home updated to ("<<Angles::degrees(home->lat)<<", "<<Angles::degrees(home->lon)<<")");
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
 m_lat(getCurrentDomain(m_variables[InsideOpLimits::LAT])),
 m_lon(getCurrentDomain(m_variables[InsideOpLimits::LON])),
 m_depth(getCurrentDomain(m_variables[InsideOpLimits::DEPTH]))
{
}

Polygon * InsideOpLimits::computeOpLimitsPolygon()
{
	//double lat, lon;
	double corners[4][2];
	int i;
	Point pts[4];

	if (s_oplimits == NULL || !(s_oplimits->mask & Dune::IMC::OperationalLimits::OPL_AREA))
		return NULL;

	corners[0][0] = +s_oplimits->length/2;
	corners[1][0] = +s_oplimits->length/2;
	corners[2][0] = -s_oplimits->length/2;
	corners[3][0] = -s_oplimits->length/2;

	corners[0][1] = -s_oplimits->width/2;
	corners[1][1] = +s_oplimits->width/2;
	corners[2][1] = +s_oplimits->width/2;
	corners[3][1] = -s_oplimits->width/2;

	for (i = 0; i < 4; i++)
	{
		Angles::rotate(s_oplimits->orientation, true, corners[i][0], corners[i][1]);
		pts[i] = boost::polygon::construct<Point>(corners[i][0], corners[i][1]);
	}

	Polygon * p = new Polygon();

	return p;
};

void InsideOpLimits::handleExecute() {

	double lat, lon;
        // double depth;

	if (s_oplimits == NULL)
		return;


	Point p;

	/*
	if (m_lat.isSingleton())
		lat = cast_basis(m_lat.getSingletonValue());
	else
		return;
	if (m_lon.isSingleton())
		lon = cast_basis(m_lon.getSingletonValue());
	else
		return;
	 */

	if (s_oplimits->mask & Dune::IMC::OperationalLimits::OPL_AREA)
	{
		// double max_lat = s_oplimits->lat;
		// double max_lon = s_oplimits->lon;
		lat = s_oplimits->lat;
		lon = s_oplimits->lon;

		WGS84::displace(s_oplimits->length/2, s_oplimits->width/2, &lat, &lon);


	}
	/*

	      if (msg->ref != DuneIMC::EstimatedState::RM_LLD_ONLY)
	      {
	        x += msg->x;
	        y += msg->y;
	      }

	      Angles::rotate(m_ol.orientation, true, x, y);

	      double d2limits =
	        std::max(std::fabs(x) - 0.5 * m_ol.length, std::fabs(y) - 0.5 * m_ol.width);
	      test(Dune::IMC::OperationalLimits::OPL_AREA, "Operational Area", d2limits, 0);*/

	if (s_oplimits->mask & Dune::IMC::OperationalLimits::OPL_MAX_DEPTH)
		intersect(m_depth, 0, s_oplimits->max_depth, 0);
	else
		intersect(m_depth, 0, m_depth.getUpperBound(), 0);

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

