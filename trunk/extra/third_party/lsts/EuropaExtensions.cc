#include "EuropaExtensions.hh"

#include <PLASMA/CFunctions.hh>

#include <trex/europa/EuropaPlugin.hh>
#include <trex/europa/Assembly.hh>

#include <DUNE/Math/Angles.hpp>
#include <DUNE/Coordinates/WGS84.hpp>

using namespace EUROPA;

/**
 * DIST == ll_dist(lat1, lon1, lat2, lon2)
 * ll_offset(LAT, LON, ?NORTHING, ?EASTING)
 */
namespace TREX {
  namespace LSTS {
    
    DECLARE_FUNCTION_TYPE(RadDeg, to_rad,
                          "deg_to_rad", FloatDT, 1);
    
    DECLARE_FUNCTION_TYPE(LatLonDist, ll_distance,
                          "ll_dist", FloatDT, 4);
    
    DECLARE_FUNCTION_TYPE(PathEmpty, empty,
                          "is_empty", EUROPA::BoolDT, 1);

// declare function do not like ObjectDT ... lets do without it for now
//    DECLARE_FUNCTION_TYPE(PathCar, car,
//                          "carf", EUROPA::ObjectDT, 1);
//    DECLARE_FUNCTION_TYPE(PathCdr, cdr,
//                          "cdrf", EUROPA::ObjectDT, 1);
    
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
      
      
      TREX_REGISTER_CONSTRAINT(assembly, TREX::LSTS::PathEmpty,
                               is_empty, trex);
      TREX_REGISTER_CONSTRAINT(assembly, TREX::LSTS::PathCar,
                               car, trex);
      TREX_REGISTER_CONSTRAINT(assembly, TREX::LSTS::PathCdr,
                               cdr, trex);
      
      declareFunction(assembly, new TREX::LSTS::PathEmptyFunction());
//      declareFunction(assembly, new TREX::LSTS::PathCarFunction());
//      declareFunction(assembly, new TREX::LSTS::PathCdrFunction());
    }
  };
  
  LSTSPlugin europa_extensions;
  
  double const deg_precision(0.000001);
  double const dist_precision(0.5);
  
} // ::


using namespace TREX::LSTS;
using DUNE::Math::Angles;
using DUNE::Coordinates::WGS84;

// statics

//DUNE::IMC::HomeRef *LatLonToOffset::s_home = NULL;
DUNE::IMC::OperationalLimits *InsideOpLimits::s_oplimits = NULL;

//void LatLonToOffset::set_home(DUNE::IMC::HomeRef *home) {
//  s_home = home;
//  debugMsg("lsts:ll_offset", "Home updated to ("<<Angles::degrees(home->lat)<<", "<<Angles::degrees(home->lon)<<")");
//}

void InsideOpLimits::set_oplimits(DUNE::IMC::OperationalLimits *oplimits) {
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
    // compute the same distance over and over again ... and
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
  
  debugMsg("trex:always", m_dist.toString() << " == " << dist);
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
{}

void InsideOpLimits::handleExecute() {
  
  double lat, lon, depth;
  // commented unused variable
  // bool inside = true;
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
  
  if (s_oplimits->mask & DUNE::IMC::OPL_MAX_DEPTH)
    if (depth >= 0 && depth >= s_oplimits->max_depth)
      m_depth.empty();
  
  if (s_oplimits->mask & DUNE::IMC::OPL_MAX_ALT)
    if (depth < 0 && -depth >= s_oplimits->max_altitude)
      m_depth.empty();
  
  if (s_oplimits->mask & DUNE::IMC::OPL_MIN_ALT)
    if (depth < 0 && -depth <= s_oplimits->min_altitude)
      m_depth.empty();
  
  if (s_oplimits->mask & DUNE::IMC::OPL_AREA)
  {
    
    WGS84::displacement(s_oplimits->lat, s_oplimits->lon, 0, lat, lon, 0, &x, &y);
    
    Angles::rotate(s_oplimits->orientation, true, x, y);
    
    double d2limits =
    std::max(std::fabs(x) - 0.5 * s_oplimits->length, std::fabs(y) - 0.5 * s_oplimits->width);
    
    debugMsg("trex:always", "dist2oplimits: " << d2limits);
    if (!(d2limits < 0))
    {
      debugMsg("trex:always", "Outside op limits! " << lat << ", " << lon << ", " << depth);
      m_lat.empty();
    }
  }
  
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
  if( false ) { // FIXME
    if( m_lat.isSingleton() && m_lon.isSingleton() ) {
      lat = cast_basis(m_lat.getSingletonValue());
      lon = cast_basis(m_lon.getSingletonValue());
      //debugMsg("lsts:ll_offset", "<n,e> = displacement(["<<s_home->lat<<", "
      //    <<s_home->lon<<"], ["<<lat<<", "<<lon<<"])");
      //DUNE::Coordinates::WGS84::displacement(s_home->lat, s_home->lon, 0, lat, lon, 0, &n, &e);
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
        //lat = s_home->lat;
        //lon = s_home->lon;
        debugMsg("lsts:ll_offset", "[lat,lon] = displace(["<<lat<<", "<<lon<<"], <"<<n<<", "<<e<<">)");
        DUNE::Coordinates::WGS84::displace(n, e, &lat, &lon);
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
  
  if (m_rad.isSingleton() && m_deg.isSingleton())
    return;
  
  m_rad.getBounds(r_lo, r_hi);
  m_deg.getBounds(d_lo, d_hi);
  
  // this boolean will indicate if I need to restrict rad
  bool t_hi = true, t_lo = true;
  
  if( r_hi < std::numeric_limits<EUROPA::edouble>::infinity() ) {
    tmp = DUNE::Math::Angles::degrees(cast_basis(r_hi));
    if( tmp<=d_hi ) {
      // upper bound restricted => no need to compute rad upperbound
      t_hi = false;
      d_hi = tmp;
    }
  }
  if( std::numeric_limits<EUROPA::edouble>::minus_infinity() < r_lo ) {
    tmp = DUNE::Math::Angles::degrees(cast_basis(r_lo));
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
    r_hi = DUNE::Math::Angles::radians(cast_basis(d_hi));
  if( t_lo && std::numeric_limits<EUROPA::edouble>::minus_infinity() < d_lo )
    r_lo = DUNE::Math::Angles::radians(cast_basis(d_lo));
  m_rad.intersect(r_lo, r_hi);
}

//====================================
// Path list handling constraints
//====================================

namespace {
  
  // Helpers to access attributes of a PathList
  // these are hardcoded indexes which makes it brittle to nddl alteration
  
  EUROPA::ConstrainedVariableId nil_attr(EUROPA::ObjectId const &l) {
    return l->getVariables()[0];
  }
  EUROPA::ConstrainedVariableId car_attr(EUROPA::ObjectId const &l) {
    return l->getVariables()[1];
  }
  EUROPA::ConstrainedVariableId cdr_attr(EUROPA::ObjectId const &l) {
    return l->getVariables()[2];
  }
    
}

/*
 * TREX::LSTS::PathCar
 */

PathCar::PathCar(EUROPA::LabelStr const &name,
                 EUROPA::LabelStr const &propagator,
                 EUROPA::ConstraintEngineId const &engine,
                 std::vector<EUROPA::ConstrainedVariableId> const &vars)
:EUROPA::Constraint(name, propagator, engine, vars) {}

void PathCar::handleExecute() {
  EUROPA::ObjectDomain &headV = dynamic_cast<EUROPA::ObjectDomain &>(getCurrentDomain(m_variables[0]));
  EUROPA::ObjectDomain &listV = dynamic_cast<EUROPA::ObjectDomain &>(getCurrentDomain(m_variables[1]));
  
  std::list<EUROPA::ObjectId> values;
  EUROPA::ObjectSet car_set;
  
  values = listV.makeObjectList();
  
  
  for(std::list<EUROPA::ObjectId>::const_iterator i=values.begin();
      values.end()!=i; ++i) {
    if( EUROPA::cast_int(nil_attr(*i)->lastDomain().getSingletonValue()) )
      listV.remove(*i);
    else if( headV.isMember(car_attr(*i)->lastDomain().getSingletonValue()) ) {
      car_set.insert(headV.getObject(car_attr(*i)->lastDomain().getSingletonValue()));
    } else
      listV.remove(*i);
  }
  
  if( car_set.empty() )
    headV.empty();
  else { 
    values = headV.makeObjectList();
    for(std::list<EUROPA::ObjectId>::const_iterator i=values.begin();
        values.end()!=i; ++i)
      if( car_set.end()==car_set.find(*i) )
        headV.remove(*i);
  }
}

/*
 * TREX::LSTS::PathCdr
 */

PathCdr::PathCdr(EUROPA::LabelStr const &name,
                 EUROPA::LabelStr const &propagator,
                 EUROPA::ConstraintEngineId const &engine,
                 std::vector<EUROPA::ConstrainedVariableId> const &vars)
:EUROPA::Constraint(name, propagator, engine, vars) {}

void PathCdr::handleExecute() {
  EUROPA::ObjectDomain &tailV = dynamic_cast<EUROPA::ObjectDomain &>(getCurrentDomain(m_variables[0]));
  EUROPA::ObjectDomain &listV = dynamic_cast<EUROPA::ObjectDomain &>(getCurrentDomain(m_variables[1]));
  
  std::list<EUROPA::ObjectId> values;
  EUROPA::ObjectSet cdr_set;
  
  values = listV.makeObjectList();
  
  for(std::list<EUROPA::ObjectId>::const_iterator i=values.begin();
      values.end()!=i; ++i) {
    if( EUROPA::cast_int(nil_attr(*i)->lastDomain().getSingletonValue()) ) {
      listV.remove(*i);
    } else {
      EUROPA::ObjectDomain cdr = dynamic_cast<EUROPA::ObjectDomain const &>(cdr_attr(*i)->lastDomain());
      
      cdr.intersect(tailV);
      if( cdr.isEmpty() )
        listV.remove(*i);
      else {
        std::list<EUROPA::ObjectId> tmp = cdr.makeObjectList();
        for(std::list<EUROPA::ObjectId>::const_iterator j=tmp.begin();
            tmp.end()!=j; ++j)
          cdr_set.insert(*j);
      }
    }
  }
  
  if( cdr_set.empty() )
    tailV.empty();
  else {
    values = tailV.makeObjectList();
    for(std::list<EUROPA::ObjectId>::const_iterator i=values.begin();
        values.end()!=i; ++i)
      if( cdr_set.end()==cdr_set.find(*i) )
        tailV.remove(*i);
  }
}

/*
 * TREX::LSTS::PathCdr
 */

PathEmpty::PathEmpty(EUROPA::LabelStr const &name,
                     EUROPA::LabelStr const &propagator,
                     EUROPA::ConstraintEngineId const &engine,
                     std::vector<EUROPA::ConstrainedVariableId> const &vars)
:EUROPA::Constraint(name, propagator, engine, vars) {}

void PathEmpty::handleExecute() {
  EUROPA::Domain &testV = getCurrentDomain(m_variables[0]);  
  EUROPA::ObjectDomain &listV = dynamic_cast<EUROPA::ObjectDomain &>(getCurrentDomain(m_variables[1]));
  
  std::list<EUROPA::ObjectId> values = listV.makeObjectList();
  

  if( testV.isSingleton() ) {
    bool is_empty = EUROPA::cast_int(testV.getSingletonValue());
    
    for(std::list<EUROPA::ObjectId>::const_iterator i=values.begin();
        values.end()!=i; ++i) {
      bool is_nil = EUROPA::cast_int(nil_attr(*i)->lastDomain().getSingletonValue());
      
      if( is_nil!=is_empty ) {
        listV.remove(*i);
      }
    }
  } else {
    bool is_set, last_val;
    
    for(std::list<EUROPA::ObjectId>::const_iterator i=values.begin();
        values.end()!=i; ++i) {
      bool is_nil = EUROPA::cast_int(nil_attr(*i)->lastDomain().getSingletonValue());
      if( is_set ) {
        if( is_nil!=last_val ) {
          is_set = false;
          break;
        }
      } else {
        is_set = true;
        last_val = is_nil;
      }
    }
    if( is_set )
      testV.set(last_val);
  }
}

