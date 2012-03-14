#include "Constraints.hh"

#include <trex/europa/EuropaPlugin.hh>
#include <trex/europa/Assembly.hh>


#include <mbari/shared/GeoUTM.hh>

namespace {
  int const WGS_84 = 23;

  void geo_to_utm(double lat, double lon, double &north, double &east) {
    char zone[4];
    LLtoUTM(WGS_84, lat, lon, north, east, zone);
  }

  void utm_to_geo(double north, double east, double &lat,  double &lon) {
    const char* Zone = "10";
    const int NORTHERN_HEMISPHERE_BUFFER = 10000000;
    UTMtoLL(WGS_84, 
	    north + NORTHERN_HEMISPHERE_BUFFER, east, 
	    Zone, lat, lon);
    if(east == 0.0)
      lon = -126.0;
  }

  bool intersect(EUROPA::Domain& dom, 
		 double lb, double ub, double precision_error){

    double _lb = (lb > dom.getUpperBound() ? lb - precision_error : lb);
    double _ub = (ub < dom.getLowerBound() ? ub + precision_error : ub);

    return dom.intersect(_lb, _ub);
  }

  class MBARIPlugin :public TREX::europa::EuropaPlugin {
  public:
    void registerComponents(TREX::europa::Assembly const &assembly) {
      TREX_REGISTER_CONSTRAINT(assembly,mbari::europa::GeoUTMConstraint, 
			       geo_to_utm, trex);
    }
  };

  MBARIPlugin europa_extensions;

}

using namespace mbari::europa;

GeoUTMConstraint::GeoUTMConstraint(EUROPA::LabelStr const &name,
				   EUROPA::LabelStr const &propagator,
				   EUROPA::ConstraintEngineId const &cstrEngine,
				   std::vector<EUROPA::ConstrainedVariableId> const &vars)
  :EUROPA::Constraint(name, propagator, cstrEngine, vars),
   m_lat(getCurrentDomain(m_variables[GeoUTMConstraint::LAT])),
   m_lon(getCurrentDomain(m_variables[GeoUTMConstraint::LON])),
   m_northing(getCurrentDomain(m_variables[GeoUTMConstraint::NORTH])),
   m_easting(getCurrentDomain(m_variables[GeoUTMConstraint::EAST])) {
  // assert ?
}

void GeoUTMConstraint::handleExecute() {
  static EUROPA::IntervalIntDomain const UTM_10(-126.0, -120.0001);
  // Lat lon precision is set to 0.018 seconds of a degree ~= 0.000005
  // this correspond roughly on 50cm precision in UTM
  static double const ll_precision = 0.00001;
  static double const utm_precision = 0.5;

  double lat, lon, north, east;

  // ensure that longitude reflects UTM 10 zone
  m_lon.intersect(UTM_10);

  if( m_lat.isSingleton() && m_lon.isSingleton() ) {
    /* convert fixed (lat,lon) into UTM */
    lat = EUROPA::cast_double(m_lat.getSingletonValue());
    lon = EUROPA::cast_double(m_lon.getSingletonValue());
        
    geo_to_utm(lat, lon, north, east);
    if( intersect(m_northing, north, north, utm_precision) &&
	m_northing.isEmpty() )
      return;
    if( intersect(m_easting, east, east, utm_precision) &&
	m_easting.isEmpty() )
      return;
  } else if( m_northing.isSingleton() && m_easting.isSingleton() ) {
    /* convert fixed UTM into (lat,lon) */
    north = EUROPA::cast_double(m_northing.getSingletonValue());
    east = EUROPA::cast_double(m_easting.getSingletonValue());
    utm_to_geo(north, east, lat, lon);
    
    if( intersect(m_lat, lat, lat, ll_precision) &&
	m_lat.isEmpty() )
      return;
    if( intersect(m_lon, lon, lon, ll_precision) &&
	m_lon.isEmpty() )
      return;
 } 
}
