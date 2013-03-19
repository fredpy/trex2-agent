#include "Constraints.hh"

#include <trex/europa/EuropaPlugin.hh>
#include <trex/europa/Assembly.hh>

#include <mbari/shared/earth_point.hh>



namespace {

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
   m_easting(getCurrentDomain(m_variables[GeoUTMConstraint::EAST])),
   m_number(getCurrentDomain(m_variables[GeoUTMConstraint::UTM_NUM])),
   m_letter(getCurrentDomain(m_variables[GeoUTMConstraint::UTM_LET])) {
  // assert ?
}

void GeoUTMConstraint::handleExecute() {
  // restrict the zone on its base domain
  m_number.intersect(EUROPA::IntervalIntDomain(1, 60));
  m_lon.intersect(EUROPA::IntervalDomain(-180.0, 180.0));
  m_lat.intersect(EUROPA::IntervalDomain(-80.0, 84.0));
  
  std::list<EUROPA::edouble> values;
  
  for(std::string::const_iterator i=earth_point::s_utm_letters.begin();
      earth_point::s_utm_letters.end()!=i; ++i) {
    EUROPA::edouble val;
    if( m_letter.convertToMemberValue(std::string(1, *i), val) )
      values.push_back(val);
  }
  if( values.empty() ) {
    m_letter.empty();
    return;
  } else {
    EUROPA::EnumeratedDomain tmp(m_letter.getDataType(), values);
    m_letter.intersect(tmp);
  }
  
  if( m_lat.isSingleton() && m_lon.isSingleton() ) {
    earth_point pos(EUROPA::cast_double(m_lon.getSingletonValue()),
                    EUROPA::cast_double(m_lat.getSingletonValue()));
    // Set the zone number
    m_number.intersect(EUROPA::IntervalIntDomain(pos.utm_number()));
    // Set the zone letter
    EUROPA::edouble val;
    if( m_letter.convertToMemberValue(std::string(1, pos.utm_letter()), val) )
      m_letter.set(val);
    else {
      m_letter.empty();
      return;
    }
    
    if( intersect(m_northing, pos.utm_northing(), pos.utm_northing(), 0.5)
       && m_northing.isEmpty() )
      return;
    if( intersect(m_easting, pos.utm_easting(), pos.utm_easting(), 0.5)
       && m_easting.isEmpty() )
      return;
  } else if( m_northing.isSingleton() && m_easting.isSingleton() &&
             m_letter.isSingleton() && m_number.isSingleton() ) {
    double north, east;
    int num;
    
    north = EUROPA::cast_double(m_northing.getSingletonValue());
    east = EUROPA::cast_double(m_easting.getSingletonValue());
    num = EUROPA::cast_int(m_number.getSingletonValue());
    std::string letter_str = m_letter.toString(m_letter.getSingletonValue());
    
    if( earth_point::is_utm_zone(num, letter_str[0]) ) {
      earth_point pos(north, east, num, letter_str[0]);
      if( intersect(m_lat, pos.latitude(), pos.latitude(), 0.00001)
          && m_lat.isEmpty() )
        return;
      if( intersect(m_lon, pos.longitude(), pos.longitude(), 0.00001)
         && m_lat.isEmpty() )
        return;
    } else {
      m_letter.empty();
      m_number.empty();
    }
  }
}
