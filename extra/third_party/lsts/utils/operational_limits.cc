#include "private/op_limits_pimpl.hh"
#include <DUNE/IMC/Definitions.hpp>
#include <DUNE/Coordinates/WGS84.hpp>
#include <DUNE/Math/Angles.hpp>

#include <utility>

using DUNE::IMC::OperationalLimits;

using namespace trex_lsts;

/*
 * class trex_lsts::op_limits::pimpl
 */

// structors

op_limits::pimpl::pimpl():m_changed(false) {}

op_limits::pimpl::~pimpl() {}

// observers

bool op_limits::pimpl::is_set() const {
  return NULL!=m_limits.get();
}

bool op_limits::pimpl::is_fresh() const {
  return m_changed.exchange(false);
}


op_limits::range op_limits::pimpl::valid_depths() const {
  range::bound lo(0.0), hi = range::plus_inf;
  if( is_set() ) {
    if( m_limits->mask & DUNE::IMC::OPL_MAX_DEPTH )
      hi = range::bound(m_limits->max_depth);
  }
  return range(lo, hi);
}

op_limits::range op_limits::pimpl::valid_altitudes() const {
  range::bound lo(0.0), hi = range::plus_inf;
  if( is_set() ) {
    if( m_limits->mask & DUNE::IMC::OPL_MIN_ALT )
      lo = range::bound(m_limits->min_altitude);
    if( m_limits->mask & DUNE::IMC::OPL_MAX_ALT )
      hi = range::bound(m_limits->max_altitude);
  }
  return range(lo, hi);
}

op_limits::range op_limits::pimpl::valid_speeds() const {
  range::bound lo(0.0), hi = range::plus_inf;
  if( is_set() ) {
    if( m_limits->mask & DUNE::IMC::OPL_MIN_SPEED )
      lo = range::bound(m_limits->min_speed);
    if( m_limits->mask & DUNE::IMC::OPL_MAX_SPEED )
      hi = range::bound(m_limits->max_speed);
  }
  return range(lo, hi);
}

boost::optional<op_limits::safe_area> op_limits::pimpl::valid_area() const {
  if( is_set() && ( m_limits->mask & DUNE::IMC::OPL_AREA )) {
    safe_area ret;
    ret.latitude = m_limits->lat;
    ret.longitude = m_limits->lon;
    ret.half_length = 0.5*m_limits->length;
    ret.half_width = 0.5*m_limits->width;
    ret.orientation = m_limits->orientation;
    
    return ret;
  } else
    return boost::optional<safe_area>();
}



// modifiers

void op_limits::pimpl::set_limits(DUNE::IMC::OperationalLimits const &lim) {
  lock_guard lock(mtx());
  m_limits.reset(new DUNE::IMC::OperationalLimits(lim));
  m_changed.store(true);
}

/*
 * class trex_lsts::op_limits
 */

using DUNE::Coordinates::WGS84;
using DUNE::Math::Angles;

// statics

op_limits::range const op_limits::s_positives(0.0, op_limits::range::plus_inf);


// structors

op_limits::op_limits():m_impl(new pimpl) {
  
}

op_limits::~op_limits() {}

// observers

bool op_limits::is_set() const {
  return m_impl->is_set();
}

op_limits::range op_limits::valid_depths() const {
  refresh();
  if( NULL!=m_depths.get() )
    return *m_depths;
  else
    return s_positives;
}

op_limits::range op_limits::valid_altitudes() const {
  refresh();
  if( NULL!=m_altitudes.get() )
    return *m_altitudes;
  else
    return s_positives;
}

op_limits::range op_limits::valid_speed() const {
  refresh();
  if( NULL!=m_speeds.get() )
    return *m_speeds;
  else
    return s_positives;
}

boost::optional<op_limits::safe_area> op_limits::get_area() const {
  refresh();
  return m_area;
}


bool op_limits::is_valid(float_type lat, float_type lon) const {
  boost::optional<safe_area> area = get_area(); // Get local copy of area for thread safety
  
  if( !area )
    // No area defined -> any coordinate is valid
    return true;
  else {
    double x, y;
    static bool const clockwise = true;
    
    // Get (northing, easting) displacement between safe center and location
    WGS84::displacement(area->latitude, area->longitude, 0, lat, lon, 0, &x, &y);
    // Rotate northing easting based on aorientation to align with width and length
    Angles::rotate(area->orientation, clockwise, x, y);
    
    // check that both remain below their respective half dimensions of the box
    return ( std::abs(x) < area->half_length ) && ( std::abs(y) < area->half_width );
    
  }
}


// manipulators

void op_limits::refresh() const {
  pimpl::lock_guard read(m_impl->mtx());
  if( m_impl->is_fresh() ) {
    m_depths.reset(new range(m_impl->valid_depths()));
    m_altitudes.reset(new range(m_impl->valid_altitudes()));
    m_speeds.reset(new range(m_impl->valid_speeds()));
    m_area = m_impl->valid_area();
  }
}
