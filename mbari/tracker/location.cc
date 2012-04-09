#include "location.hh"

using namespace mbari;

void location::update(time_t date, double north, double east) {
  point<2> former(m_last_pos);
  m_last_pos[0] = north;
  m_last_pos[1] = east;

  if( m_valid ) {
    long dt = date-m_date;
    m_speed = m_last_pos - former;
    m_speed /= dt;
    m_have_speed = true;
  } else {
    m_valid = true;
    m_have_speed = false;
  }
  m_date = date;
  m_valid = true;
}

point<2> location::position(time_t now, long int &delta_t) const {
  point<2> estimate(m_last_pos);
  delta_t = now-m_date;
  if( have_speed() )
    estimate += m_speed*delta_t;
  return estimate;
}
