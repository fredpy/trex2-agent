/*********************************************************************
 * Software License Agreement (BSD License)
 * 
 *  Copyright (c) 2011, MBARI.
 *  All rights reserved.
 * 
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 * 
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the TREX Project nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 * 
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */
#include "earth_point.hh"

#include "GeoUTM.hh"
#include "GeoUTM.Constants.h"

#include <sstream>
#include <cmath>

#include <boost/tuple/tuple.hpp>

using namespace mbari;

namespace {
  int const WGS_84 = 23;
}

/*
 * class mbari::earth_point
 */

// statics

std::string const earth_point::s_utm_letters("CDEFGHJKLMNPQRSTUVWX");
long double const earth_point::s_radius_km(6371); // earth radius in km


long double earth_point::to_rad(double angle) {
  long double ret = angle;
  ret *= deg2rad;
  return ret;
}

double earth_point::to_deg(long double angle) {
  angle *= rad2deg;
  return angle;
}

bool earth_point::is_utm_zone(unsigned short number, char letter) {
  if( is_utm_letter(letter) && is_utm_number(number) )
    return 'X'!=letter || ( 32!=number && 34!=number );
  return false;
}

std::pair<double, double> earth_point::lat_range(char utm_ltr) {
  size_t pos = letter_to_rank(utm_ltr);
  
  if( pos==std::string::npos ) {
    std::ostringstream oss;
    oss<<"Invalid UTM letter \'"<<utm_ltr<<'\'';
    throw utm_error(oss.str());
  }
  
  double lower_bound = 8.0;
  lower_bound *= pos;
  lower_bound -= 80.0;
  
  double padding = 8.0;
  if( 'X'==utm_ltr )
    padding = 12.0; // X is larger then the others
  
  return std::make_pair(lower_bound, lower_bound+padding);
}

std::pair<double, double> earth_point::lon_range(unsigned short utm_nbr, char utm_ltr) {
  if( !is_utm_number(utm_nbr) ) {
    std::ostringstream oss;
    oss<<"Invalid UTM number "<<utm_nbr;
    throw utm_error(oss.str());    
  }
  if( !is_utm_letter(utm_ltr) ) {
    std::ostringstream oss;
    oss<<"Invalid UTM letter \'"<<utm_ltr<<'\'';
    throw utm_error(oss.str());
  }
  
  double lower_bound = 6.0, padding = 6.0;
  
  lower_bound *= utm_nbr-1;
  
  if( 'X'==utm_ltr ) {
    switch( utm_nbr ) {
      case 32: // 32X do not exist
      case 34: // 34X do not exist
      {
        std::ostringstream oss;
        oss<<"UTM zone "<<utm_nbr<<utm_ltr<<" does not exist";
        throw utm_error(oss.str());
      }
        break;
      case 37: // 37X starts sightly before and is 1.5x larger
        lower_bound -= 3.0;
      case 31: // 31X is 1.5x larger
        padding = 9.0;
        break;
      case 33: // 33X starts earlier and covers 2 typical zones
      case 35: // 35X starts earlier and covers 2 typical zones
        lower_bound -= 3.0;
        padding = 12.0;
    }
  } else if( 'V'==utm_ltr ) {
    if( 31==utm_nbr )
      // 31V is half the size of other zones
      padding = 3.0;
    else if( 32==utm_nbr ) {
      // 32V starts earlier and is 1.5x larger
      lower_bound -= 3.0;
      padding = 9.0;
    }
  }
  lower_bound -= 180.0; // realign around GMT
  return std::make_pair(lower_bound, lower_bound+padding);
}

// structors

earth_point::earth_point(double lon, double lat)
:m_lat(lat), m_lon(lon) {
  if( lat>90.0 || lat<-90.0 ) {
    std::ostringstream oss;
    oss<<"Latitude "<<lat<<" is outside of [-90, 90] range";
    throw bad_coordinate(oss.str());
  }
  if( lon>180.0 || lon<=-180.0 ) {
    std::ostringstream oss;
    oss<<"Longitude "<<lon<<" is outside of [-180, 180] range";
    throw bad_coordinate(oss.str());
  }
  if( is_utm() ) {
    char zone[4];
    LLtoUTM(WGS_84, m_lat, m_lon, m_northing, m_easting, zone);
    m_zone_letter = UTMLetterDesignator(m_lat);
    m_zone_number = 0;
    for(size_t i=0; m_zone_letter!=zone[i]; ++i)
      m_zone_number = (10*m_zone_number)+(zone[i]-'0');
  }
}

earth_point::earth_point(double north, double east,
                         unsigned short utm_nbr, char utm_ltr)
:m_northing(north), m_easting(east), m_zone_number(utm_nbr), m_zone_letter(utm_ltr) {
  if( !is_utm_zone(m_zone_number, m_zone_letter) ) {
    std::ostringstream oss;
    oss<<"Invalid UTM zone "<<m_zone_number<<m_zone_number;
    throw utm_error(oss.str());
  }
  std::ostringstream zone;
  zone<<m_zone_number<<m_zone_letter;
  
  UTMtoLL(WGS_84, m_northing, m_easting, zone.str().c_str(), m_lat, m_lon);
  double lo, hi;
  
  boost::tie(lo, hi) = lat_range(m_zone_letter);
  if( m_lat<lo || m_lat>hi ) {
    std::ostringstream oss;
    oss<<"Computed latitude was outside of valid range for zone "<<zone.str();
    throw utm_error(oss.str());
  }
  boost::tie(lo, hi) = lon_range(m_zone_number, m_zone_letter);
  if( m_lon<lo || m_lon>hi ) {
    std::ostringstream oss;
    oss<<"Computed longitude was outside of valid range for zone "<<zone.str();
    throw utm_error(oss.str());
  }
}


// observers

bool earth_point::is_utm() const {
  return m_lat>=-80.0 && m_lat<=84.0;
}

// manipulators

double earth_point::distance_to(earth_point const &other) const {
  // Compute the great circle distance (haversine formula)
  
  long double lat_1 = to_rad(latitude()),
  lon_1 = to_rad(longitude()),
  lat_2 = to_rad(other.latitude()),
  lon_2 = to_rad(other.longitude());
  
  long double d_lat = lat_2-lat_1, d_lon = lon_2-lon_1;
  long double sin_lat = sinl(d_lat/2.0), sin_lon = sinl(d_lon/2.0);
  
  long double a = sin_lat*sin_lat;
  a += sin_lon*sin_lon*cosl(lat_1)*cosl(lat_2);
  
  long double c = atan2l(sqrtl(a), sqrtl(1.0-a))*2.0;
  
  return c*s_radius_km*1000.0;
}

double earth_point::bearing_to(earth_point const &other) const {
  // Compute the bearing
  long double lat_1 = to_rad(latitude()),
  lon_1 = to_rad(longitude()),
  lat_2 = to_rad(other.latitude()),
  lon_2 = to_rad(other.longitude());
  long double d_lon = lon_2-lon_1;

  long double y = sinl(d_lon)*cosl(lat_2);
  long double x = cosl(lat_1)*sinl(lat_2);
  x -= sinl(lat_1)*cosl(lat_2)*cosl(d_lon);
  
  return to_deg(atan2l(y,x));
}

earth_point earth_point::destination(double bearing, double dist) const {
  // idenitfy coordinate on a projected point
  long double lat_1 = to_rad(latitude()),
  lon_1 = to_rad(longitude()),
  b_rad = to_rad(bearing);
  long double d_r = dist;
  
  d_r /= s_radius_km*1000.0;
  
  long double lat_r = asinl(sinl(lat_1)*cosl(d_r)+cosl(lat_1)*sinl(d_r)*cosl(b_rad)),
  lon_r = lon_1 + atan2l(sinl(b_rad)*sinl(d_r)*cosl(lat_1),
                         cosl(d_r)-sinl(lat_1)*sinl(lat_1));
  return earth_point(to_deg(lon_r), to_deg(lat_r));
}





