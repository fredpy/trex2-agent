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
long double const earth_point::s_radius(6378137.); // earth radius in meters :
                                                   // same value as for WGS_84


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

earth_point::earth_point():m_lat(0.0), m_lon(0.0) {
  char zone[4];
  LLtoUTM(WGS_84, m_lat, m_lon, m_northing, m_easting, zone);
  m_zone_letter = UTMLetterDesignator(m_lat);
  m_zone_number = 0;
  for(size_t i=0; m_zone_letter!=zone[i]; ++i)
    m_zone_number = (10*m_zone_number)+(zone[i]-'0');  
}


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



/*
 * struct mbari::vincenty
 */


bool vincenty::fixed_point(earth_point const &a, earth_point const &b,
                           long double &sin_l, long double &cos_l,
                           long double &sin_u_a, long double &cos_u_a,
                           long double &sin_u_b, long double &cos_u_b,
                           long double &sigma,
                           long double &sin_sigma,
                           long double &cos_sigma,
                           long double &cos_alpha_2,
                           long double &cos_2_sigma_m) const {
  long double lat_a = earth_point::to_rad(a.latitude()),
     lon_a = earth_point::to_rad(a.longitude()),
     lat_b = earth_point::to_rad(b.latitude()),
    lon_b = earth_point::to_rad(b.longitude());
  long double d_lon = lon_b-lon_a,
    u_a = atanl((1.0L-m_f)*tanl(lat_a)),
    u_b = atanl((1.0L-m_f)*tanl(lat_b));
  
  sin_u_a = sinl(u_a); cos_u_a = cosl(u_a);
  sin_u_b = sinl(u_b); cos_u_b = cosl(u_a);
  
  long double l = d_lon, prev_l;
  size_t iter = 100;
  
  do {
    sin_l = sinl(l); cos_l = cosl(l);
    long double d_x=cos_u_b*sin_l,
      d_y=(cos_u_a*sin_u_b)-(sin_u_a*cos_u_b*cos_l);
    
    sin_sigma = sqrtl((d_x*d_x)+(d_y*d_y));
    if( 0.0L==sin_sigma )
      return false; // notifies that dist is 0 and therefore paramters are not
                    // to be trusted
    cos_sigma = (sin_u_a*sin_u_b) + (cos_u_a*cos_u_b*cos_l);
    
    sigma = atan2l(sin_sigma, cos_sigma);
    
    long double sin_alpha = cos_u_a * cos_u_b * sin_l / sin_sigma;
    
    cos_alpha_2 = 1.0L - sin_alpha*sin_alpha;
    cos_2_sigma_m = cos_sigma;
    cos_2_sigma_m -= 2.0L * sin_u_a * sin_u_b / cos_alpha_2;
  
    if( std::isnan(cos_2_sigma_m) )
      cos_2_sigma_m = 0.0L;
    
    long double c = m_f/16.0L;
    c *= cos_alpha_2;
    c *= 4.0L + m_f*(4.0 - 3.0*cos_alpha_2);
    
    prev_l = l;
    l += (1.0L-c)*m_f*sin_alpha* (sigma + c*sin_sigma* (cos_2_sigma_m + c*cos_sigma * (-1.0L + 2.0L*cos_2_sigma_m*cos_2_sigma_m)));
  } while( fabsl(prev_l-l)>1e-12L && (iter--)>1 );
  
  if( 0==iter ) {
    std::ostringstream oss;
    
    oss<<"Vincenty failed to converge :\n"
    <<" - input:\n"
    <<"    source: <"<<a.longitude()<<", "<<a.latitude()<<">\n"
    <<"    destination: <"<<b.longitude()<<", "<<b.latitude()<<">\n"
    <<"    distance: <"<<earth_point::to_deg(d_lon)<<", "<<earth_point::to_deg(lat_b-lat_a)<<">\n"
    <<" - context:\n"
    <<"    lambda: "<<earth_point::to_deg(l)<<"\n"
    <<"    lambda': "<<earth_point::to_deg(prev_l)<<"pi\n";
    
    
    throw earth_point::bad_coordinate(oss.str());
  }
  
  return true;
}


long double vincenty::distance(earth_point const &a,
                               earth_point const &b) const {
  long double sin_l, cos_l, sin_u_a, cos_u_a, sin_u_b, cos_u_b, sigma,
  sin_sigma, cos_sigma, cos_alpha_2, cos_2_sigma_m;
  
  if( fixed_point(a, b, sin_l, cos_l, sin_u_a, cos_u_a,
                  sin_u_b, cos_u_b, sigma, sin_sigma, cos_sigma,
                  cos_alpha_2, cos_2_sigma_m) ) {
    long double a_2 = m_equator_radius * m_equator_radius,
    b_2 = m_pole_radius*m_pole_radius;
    
    long double u_2 = cos_alpha_2 * (a_2-b_2) / b_2;
    
    long double a_ = 320-175*u_2;
    a_ = -768 + u_2*a_;
    a_ = 4096 + u_2*a_;
    a_ *= u_2/16384;
    a_ += 1.0L;
    long double b_ = 74-47*u_2;
    b_ = -128 + u_2*b_;
    b_ = 256 + u_2*b_;
    b_ *= u_2/1024;
    
    long double delta_sigma = -3.0L + 4.0L*(cos_2_sigma_m*cos_2_sigma_m);
    delta_sigma *= -3.0L + 4.0L*(sin_sigma*sin_sigma);
    delta_sigma *= cos_2_sigma_m;
    delta_sigma *= b_/6.0L;
    delta_sigma = cos_sigma*(-1.0L+ 2.0L*cos_2_sigma_m*cos_2_sigma_m) - delta_sigma;
    delta_sigma = cos_2_sigma_m + b_/4.0L*delta_sigma;
    delta_sigma *= b_*sin_sigma;
    
    return m_pole_radius*a_*(sigma-delta_sigma);
  } else
    return 0.0L;
}

double vincenty::bearing(earth_point const &a,
                              earth_point const &b) const {
  long double sin_l, cos_l, sin_u_a, cos_u_a, sin_u_b, cos_u_b, sigma,
  sin_sigma, cos_sigma, cos_alpha_2, cos_2_sigma_m;
  
  if( fixed_point(a, b, sin_l, cos_l, sin_u_a, cos_u_a,
                  sin_u_b, cos_u_b, sigma, sin_sigma, cos_sigma,
                  cos_alpha_2, cos_2_sigma_m) ) {
    // note: return initial bearing
    return atan2l(cos_u_b*sin_l, (cos_u_a*sin_u_b)-(sin_u_a*cos_u_b*cos_l));
  } else
    return 0.0; // no speed => set bearing to 0
}


earth_point vincenty::destination(earth_point const &a,
                                       double heading,
                                       long double dist) const {
  long double lat_a = earth_point::to_rad(a.latitude()),
  lon_a = earth_point::to_rad(a.longitude()),
  alpha_1 = earth_point::to_rad(heading);
  
  long double sin_alpha_1 = sinl(alpha_1), cos_alpha_1 = cosl(alpha_1),
  tan_u_a = (1.0L-m_f)*tanl(lat_a);
  
  long double cos_u_a = 1.0L / sqrtl(1.0L + tan_u_a*tan_u_a),
  sin_u_a = tan_u_a*cos_u_a,
  sigma_1 = atan2l(tan_u_a, cos_alpha_1),
  sin_alpha = cos_u_a * sin_alpha_1;
  
  long double a_2 = m_equator_radius * m_equator_radius,
  b_2 = m_pole_radius*m_pole_radius;
  long double cos_alpha_2 = 1.0L - sin_alpha*sin_alpha;
  long double u_2 = cos_alpha_2 * (a_2 - b_2)/b_2;
  long double a_ = 320-175*u_2;
  a_ = -768+u_2*a_;
  a_ = 4096+u_2*a_;
  a_ = 1.0L + u_2/16384*a_;
  long double b_ = 74-47*u_2;
  b_ = -128+u_2*b_;
  b_ = 256+u_2*b_;
  b_ *= u_2/1024;
  
  long double sigma = dist / (m_pole_radius*a_), prev_sigma = 2.0L*PI;
  long double sin_sigma = sinl(sigma), cos_sigma=cosl(sigma),
     cos_2_sigma_m = cosl(2.0L*sigma_1 + sigma);
  
  while (fabsl(prev_sigma-sigma) > 1e-12L) {
    long double delta_sigma = -3.0L +4.0L*cos_2_sigma_m*cos_2_sigma_m;
    
    delta_sigma *= -3.0L+4.0L*sin_sigma*sin_sigma;
    delta_sigma *= b_/6.0L*cos_2_sigma_m;
    delta_sigma = cos_sigma*(-1.0L + 2.0L*cos_2_sigma_m*cos_2_sigma_m)-delta_sigma;
    delta_sigma = cos_2_sigma_m + b_/4.0L*delta_sigma;
    delta_sigma *= b_*sin_sigma;
    
    prev_sigma = sigma;
    sigma = dist /(m_pole_radius*a_) + delta_sigma;
    sin_sigma = sinl(sigma); cos_sigma = cosl(sigma);
    cos_2_sigma_m = cosl(2.0L*sigma_1 + sigma);
  }
  
  long double tmp = sin_u_a*sin_sigma - cos_u_a*cos_sigma*cos_alpha_1;
  long double lat_b = atan2l(sin_u_a*cos_sigma + cos_u_a*sin_sigma*cos_alpha_1,
                             (1.0L-m_f)*sqrtl(sin_alpha*sin_alpha + tmp*tmp));
  long double l = atan2l(sin_sigma*sin_alpha_1,
                         cos_u_a*cos_sigma - sin_u_a*sin_sigma*cos_alpha_1);
  long double c = m_f/16.0L;
  c *= cos_alpha_2;
  c *= 4.0L + m_f*(4.0 - 3.0*cos_alpha_2);
  
  long double d_lon = -1.0L+2.0*cos_2_sigma_m*cos_2_sigma_m;
  d_lon = cos_2_sigma_m + c*cos_sigma*d_lon;
  d_lon = sigma + c*sin_sigma*d_lon;
  d_lon = l - (1.0L - c)*m_f*sin_alpha*d_lon;
  long double lon_b = fmodl(lon_a + d_lon + 3.0*PI, 2.0*PI) - PI;
  
  return earth_point(earth_point::to_deg(lon_b), earth_point::to_deg(lat_b));
 }


/*
 * struct mbari::great_circles
 */

/* Haversine formula:	 a = sin²(Δφ/2) + cos(φ1).cos(φ2).sin²(Δλ/2)
 * c = 2.atan2(√a, √(1−a))
 * d = R.c
 * where	φ is latitude, λ is longitude, R is earth’s radius 
 * note that angles need to be in radians to pass to trig functions!
 */
long double great_circles::distance(earth_point const &a,
                                    earth_point const &b) const {
  long double lat_a = earth_point::to_rad(a.latitude()),
  lon_a = earth_point::to_rad(a.longitude()),
  lat_b = earth_point::to_rad(b.latitude()),
  lon_b = earth_point::to_rad(b.longitude());
  
  long double d_lat = lat_b-lat_a, d_lon = lon_b-lon_a;
  long double sin_lat = sinl(d_lat/2.0), sin_lon=sinl(d_lon/2.0);
  
  long double a_ = sin_lat*sin_lat;
  a_ *= sin_lon*sin_lon*cosl(lat_a)*cosl(lat_b);
  
  long double c = atan2l(sqrtl(a_), sqrtl(1.0-a_))*2.0;
  return c*m_radius;
}

/*
 * Formula:	θ = atan2( sin(Δλ).cos(φ2), cos(φ1).sin(φ2) − sin(φ1).cos(φ2).cos(Δλ) )
 */
double great_circles::bearing(earth_point const &a,
                              earth_point const &b) const {
  long double lat_a = earth_point::to_rad(a.latitude()),
  lon_a = earth_point::to_rad(a.longitude()),
  lat_b = earth_point::to_rad(b.latitude()),
  lon_b = earth_point::to_rad(b.longitude());
  
  long double d_lon = lon_b-lon_a;
  long double y = sinl(d_lon)*cosl(lat_b),
  x = cosl(lat_a)*sinl(lat_b);
  x -= sinl(lat_a)*cosl(lat_b)*cosl(d_lon);
  
  return earth_point::to_deg(atan2l(y, x));
}


/*
 * Formula:	φ2 = asin( sin(φ1)*cos(d/R) + cos(φ1)*sin(d/R)*cos(θ) )
 * λ2 = λ1 + atan2( sin(θ)*sin(d/R)*cos(φ1), cos(d/R)−sin(φ1)*sin(φ2) )
 * where	φ is latitude, λ is longitude, θ is the bearing (in radians, 
 * clockwise from north), d is the distance travelled, R is the earth’s radius 
 * (d/R is the angular distance, in radians)
 */
earth_point great_circles::destination(earth_point const &a,
                                       double heading,
                                       long double dist) const {
  long double lat_a = earth_point::to_rad(a.latitude()),
  lon_a = earth_point::to_rad(a.longitude()),
  b_rad = earth_point::to_rad(heading),
  d_r = dist/m_radius;
  
  long double sin_lat = sinl(lat_a), cos_lat=cosl(lat_a),
  sin_r = sinl(d_r), cos_r = cosl(d_r);
  
  long double lat_r = asinl((sin_lat*cos_r)+(cos_lat*sin_r*cosl(b_rad))),
  lon_r = lon_a+atan2l(sinl(b_rad)*sin_r*cos_lat,
                       cos_r-(sin_lat*sin_lat));
  return earth_point(earth_point::to_deg(lon_r), earth_point::to_deg(lat_r));
}

/*
 * class mbari::rhumb_lines
 */

long double rhumb_lines::delta_phi(long double lat_a, long double lat_b) {
  return logl(tanl((lat_b/2.0)+ FOURTHPI)/tanl((lat_a/2.0)+ FOURTHPI));
}

void rhumb_lines::align(long double &lon) {
  if( lon>PI )
    lon -= PI*2.0;
  else if( lon<(-PI) )
     lon += PI*2.0;
}

long double rhumb_lines::get_q(long double lat_a, long double d_lat,
                               long double d_phi) {
  if( 0.0==d_phi )
    return cosl(lat_a);
  else
    return d_lat/d_phi;
}

/*
 * Formula: Δφ′ = ln(tan(π/4+φ2/2)/tan(π/4+φ1/2) )	 (the ‘stretched’ latitude difference)
 * if E:W line,	q = cos(φ1)
 *    otherwise,	q = Δφ/Δφ′
 * d = √(Δφ² + q².Δλ²).R	(pythagoras)
 * where φ is latitude, λ is longitude, Δλ is taking shortest route (<180º), 
 * R is the earth’s radius, ln is natural log
 */
long double rhumb_lines::distance(earth_point const &a,
                                  earth_point const &b) const {
  long double lat_a = earth_point::to_rad(a.latitude()),
  lon_a = earth_point::to_rad(a.longitude()),
  lat_b = earth_point::to_rad(b.latitude()),
  lon_b = earth_point::to_rad(b.longitude());
  long double d_lat = lat_b-lat_a,
  d_lon = lon_b-lon_a;
  
  long double q = get_q(lat_a, d_lat, delta_phi(lat_a, lat_b));
  align(d_lon);
  
  return sqrtl(d_lat*d_lat + q*q*d_lon*d_lon)*m_radius;
}

/*
 * Formula: Δφ′ = ln(tan(π/4+φ2/2)/tan(π/4+φ1/2) )	 (the ‘stretched’ latitude difference)
 * if E:W line,	q = cos(φ1)
 *    otherwise,	q = Δφ/Δφ′
 * θ = atan2(Δλ, Δφ′)
 * where φ is latitude, λ is longitude, Δλ is taking shortest route (<180º),
 * R is the earth’s radius, ln is natural log
 */
double rhumb_lines::bearing(earth_point const &a,
                            earth_point const &b) const {
  long double lat_a = earth_point::to_rad(a.latitude()),
  lon_a = earth_point::to_rad(a.longitude()),
  lat_b = earth_point::to_rad(b.latitude()),
  lon_b = earth_point::to_rad(b.longitude());
  long double d_lon = lon_b-lon_a;
  
  align(d_lon);
  return earth_point::to_deg(atan2l(d_lon, delta_phi(lat_a, lat_b)));
}


/*
 * Formula:	α = d/R	(angular distance)
 * φ2 = φ1 + α.cos(θ)
 * Δφ′ = ln( tan(π/4+φ2/2) / tan(π/4+φ1/2) )	(the ‘stretched’ latitude difference)
 * if E:W line,	q = cos(φ1)
 *   otherwise,	q = Δφ/Δφ′
 * Δλ = α.sin(θ)/q
 * λ2 = (λ1+Δλ+π) % 2.π − π
 * where	φ is latitude, λ is longitude, Δλ is taking shortest route (<180°), 
 * ln is natural log and % is modulo, R is the earth’s radius
 */
earth_point rhumb_lines::destination(earth_point const &a,
                                     double heading,
                                     long double dist) const {
  // Convert my angles into radian
  long double lat_a = earth_point::to_rad(a.latitude()),
  lon_a = earth_point::to_rad(a.longitude()),
  b_rad = earth_point::to_rad(heading),
  // compute the angular distance
  alpha = dist/m_radius;
  
  long double d_lat = alpha*cosl(b_rad);
  
  long double lat_b = lat_a+d_lat;
  long double d_phi = delta_phi(lat_a, lat_b);
  
  long double q = get_q(lat_a, d_lat, d_phi);
  long double d_lon = alpha*sinl(b_rad)/q;
  
  if ( lat_b>(PI/2.0) )
    lat_b = PI-lat_b;
  else if( lat_b<(-PI/2.0) )
    lat_b = -(PI+lat_b);
  long double lon_b = fmodl(lon_a+d_lon+PI, PI*2.0) - PI;
 
  return earth_point(earth_point::to_deg(lon_b), earth_point::to_deg(lat_b));
}

/*
 * struct mbari::utm_nav
 */

long double utm_nav::distance(earth_point const &a,
                              earth_point const &b) const {
  if( !a.is_utm() ) {
    std::ostringstream oss;
    oss<<"Cannot compute UTM distance with a source point that is not UTM ("
    <<a.longitude()<<", "<<a.latitude()<<')';
    throw earth_point::utm_error(oss.str());
  }
  if( !b.is_utm() ) {
    std::ostringstream oss;
    oss<<"Cannot compute UTM distance with a destination point that is not UTM ("
    <<b.longitude()<<", "<<b.latitude()<<')';
    throw earth_point::utm_error(oss.str());
  }
  // TODO: I need to handle the case where the points are closer through
  // the Pacific
  long double d_north = a.utm_northing(),
              d_east = a.utm_easting();
  d_north -= b.utm_northing();
  d_east -= b.utm_easting();
  
  return sqrtl((d_north*d_north)+(d_east*d_east));
}

double utm_nav::bearing(earth_point const &a,
                        earth_point const &b) const {
  if( !a.is_utm() ) {
    std::ostringstream oss;
    oss<<"Cannot compute UTM bearing with a source point that is not UTM ("
    <<a.longitude()<<", "<<a.latitude()<<')';
    throw earth_point::utm_error(oss.str());
  }
  if( !b.is_utm() ) {
    std::ostringstream oss;
    oss<<"Cannot compute UTM bearing with a destination point that is not UTM ("
    <<b.longitude()<<", "<<b.latitude()<<')';
    throw earth_point::utm_error(oss.str());
  }
  // TODO: I need to handle the case where the points are closer through
  // the Pacific
  long double d_north = a.utm_northing(),
  d_east = a.utm_easting();
  d_north -= b.utm_northing();
  d_east -= b.utm_easting();
 
  return earth_point::to_deg(atan2l(d_north, d_east));
}

earth_point utm_nav::destination(earth_point const &a,
                                 double heading,
                                 long double dist) const {
  if( !a.is_utm() ) {
    std::ostringstream oss;
    oss<<"Cannot project UTM destination with a source point that is not UTM ("
    <<a.longitude()<<", "<<a.latitude()<<')';
    throw earth_point::utm_error(oss.str());
  }
  long double b_rad = earth_point::to_rad(heading);
  
  long double d_north = dist*cosl(b_rad), d_east = dist*sinl(b_rad);
  
  // TODO: this stongly assumes that we stay on the same zone
  return earth_point(a.utm_northing()+d_north, a.utm_easting()+d_east,
                     a.utm_number(), a.utm_letter());
}


