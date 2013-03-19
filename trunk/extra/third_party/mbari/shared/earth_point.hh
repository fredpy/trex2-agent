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
#ifndef H_mbari_shared_earth_point
# define H_mbari_shared_earth_point

# include <string>
# include <stdexcept>

namespace mbari {
  
  class earth_point {
  public:
    class utm_error :public std::runtime_error {
    public:
      utm_error(std::string const &msg) throw()
      :std::runtime_error(msg) {}
      virtual ~utm_error() throw() {}
    };
    class bad_coordinate :public std::runtime_error {
    public:
      bad_coordinate(std::string const &msg) throw()
      :std::runtime_error(msg) {}
      virtual ~bad_coordinate() throw() {}
    };
    
    static long double const s_radius;
     
    static bool is_utm_letter(char letter) {
      return std::string::npos!=letter_to_rank(letter);
    }
    static bool is_utm_number(unsigned short number) {
      return number>1 && number<=60;
    }
    static bool is_utm_zone(unsigned short number, char letter);
    
    static std::pair<double, double> lat_range(char utm_ltr);
    static std::pair<double, double> lat_range(unsigned short, char utm_ltr) {
      return lat_range(utm_ltr);
    }
    static std::pair<double, double> lon_range(unsigned short utm_nbr, char utm_ltr);
    
    earth_point(double lon, double lat);
    earth_point(double north, double east, unsigned short utm_nbr, char utm_ltr);
    ~earth_point() {}
                
    double latitude() const {
      return m_lat;
    }
    double longitude() const {
      return m_lon;
    }
    
    bool is_utm() const;
    unsigned short utm_number()   const {
      return m_zone_number;
    }
    char           utm_letter()   const {
      return m_zone_letter;
    }
    double         utm_northing() const {
      return m_northing;
    }
    double         utm_easting()  const {
      return m_easting;
    }
    
    long double distance_to(earth_point const &other) const;
    double bearing_to(earth_point const &other) const;
    
    earth_point destination(double bearing, long double dist) const;
    
  private:
    static long double to_rad(double angle);
    static double to_deg(long double angle);

    
    static std::string const s_utm_letters;

    static size_t letter_to_rank(char utm_ltr) {
      return  s_utm_letters.find(utm_ltr);
    }
    static char rank_to_letter(size_t rnk) {
      if( rnk < s_utm_letters.length() )
        s_utm_letters[rnk];
      return '\0';
    }
        
    double m_lat;
    double m_lon;
    double m_northing;
    double m_easting;
    unsigned short m_zone_number;
    char           m_zone_letter;
  };
  
}

#endif // earth_point