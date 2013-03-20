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
/** @file "earth_point.hh"
 * @brief Utilities to manipulate points in the earth frame
 *
 * Provide sa set of tools to represent and manipulate points that are in earth 
 * coordinates.
 *
 * @note Many of the code to compute distance and headings is borrsowe from this 
 * page http://www.movable-type.co.uk/scripts/latlong.html
 *
 * @author Frederic Py
 */
#ifndef H_mbari_shared_earth_point
# define H_mbari_shared_earth_point

# include <string>
# include <stdexcept>

namespace mbari {
  
  /** @brief earth frame coordinate point
   *
   * This class alow to mainitin and manipulate a point in the earth 
   * frame reference. It maintins this poiunt as a latitude-longitude point 
   * along with -- when available -- the UTM coordinates of this point.
   *
   * It also provides a configurable way to compute the distance between two 
   * point along with its bearing (should it be just initial or constant along 
   * the line)
   *
   * @author Frederic Py <fpy@mbari.org>
   * @ingroup mbari
   */
  class earth_point {
  public:
    /** @brief UTM related exception
     *
     * This exception can be uthrown whenever an invalid operation is made 
     * related to UTM. such error could be:
     * @li invalid UTM zone
     * @li point is outside of UTM coverage
     * @li ...
     *
     * @relates earth_point
     * @ingroup mbari
     */
    class utm_error :public std::runtime_error {
    public:
      utm_error(std::string const &msg) throw()
      :std::runtime_error(msg) {}
      virtual ~utm_error() throw() {}
    };
    /** @brief bad coordinate
     *
     * This exception is thrown wheneve a coordinate is invalid
     *
     * @relates earth_point
     * @ingroup mbari
    */
    class bad_coordinate :public std::runtime_error {
    public:
      bad_coordinate(std::string const &msg) throw()
      :std::runtime_error(msg) {}
      virtual ~bad_coordinate() throw() {}
    };
    
    /** @brief earth radius
     *
     * This is the estimated radius of earth used for computations.
     * The value is expressed in meters.
     */
    static long double const s_radius;
    /** @brief UTM valid letters
     *
     * This string list all the letters used for representing UTM zones.
     * It is mosdtly used for internal purpose.
     */
    static std::string const s_utm_letters;
    
    /** @brief Check if valid UTM letter
     *
     * @param[in] letter A character
     *
     * Check if @p letter is a valid letter for an UTM zone letter. 
     * @note In the current version we only accept the uppercase version of 
     * the UTM letters (i.e 'S' is valid while 's' is not)
     *
     * @retval true if @p letter is valid
     * @retval false otherwise
     *
     * @sa is_utm_number(unsigned short)
     * @sa is_utm_zone(unsigned short, char) 
     */
    static bool is_utm_letter(char letter) {
      return std::string::npos!=letter_to_rank(letter);
    }
    /** @brief Check if valid UTM number
     *
     * @param[in] number a number
     *
     * Check if @p number is a valid UTM zone number. 
     * The number needs to be between 1 and 60.
     * 
     * @retval true if @p number is valid
     * @retval false otherwise
     *
     * @sa is_utm_letter(char)
     * @sa is_utm_zone(unsigned short, char)
     */    
    static bool is_utm_number(unsigned short number) {
      return number>1 && number<=60;
    }
    /** @brief Check if valid UTM zone
     *
     * @param[in] number a number
     * @param[in] letter a charcter
     *
     * Check if the pair (@p number, @p letter) is a valid UTM zone.
     * @note while it is necessary that both @c is_utm_letter and 
     * @c is_utm_number are true for this to be true, do know that some couples 
     * are invalid despite it (32X and 34X)
     *
     * @retval true if (@p number, @p letter) is valid
     * @retval false otherwise
     *
     * @sa is_utm_letter(char)
     * @sa is_utm_number(unsigned short)
     */
    static bool is_utm_zone(unsigned short number, char letter);
    
    /** @{
     * @brief Get latitude range for a UTM zone
     * @param[in] utm_nbr An UTM zone number (not used)
     * @param[in] utm_ltr An UTM zone letter
     *
     * This function gives the latitude range covered by a zone with the UTM 
     * letter @p utm_ltr. 
     *
     * @note As all the zones with a same letter share the same
     * latitude range. It is not necessary to provide the utm zone number. The 
     * 2 arguments form of this method only exists to bhave a oncsistent interface 
     * with @c lon_range method.
     *
     * @pre utm_ltr is a valid UTM letter
     * @throw utm_error @p utm_ltr is invalid
     *
     * @return a pair giving the valid interval of latitude for this zone
     *
     * @sa is_utm_letter(char)
     * @sa lon_range(unsigned short,char)
     */
    static std::pair<double, double> lat_range(char utm_ltr);
    static std::pair<double, double> lat_range(unsigned short utm_nbr, char utm_ltr) {
      return lat_range(utm_ltr);
    }
    /** @}
     * @brief Get longitude range for a UTM zone
     * @param[in] utm_nbr An UTM zone number 
     * @param[in] utm_ltr An UTM zone letter
     *
     * This function gives the latitude range covered by a zone  given by 
     * the pair (@p utm_nbr, @p utm_ltr)
     *
     * @pre (@p utm_nbr, @p utm_ltr) is a valid UTM zone
     * @throw utm_error the zone (@p utm_nbr, @p utm_ltr) is invalid
     *
     * @return a pair giving the valid interval of longitude for this zone
     *
     * @sa is_utm_zone(unsigned short, char)
     * @sa lat_range(char)
     * @sa lat_range(unsigned short,char)
     */
    static std::pair<double, double> lon_range(unsigned short utm_nbr, char utm_ltr);
    
    /** @brief Default constructor
     *
     * Create a new instance with its latitude and longitude set to 0.0.
     * It also compute the UTM coordinate of this point.
     */
    earth_point();
    /** @brief longitude, latitude constructor
     *
     * @param[in] lon A longitude in degrees
     * @param[in] lat A latitude in degrees
     *
     * Create a new instance located at (@p lon, @p lat) and compute when 
     * possible its corresponding UTM coordinates
     *
     * @pre @p lon is a valid longitude between -180 and 180
     * @pre @p lat is a valid latitude between -90 and 90
     *
     * @throw bad_coordinate The coordinate (@p lon, @p lat) is invalid
     */
    earth_point(double lon, double lat);
    /** @brief UTM constructor
     *
     * @param[in] north An UTM northing in meters
     * @param[in] east An UTM easting in meters
     * @param[in] utm_nbr An UTM zone number
     * @param[in] utm_ltr An UTM zone letter 
     *
     * Create a new instance located at the give UTM coordinates     
     * 
     * @pre @p north is valid UTM northing (between 0 and 10000000)
     * @pre @p east is a valid UTM easting (between 0 and 6000000)
     * @pre (@p utm_nbr, @p utm_ltr) lon is a valid UTM zone
     * @pre (@p ntroth, @p east) is within the given UTM zone
     *
     * @throw utm_error The UTM zone is invalid
     * @throw utm_error The resulting latitude or longitude is not in 
     *                  the UTM zone range
     * @note This method can be delicate to use as it require to both know the 
     * UTM norhting and easting but also the corresponding zone. It only exists 
     * as a facility and the lat,lon conunterpart of this constructor should 
     * always be preferred
     *
     * @sa is_utm_zone(unsigned short,char)
     */
    earth_point(double north, double east, unsigned short utm_nbr, char utm_ltr);
    /** @brief Destructor */
    ~earth_point() {}
    
    /** @brief Point latitude
     *
     * @return The latitude of this point
     * @sa longitude() const
     */
    double latitude() const {
      return m_lat;
    }
    /** @brief Point longitude
     *
     * @return The longitude of this point
     * @sa latitude() const
     */
    double longitude() const {
      return m_lon;
    }
    
    /** @brief Check if UTM
     *
     * Checks if this instance can be represented within UTM coordinates. 
     * Indeed points around the polar regions cannot be represented as UTM.
     * 
     * @retval true if the point have UTM coordinates
     * @retval false otherwise
     *
     * @sa utm_number() const
     * @sa utm_letter() const
     * @sa utm_northing() const
     * @sa utm_easting() const
     */
    bool is_utm() const;
    /** @brief UTM zone number
     *
     * Gives when possible the utm zone number for this point.
     *
     * @pre is_utm() should be truwe otherwise the returned value is undefined
     * @return the UTM zone number for this point
     *
     * @sa is_utm() const
     * @sa utm_letter() const
     * @sa utm_northing() const
     * @sa utm_easting() const     
     */
    unsigned short utm_number()   const {
      return m_zone_number;
    }
    /** @brief UTM zone letter
     *
     * Gives when possible the utm zone letter for this point.
     *
     * @pre is_utm() should be truwe otherwise the returned value is undefined
     * @return the UTM zone letter for this point
     *
     * @sa is_utm() const
     * @sa utm_number() const
     * @sa utm_northing() const
     * @sa utm_easting() const
     */
    char           utm_letter()   const {
      return m_zone_letter;
    }
    /** @brief UTM northing (m)
     *
     * Gives when possible the utm northing for this point
     *
     * @pre is_utm() should be truwe otherwise the returned value is undefined
     * @return the UTM northing for this point in meters
     *
     * @sa is_utm() const
     * @sa utm_number() const
     * @sa utm_letter() const
     * @sa utm_easting() const
     */
    double         utm_northing() const {
      return m_northing;
    }
    /** @brief UTM easting (m)
     *
     * Gives when possible the utm easting for this point
     *
     * @pre is_utm() should be truwe otherwise the returned value is undefined
     * @return the UTM easting for this point in meters
     *
     * @sa is_utm() const
     * @sa utm_number() const
     * @sa utm_letter() const
     * @sa utm_northing() const
     */
    double         utm_easting()  const {
      return m_easting;
    }
    
    /** @brief Distance calculator interface
     *
     * This abstract class gives the interface for a distance and bearing 
     * calculator class.
     *
     * There is many ways to compute the distance between 2 points in a globe.
     * They may vary on their complexity and accuracy. Therefore we provide this
     * abstract interface in order to easily augment this class with any method 
     * we want.
     *
     * @relates earth_point
     * @ingroup mbari
     */
    struct nav_calculator {
      /** @brief Constructor */
      nav_calculator() {}
      /** @brief Destructor */
     virtual ~nav_calculator() {}
      
      /** @brief distance between two point
       *
       * @param[in] a A source point
       * @param[in] b A destination point
       *
       * Compute the distance (supposedly in meters) between @p a and @p b
       *
       * @return The computed distance
       *
       * @sa earth_point::distance
       */
      virtual long double distance(earth_point const &a,
                                   earth_point const &b) const =0;
      /** @brief bearing between two point
       *
       * @param[in] a A source point
       * @param[in] b A destination point
       *
       * Compute the bearing (supposedly in degrees) from @p a to @p b
       *
       * @return The computed bearing
       *
       * @sa earth_point::bearing
       */
      virtual double bearing(earth_point const &a,
                             earth_point const &b) const =0;
      /** @brief Projected destination
       *
       * @param[in] a A source point
       * @param[in] heading The heading from @p a (in degrees)
       * @param[in] dist The distance to travel (in meters)
       *
       * compute the destination point from @p a with the given @p heading and 
       * @p distance. It is assumed that the travelling method is the ame as 
       * the one used to compute the diatnce and bearing between 2 points. In 
       * that sense, this method is the inverse function of both @c distance and 
       * @c bearing
       *
       * @return The computed destination
       *
       * @sa earth_point::destination
       * @sa bearing(earth_point const &,earth_point const &) const
       * @sa distance(earth_point const &,earth_point const &) const
       */
      virtual earth_point destination(earth_point const &a,
                                      double heading,
                                      long double dist) const =0;
    };
    
    /** @brief distance to another point
     *
     * @param[in] other The destination
     * @param[in] calc A distance calculation method
     *
     * Compute the distance (supposedly in meters) betweenthis point and 
     * @p other using the nav_calculator @p calc
     *
     * @return The computed distance
     *
     * @sa earth_point::nav_calculator::distance
     */
    long double distance(earth_point const &other, nav_calculator &calc) const {
      return calc.distance(*this, other);
    }
    /** @brief bearing to another point
     *
     * @param[in] other The destination
     * @param[in] calc A distance calculation method
     *
     * Compute the bearing (supposedly in meters) form this point toward
     * @p other using the nav_calculator @p calc
     *
     * @return The computed bearing
     *
     * @sa earth_point::nav_calculator::distance
     */
    double bearing(earth_point const &other, nav_calculator &calc) const {
      return calc.bearing(*this, other);
    }
    /** @brief projected destination
     *
     * @param[in] heading An intial heading (in degrees)
     * @param[in] dist A travel distance (in meters)
     * @param[in] calc A distance calculation method
     *
     * Compute the destination point assuming hat we travel from this point 
     * with and intial @p heading other @p distance. Computation is made using 
     * the nav_calculator @p calc
     *
     * @return The computed destination
     *
     * @sa earth_point::nav_calculator::destination
     */
    earth_point destination(double heading, long double dist,
                            nav_calculator &calc) const {
      return calc.destination(*this, heading, dist);
    }
    
    /** @brief degree to radian
     *
     * @param[in] An angle in degree
     *
     * @return the radian value of @p angle
     *
     * @sa to_deg(long double)
     */
    static long double to_rad(double angle);
    /** @brief radian to degree
     *
     * @param[in] An angle in radian
     *
     * @return the degree value of @p angle
     *
     * @sa to_rad(double)
     */
   static double to_deg(long double angle);
  private:
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
  
  /** @brief Great circles nav calculator
   *
   * This implement a nav_clculator using great circles computation.
   * It computes the diatnce between two points using the shortest arc
   * between those.
   * For this reason the bearing computed is only the inital bearing 
   * from the source point and will probabluy evolve along the path
   *
   * @note this class assumes earth as a perfect sphere. While this may
   * introduce some error theyese errors should be small enough to be 
   * neglected in most cases.
   * 
   * @author Frederic Py
   */
  struct great_circles:public earth_point::nav_calculator {
    great_circles(long double r=earth_point::s_radius):m_radius(r) {}
    virtual ~great_circles() {}
    
    virtual long double distance(earth_point const &a,
                                 earth_point const &b) const;
    virtual double bearing(earth_point const &a,
                           earth_point const &b) const;
    virtual earth_point destination(earth_point const &a,
                                    double heading,
                                    long double dist) const;
    long double const m_radius;
  };
  
  /** @brief Rhumb lines nav calculator
   *
   * This implement a nav_calculator taht compute the shortest path between 
   * 2 point with a constant bearing. Compared to @c great_circles the bearing 
   * given here can be used along all the path which make it easier for 
   * navigation purpose at the cost of a longer path.
   *
   * @note this class assumes earth as a perfect sphere. While this may
   * introduce some error theyese errors should be small enough to be
   * neglected in most cases.
   *
   * @author Frederic Py
   */
  class rhumb_lines:public earth_point::nav_calculator {
  public:
    rhumb_lines(long double r=earth_point::s_radius):m_radius(r) {}
    virtual ~rhumb_lines() {}
    
    virtual long double distance(earth_point const &a,
                                 earth_point const &b) const;
    virtual double bearing(earth_point const &a,
                           earth_point const &b) const;
    virtual earth_point destination(earth_point const &a,
                                    double heading,
                                    long double dist) const;
    long double const m_radius;
  protected:
    static long double delta_phi(long double lat_a, long double lat_b);
    static void align(long double &lon);
    static long double get_q(long double lat_a, long double d_lat,
                             long double d_phi);
    
  };
  
  /** @brief UTM nav calculator
   *
   * This implement a nav_calculator based on UTM coordinates. It assumes that 
   * UTM projection transpose earth in a perfect plane and that we can therefore
   * use classic plane trigonometry.
   *
   * @note As this class assumes UTM as "perfect" the results can loose quickly 
   * precision as the two points gets further (especially if not in the same UTM 
   * zone). Additionally this method won't work if tone of the two point is not 
   * UTM. That being said it is also the only one for now that do not assume 
   * earth as a perfect sphere
   *
   * @warning Currently it does not consider earth as a cylinder either which means 
   * that to travel from Japan to Hawaii it will go through Europe. Also I do not 
   * guarantee results as soon as the two points (including projected destination) 
   * are not in the same zone. 
   *
   * @author Frederic Py
   */
  struct utm_nav:public earth_point::nav_calculator {
    utm_nav() {}
    virtual ~utm_nav() {}
    
    virtual long double distance(earth_point const &a,
                                 earth_point const &b) const;
    virtual double bearing(earth_point const &a,
                           earth_point const &b) const;
    virtual earth_point destination(earth_point const &a,
                                    double heading,
                                    long double dist) const;
  };

}

#endif // earth_point