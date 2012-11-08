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
#ifndef H_mbari_location
# define H_mbari_location

# include "point.hh"
# include <boost/date_time/posix_time/ptime.hpp>

namespace mbari {

  /** @brief 2-D location
   *
   * A class used to maintain the location of a drifting point into a 2-D space.
   * When updated with a date and 2-D corrdinates multiples times this class 
   * estimates the speed vector fot the point allowing to project its location
   * for any future date.
   * 
   * @author Frederic Py <fpy@mbari.org>
   * @ingroup tracker
   */
  class location {
  public:
    typedef boost::posix_time::ptime         date_type;
    typedef boost::posix_time::time_duration duration_type;
    
    /** @brief Constructor
     *
     * @create a new instance with no location set
     *
     * @post the location is not valid
     * @sa is_valid() const
     */
    location():m_valid(false), m_have_speed(false) {}

    /** @brief Check for validity
     *
     * Indicates if the location has been set or not
     *
     * @retval true if the location is set
     * @retval false if the location is not set
     */
    bool is_valid() const {
      return m_valid;
    }
    /** @brief Check for speed
     *
     * Check if the location have a speed identified allowing to project 
     * the point in time.  
     *
     * In order to have a pseed this location should have been updated at least 
     * twice.
     *
     * @retval true if this location have a speed
     * @retval false otherwise
     */
    bool have_speed() const {
      return m_have_speed;
    }
    /** @brief update location
     * @param[in] date  A time stamp
     * @param[in] north A northing value
     * @param[in] east  A easting value
     *
     * Updates the location by indicating that the last position was produced at
     * the date @p date and was at (@p north, @p east)
     *
     * @pre @p date is posterior tto the date odf the last update  
     *
     * @post The location is valid
     * @post if the location was valid before then the location have a speed
     *
     * @sa is_valid() const
     * @sa have_speed() const
     */
    void update(date_type const &date, double north, double east);
    /** @brief speed of the location
     *
     * @pre have_speed()
     *
     * @return The current drifting speed vector for this location.
     *
     * @sa have_speed() const;
     */
    point<2> const &speed() const {
      return m_speed;
    }
    /** @brief Estimated position
     *
     * @param[in]  now     A date
     * @param[out] delta   update freshness
     * @param[in]  projected Estimation flag
     *
     * @pre is_valid()
     * @pre @p now is posterior or equal to the date of the last update 
     *
     * This method estimates the position of this location at the date @p now 
     * and indicates how far in the past was the last update
     * The estimation is done as follow :
     * @li if @c have_speed() and @p projected then project the last update to @p now using the speed vector
     * @li otherwise just give the last update position
     *
     * @return A point giving the estimated position for this location
     *
     * @sa is_valid() const
     * @sa have_speed() const
     * @sa update(time_t,double, double)
     */
    point<2> position(date_type const &now, duration_type &delta, 
                      bool projected=true) const; 

  private:
    date_type   m_date;
    bool        m_valid, m_have_speed;
    point<2>    m_last_pos, m_speed;
  }; 

} // mbari

#endif 
