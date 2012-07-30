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
#ifndef H_mbari_serie
# define H_mbari_serie

# include <map>
# include <vector>
# include <functional>

# include <boost/date_time/posix_time/ptime.hpp>

# include "point.hh"


namespace mbari {

  /** @brief MBARI utilities implementation details
   *
   * This namespace provides tools used for implementing mbari classes. 
   * Most of the tools found here are not meant to be used directly but 
   * just serve the implementation of the mbari components
   *
   * @author Frederic Py
   * @ingroup mbari
   */
  namespace details {
    
    /** @brief Identity functor
     *
     * @tparam Ty the argument type
     *
     * A functor that will returns its argument. This functor can be
     * useful in conjunction with standard STL algorithms when we 
     * just want to extract the current values from a container.
     *
     * @author Frederic Py
     * @ingroup mbari
     */
    template<typename Ty>
    struct identity :public std::unary_function<Ty, Ty> {
      /** @brief Functor call
       * @param[in] x A value
       * @return @p x
       */
      Ty const &operator()(Ty const &x) const {
	return x;
      }
    }; // mbari::details::identity<>

  } // mbari::details

  /** @brief Time serie
   * @tparam Ty A value type
   * 
   * This class is used to maintain a spacially located time serie 
   * with the samples of type @p Ty. 
   *
   * Each sample is associated to a date -- used as the key -- and a 
   * 3-D location (northing, easting, depth). All the samples are
   * sorted accroding to their date 
   *
   * @author Frederic Py <fpy@mbari.org>
   * @ingroup tracker
   */
  template<typename Ty>
  class serie {
  public:
    /** @brief Date type
     * The type of the date which is also used as a unique key 
     * for the samples
     */
    typedef boost::posix_time::ptime                  key_type;
    /** @brief sample type
     *
     * The type used for the sample. This tyape is a pair with 
     * the @c first element being the sample location and the 
     * @c second being the sample value
     */
    typedef std::pair<point<3>, Ty>         sample_type;
    /** @brief container type
     *
     * The type used to store and map samples and their location 
     * to the sampling date
     */
    typedef std::map<key_type, sample_type> container_type;

    /** @brief iterator type
     *
     * The type used to iterator through the time serie
     * @sa begin() const
     * @sa end() const
     */
    typedef typename container_type::const_iterator iterator;

    /** @brief Constructor 
     *
     * Create an empty serie
     */
    serie() {}
    /** @brief Destructor */
    ~serie() {}

    /** @brief Number of sample
     *
     * @return the number of smaples
     */
    size_t size() const {
      return m_samples.size();
    }
    /** @brief Check if empty
     *
     * @retval true if the serie is empty
     * @retval false if the serie contains at least 1 sample
     */
    bool empty() const {
      return m_samples.empty();
    }
    /** @brief Start iterator
     * @return An iterator pointing to the first sample
     * @sa end() const
     */
    iterator begin() const {
      return m_samples.begin();
    }
    /** @brief End iterator
     * @return An iterator pointing to the end of the time serie
     * @sa begin() const
     */
    iterator end() const {
      return m_samples.end();
    }
    /** @brief Find lower bound
     * @param[in] key A date
     * 
     * Locate the first sample which was not taken before @p key
     *
     * @return An iterator with a date greater or equal to @p key or 
     * @c end() if no such element exists
     *
     * @sa end() const
     * @sa between(key_type, key_type) const
     */
    iterator lower_bound(key_type key) const {
      return m_samples.lower_bound(key);
    }
    /** @brief Add sample
     * @param[in] when  A date
     * @param[in] where A location
     * @param[in] what  A sample value
     *
     * Adds the sample @p what on location @p where at the date @p 
     * date. If a sample for @p date already did exists this 
     * operation will be ignored.
     *
     * @retval true The new sample has been added and the serie size 
     * has increased by 1
     * @retval false A sample for @p date already exists
     *
     * @post The serie is not empty
     *
     * @sa empty() const
     */
    bool add(key_type when, point<3> const &where, Ty const &what) {
      return m_samples.insert(std::make_pair(when,
					     std::make_pair(where, what))).second;
    }
    /** @brief clear samples
     *
     * Removes all the samples in this time serie
     *
     * @post The serie is empty
     * @sa clear(key_type)
     */
    void clear() {
      m_samples.clear();
    }
    
    /** @brief clear past samples
     * @param[in] date A date
     *  
     * Remove all samples anterior to @p date
     *
     * @post The serie  does not conatin any sample dating before @p date
     * @sa clear()
     */
    void clear(key_type date) {
      typename container_type::iterator 
         pos = m_samples.lower_bound(date);
      m_samples.erase(m_samples.begin(), pos);
    }
    /** @brief Correct sample position
     *
     * @param[in] from Initial time
     * @param[in] to End time
     * @param[in] fix (northing, easting) error rate in m/s
     *
     * Realign the position of all the sample in the time interval 
     * [@p from , @p to) by applying the linear error @p fix. for each 
     * samples with their @c date within this time interval and a 
     * position @c pos we correct the two first dimensions of  
     * @p pos as follow:
     * @code 
     * pos[i] += fix[i] * date - from
     * @endcode
     * This allow to back-propagate a position error identified 
     * later on by, for example, the AUV getting a new GPS fix 
     * after being underwater.
     */
    void align(key_type from, key_type to, point<2> const &fix) {
      point<3> fix3(fix);
      typename container_type::iterator i=m_samples.upper_bound(from),
	end_i = m_samples.lower_bound(to);
      for(; end_i!=i; ++i)
	i->second.first += fix3 * (i->first-from).seconds();
    }
     
    /** @brief Get sample interval
     *
     * @param[in] from Start date
     * @param[in] to End date
     *
     * @return A pair of iterators demarking the range of all the 
     * samples within the time interval [@p from, @p to]
     */
    std::pair<iterator, iterator> between(key_type from ,key_type to) const {
      return std::make_pair(m_samples.lower_bound(from),
			    m_samples.upper_bound(to));
    }
    /** @brief Travel distance cordinates
     *
     * @param[in] from An iterator
     * @param[to] to An iterator
     *
     * @pre @p from and @p to are valid iterator for this serie
     * @pre @p from is before @p to in the serie
     *
     * Project sample locations into the travel coordinates within 
     * the time interval referred by [@p from, @p to). Travel
     * coordinates are computing the travel distance between all the
     * samples in the (northing, easting) plane with the second
     * dimension being the depth of sample.
     *
     * This is usefull to interpolate or display samples within a
     * curtain plot with x being the travel distanbce of the vehicle 
     * and y being the depth. 
     * Such plot is often used in oceanography to show mobile assets
     * data.  
     *
     * @return the travel distance corrdinates of each samples in 
     *         [@p from, @p to)
     */
    std::vector< point<2> > travel_pts(iterator from, iterator to) const {
      std::vector< point<2> > result;
      if( to!=from ) {
	point<3> const *pred = &(from->second.first);
	point<2> pt;
	// First point is at (0, z)
	pt[1] = (*pred)[2];
	result.push_back(pt);
	
	for(++from; to!=from; ++from) {
	  point<2> tmp(*pred);
	  pred = &(from->second.first);
	  
	  tmp.substract(pred->begin(), pred->end());
	  pt[0] += tmp.norm_2(); // distance along track
	  pt[1] = (*pred)[2];    // depth
	  result.push_back(pt);
	}
      }
      return result;
    }

    /** @brief transform method
     *
     * @tparam Accessor A unary functor type
     * @param[in] from An iterator
     * @param[in] to An iterator
     * @param[in] fn A functor
     *
     * @pre Accessor::argument_type is compatibe with @p Ty
     * @pre @p from and @p to are valid iterator for this serie
     * @pre @p from precede @p to
     *
     * Applies @p fn to all the element of the serie within 
     * [@p from, @p to)
     *
     * @return The results of @p fn applied to all the elements of 
     * [@p from, @p to)
     *
     * @sa get(iterator, iterator) const
     */
    template<class Accessor>
    std::vector<typename Accessor::return_type> get(iterator from, iterator to, Accessor fn) const {
      std::vector<typename Accessor::return_type> res;
      for(; from!=to; ++from) 
	res.push_back(fn(from->second.second));
      return res;
    }
    
    /** @brief Get sample values
     * @param[in] from An iterator
     * @param[in] to An iterator
     * 
     * @pre @p from and @p to are valid iterator for this serie
     * @pre @p from precede @p to
     * 
     * Extract the sample data for all the elements within 
     * [@p from, @p to)
     *
     * @return The samples values extracted
     *
     * @sa template<Accessor> get(iterator, iterator, Accessor)
     * @sa mabri::details::identity
     */
     std::vector<Ty> get(iterator from, iterator to) const {
      typename details::identity<Ty> id;
      return get(from, to, id);
    }

    /** @brief Freshest sample date
     *
     * @pre the seroie is not empty
     * @return the date of the most recent sample
     * @sa empty() const
     */
    key_type newest() const {
      return m_samples.rbegin()->first;
    }

  private:
    container_type m_samples;

  }; // mbari::serie<>

} // mbari

#endif // H_mbari_serie
