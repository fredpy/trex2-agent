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
#ifndef H_mbari_point
# define H_mbari_point

# include <boost/mpl/min_max.hpp>

# include <algorithm>
# include <cmath>

namespace mbari {

  /** @brief Fixed dimensions point
   *
   * @tparam Dim Dimensions for the point
   *
   * A class that represent a point in @p Dim dimensions space
   *
   * @ingroup tracker
   * @author Frederic Py <fpy@mbari.org>
   */
  template<unsigned Dim>
  class point {
  public:
    /** @brief dimensions iterator
     *
     * The type used to iterate through the different dimensions of the point
     *
     * @sa begin()
     * @sa end()
     * @sa const_iterator
     */
    typedef double       *iterator;
    /** @brief dimensions iterator
     *
     * The type used to iterate through the different dimensions of the point 
     * without mofification. 
     *
     * @sa begin() const
     * @sa end() const
     * @sa iterator
     */
    typedef double const *const_iterator;

    /** @brief Point dimensions
     * 
     * The number of dimensions for this point represented into meta 
     * programming.  
     *
     * Meta programming is usefull to do compilation-time computation 
     * avoiding in turn to do sthis computation ion running time.
     *
     * @sa dimension
     * @sa size() const
     */ 
    typedef boost::mpl::int_<Dim> dimension_;
        
    enum {
      /** @brief Point dimensions
       *
       * The number of dimensions for this point
       *
       * @sa size() const
       * @sa dimension_
       */
      dimension = dimension_::value
    };

    /** @brief Default constructor
     *
     * Create a new point. With all its dimensions set to @c 0.0
     */
    point() {
      std::fill(begin(), end(), 0.0);
    }
    /** @brief Constructor
     * @tparam Iter an iterator type
     * @param[in] from beginning iterator
     * @param[in] to end iterator
     *
     * Create a new instance and assign its dimensions based on the values
     * pointed by [@p from, @p to).
     *
     * If the the interval @p from @p to contains less than @p Dim values, 
     * the remaining dimensions will be set to @c 0.0.
     *
     * If the the interval @p from @p to contains less than @p Dim values, the 
     * remaining iterators will be ignored
     *
     * @pre @p Iter refers to a type that cna be implicitely converted into a 
     * @c double
     */
    template<class Iter>
    point(Iter from, Iter to) {
      unsigned i;
      for(i=0; to!=from && i<Dim; ++i, ++from) 
	m_value[i] = *to;
      std::fill(m_value+i, end(), 0.0);
    }

    /** @brief Copy constructor
     * @tparam D2 dimensions of the input point
     * @param[in] other the input point
     *
     * Create a new point which is a "copy" of @p other.
     *
     * If @p D2 is less than @p Dim the remaining diemensions are set to @c 0.0
     *
     * If @p D2 is larger than @p Dim the extra dimensions of @p other are ignored
     */
    template<unsigned D2>
    explicit point(point<D2> const &other) {
      unsigned i;
      for(i=0; i< boost::mpl::min< dimension_, typename point<D2>::dimension_ >::type::value; ++i)
	m_value[i] = other[i];
      std::fill(m_value+i, end(), 0.0);
    }
    /** @brief Destructor */
    ~point() {}

    /** @brief Point dimensions
     *
     * @return The number of dimensions for this point
     *
     * @sa dimension
     */
    size_t size() const {
      return dimension;
    }

    /** @brief Start iterator
     *
     * @return an iterator referring to the first dimension of the point
     * @sa end()
     * @sa begin() const
     * @sa end() const
     */
    iterator begin() {
      return m_value;
    }
    /** @brief End iterator
     *
     * @return an iterator referring to after the last dimension of this point
     * @sa begin()
     * @sa begin() const
     * @sa end() const
     */
    iterator end() {
      return m_value+Dim;
    }

    /** @brief Start iterator
     *
     * @return an iterator referring to the first dimension of the point
     * @sa end() const
     * @sa begin()
     * @sa end()
     */
    const_iterator begin() const {
      return m_value;
    }
    /** @brief End iterator
     *
     * @return an iterator referring to after the last dimension of this point
     * @sa begin() const
     * @sa begin()
     * @sa end()
     */
    const_iterator end() const {
      return m_value+Dim;
    }
    /** @brief Dimension access
     *
     * @param[in] idx A dimension index
     *
     * @pre @c 0 < @p idx < @p Dim
     *
     * @return A reference to the value for the dimensions @p idx
     *
     * @sa size() const
     * @{
     */
    double &operator[](size_t idx) {
      // if( Dim<=idx ) 
      // 	throw dimension_error("Index larger than point dimensions");
      return m_value[idx];
    }
    double const &operator[](size_t idx) const {
      // if( Dim<=idx ) 
      // 	throw dimension_error("Index larger than point dimensions");
      return m_value[idx];
    }
    /** @} */

    /** @brief Addition
     *
     * @tparam Iter an iterator type
     * @param[in] from beginning iterator
     * @param[in] to end iterator
     *
     * Add the vector [ @p from, @p to ) to this point.
     * This operation is equivalent to: 
     * @code 
     * *this += point<Dim>(from, to);
     * @endcode
     *
     * @return the point after the operation
     * @sa substract(Iter, Iter)
     */
    template<class Iter>
    point &add(Iter from, Iter to) {
      for(unsigned i=0; from!=to && i<Dim; ++i, ++from)
	m_value[i] += *from;
      return *this;
    }
    /** @brief Addition operator
     * @param[in] other A point
     * 
     * Adds @p other to this point 
     * 
     * @return the point after the addition operation
     *
     * @sa add(Iter, Iter)
     * @sa operator+(point const &) const
     */
    point &operator+=(point const &other) {
      return add(other.begin(), other.end());
    }
    /** @brief Addition operator
     * @param[in] other A point
     * 
     * compute the sum of this point with @p other
     * 
     * @return the sum of the two points
     *
     * @sa operator+=(point const &)
     */
    point operator+ (point const &other) const {
      return point<Dim>(*this).operator+=(other);
    }

    /** @brief Substraction
     *
     * @tparam Iter an iterator type
     * @param[in] from beginning iterator
     * @param[in] to end iterator
     *
     * Substract the vector [ @p from, @p to ) from this point.
     * This operation is equivalent to: 
     * @code 
     * *this -= point<Dim>(from, to);
     * @endcode
     *
     * @return the point after the operation
     * @sa add(Iter, Iter)
     */
    template<class Iter>
    point &substract(Iter from, Iter to) {
      for(unsigned i=0; from!=to && i<Dim; ++i, ++from)
	m_value[i] -= *from;
      return *this;
    }
    /** @brief Substraction operator
     * @param[in] other A point
     * 
     * Substract @p other from this point 
     * 
     * @return the point after the substraction operation
     *
     * @sa substract(Iter, Iter)
     * @sa operator-(point const &) const
     */
    point &operator-=(point const &other) {
      return substract(other.begin(), other.end());
    }
    /** @brief Difference operator
     * @param[in] other A point
     * 
     * compute the difference between this point and @p other
     * 
     * @return the difference between the two points
     *
     * @sa operator-=(point const &)
     */
    point operator- (point const &other) const {
      return point<Dim>(*this).operator-=(other);
    }
    
    /** @brief Scalar multiplication
     *
     * @param[in] factor A multiplicative factor
     *
     * Multiplies all the dimension of this instance by @p factor
     *
     * @return The point after the operation
     *
     * @sa operator*(double) const
     * @sa operator/=(double)
     */
    point &operator *=(double factor) {
      for(unsigned i=0; i<Dim; ++i)
	m_value[i] *= factor;
      return *this;
    }
    /** @brief Scalar multiplication
     *
     * @param[in] factor A multiplicative factor
     *
     * Copute the multiplication of this point by @p factor
     *
     * @return The result of the multiplication
     *
     * @sa operator*=(double)
     * @sa operator/(double) const
     */
    point operator* (double factor) const {
      return point<Dim>(*this).operator*=(factor);
    }
    /** @brief Scalar division
     *
     * @param[in] factor A divisor
     *
     * Divide all the dimension of this instance by @p factor
     *
     * @pre @p factor is not @c 0.0
     *
     * @return The point after the operation
     *
     * @sa operator/(double) const
     * @sa operator*=(double)
     */
    point &operator /=(double factor) {
      for(unsigned i=0; i<Dim; ++i)
	m_value[i] /= factor;
      return *this;
    }
    /** @brief Scalar multiplication
     *
     * @param[in] factor A divisor
     *
     * Compute the division of this point by @p factor
     *
     * @pre @p factor is not @c 0.0
     *
     * @return The result of the division
     *
     * @sa operator/=(double)
     * @sa operator*(double) const
     */
    point operator/ (double factor) const {
      return point<Dim>(*this).operator/=(factor);
    }

    /** @brief dot product
     *
     * @param[in] other Another point
     *
     * Compute the dot (or scalar) product between this point And @p other
     *
     * @return The result of the dot product with @p other
     */
    double dot(point const &other) const {
      double ret = 0.0;
      for(unsigned i=0; i<Dim; ++i)
	ret += m_value[i]*other[i];
      return ret;
    }
    /** @brief norm-2 of the point
     *
     * Compute the norm-2 of the point. The norm-2 of a point is the 
     * square root of the dot produc of this point with itslef. 
     * 
     * This is also the typical way to compute the length of a vector 
     * in an Euclidian space.
     *
     * @return The norm-2 of this point
     * @sa dot(point const &) const
     */
    double norm_2() const {
      return sqrt(dot(*this));
    }
  private:
    double m_value[Dim];
  }; // mbari::point<>
  

} // mbari

#endif // H_mbari_point
