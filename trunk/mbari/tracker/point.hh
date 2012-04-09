#ifndef H_mbari_point
# define H_mbari_point

# include <boost/mpl/min_max.hpp>

# include <algorithm>
# include <cmath>

namespace mbari {

  template<unsigned Dim>
  class point {
  public:
    typedef double       *iterator;
    typedef double const *const_iterator;

    typedef boost::mpl::int_<Dim> dimension_;

    enum {
      dimension = dimension_::value
    };

    point() {
      std::fill(begin(), end(), 0.0);
    }
    template<class Iter>
    point(Iter from, Iter to) {
      unsigned i;
      for(i=0; to!=from && i<Dim; ++i, ++from) 
	m_value[i] = *to;
      std::fill(m_value+i, end(), 0.0);
    }

    template<unsigned D2>
    explicit point(point<D2> const &other) {
      unsigned i;
      for(i=0; i< boost::mpl::min< dimension_, typename point<D2>::dimension_ >::type::value; ++i)
	m_value[i] = other[i];
      std::fill(m_value+i, end(), 0.0);
    }
    ~point() {}

    size_t size() const {
      return dimension;
    }

    iterator begin() {
      return m_value;
    }
    iterator end() {
      return m_value+Dim;
    }

    const_iterator begin() const {
      return m_value;
    }
    const_iterator end() const {
      return m_value+Dim;
    }
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

    template<class Iter>
    point &add(Iter from, Iter to) {
      for(unsigned i=0; from!=to && i<Dim; ++i, ++from)
	m_value[i] += *from;
      return *this;
    }
    point &operator+=(point const &other) {
      return add(other.begin(), other.end());
    }
    point operator+ (point const &other) const {
      return point<Dim>(*this).operator+=(other);
    }

    template<class Iter>
    point &substract(Iter from, Iter to) {
      for(unsigned i=0; from!=to && i<Dim; ++i, ++from)
	m_value[i] -= *from;
      return *this;
    }
    point &operator-=(point const &other) {
      return substract(other.begin(), other.end());
    }
    point operator- (point const &other) const {
      return point<Dim>(*this).operator-=(other);
    }
    
    point &operator *=(double factor) {
      for(unsigned i=0; i<Dim; ++i)
	m_value[i] *= factor;
    }
    point operator* (double factor) const {
      return point<Dim>(*this).operator*=(factor);
    }
    point &operator /=(double factor) {
      for(unsigned i=0; i<Dim; ++i)
	m_value[i] /= factor;
    }
    point operator/ (double factor) const {
      return point<Dim>(*this).operator/=(factor);
    }

    double dot(point const &other) const {
      double ret = 0.0;
      for(unsigned i=0; i<Dim; ++i)
	ret += m_value[i]*other[i];
      return ret;
    }
    double norm_2() const {
      return sqrt(dot(*this));
    }
  private:
    double m_value[Dim];
  }; // mbari::point<>
  

} // mbari

#endif // H_mbari_point
