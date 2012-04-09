#ifndef H_mbari_serie
# define H_mbari_serie

# include <map>
# include <vector>
# include <functional>

# include <sys/time.h>

# include "point.hh"


namespace mbari {

  namespace details {
    
    template<typename Ty>
    struct identity :public std::unary_function<Ty, Ty> {
      Ty const &operator()(Ty const &x) const {
	return x;
      }
    }; // mbari::details::identity<>

  } // mbari::details

  template<typename Ty>
  class serie {
  public:
    typedef time_t                          key_type;
    typedef std::pair<point<3>, Ty>         sample_type;
    typedef std::map<key_type, sample_type> container_type;

    typedef typename container_type::const_iterator iterator;

    serie() {}
    ~serie() {}

    size_t size() const {
      return m_samples.size();
    }
    bool empty() const {
      return m_samples.empty();
    }
    iterator begin() const {
      return m_samples.begin();
    }
    iterator end() const {
      return m_samples.end();
    }

    iterator lower_bound(key_type const &key) const {
      return m_samples.lower_bound(key);
    }
    bool add(time_t when, point<3> const &where, Ty const &what) {
      return m_samples.insert(std::make_pair(when,
					     std::make_pair(where, what))).second;
    }
    void clear() {
      m_samples.clear();
    }
    void clear(time_t date) {
      typename container_type::iterator pos = m_samples.lower_bound(date);
      m_samples.erase(m_samples.begin(), pos);
    }
    void align(time_t from, time_t to, point<2> const &fix) {
      point<3> fix3(fix);
      typename container_type::iterator i=m_samples.upper_bound(from),
	end_i = m_samples.lower_bound(to);
      for(; end_i!=i; ++i)
	i->second.first += fix3 * (i->first-from);
    }
     
    std::pair<iterator, iterator> between(time_t from ,time_t to) const {
      return std::make_pair(m_samples.lower_bound(from),
			    m_samples.upper_bound(to));
    }
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

    template<class Accessor>
    std::vector<typename Accessor::return_type> get(iterator from, iterator to, Accessor fn) const {
      std::vector<typename Accessor::return_type> res;
      for(; from!=to; ++from) 
	res.push_back(fn(from->second.second));
      return res;
    }

    std::vector<Ty> get(iterator from, iterator to) const {
      typename details::identity<Ty> id;
      return get(from, to, id);
    }

    key_type newest() const {
      return m_samples.rbegin()->first;
    }

  private:
    container_type m_samples;

  }; // mbari::serie<>

} // mbari

#endif // H_mbari_serie
