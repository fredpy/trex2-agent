#ifndef H_mbari_location
# define H_mbari_location

# include "point.hh"
# include <sys/time.h>

namespace mbari {

  class location {
  public:
    location():m_valid(false), m_have_speed(false) {}

    bool is_valid() const {
      return m_valid;
    }
    bool have_speed() const {
      return m_have_speed;
    }
    void update(time_t date, double north, double east);
    point<2> const &speed() const {
      return m_speed;
    }
    point<2> position(time_t now, long int &delta) const; 

  private:
    time_t   m_date;
    bool     m_valid, m_have_speed;
    point<2> m_last_pos, m_speed;
  }; 

} // mbari

#endif 
