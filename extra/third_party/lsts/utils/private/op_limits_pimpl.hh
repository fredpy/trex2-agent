#ifndef H_TREX_dune_private_op_limits_pimpl
# define H_TREX_dune_private_op_limits_pimpl

# include "../operational_limits.hh"
# include <boost/thread/mutex.hpp>
# include <boost/atomic.hpp>


namespace DUNE {
  namespace IMC {
    class OperationalLimits;
  }
}

namespace trex_lsts {
  
  class op_limits::pimpl {
  public:
    typedef boost::mutex                  mutex_type;
    typedef boost::lock_guard<mutex_type> lock_guard;

    pimpl();
    ~pimpl();
    
    void set_limits(DUNE::IMC::OperationalLimits const &lim);
    bool is_set() const;
    
    bool is_fresh() const;
    
    range valid_depths() const;
    range valid_altitudes() const;
    range valid_speeds() const;
    
    boost::optional<safe_area> valid_area() const;
    
    
    mutex_type &mtx() const {
      return m_mutex;
    }
    
    
  private:
    mutable mutex_type m_mutex;
    mutable boost::atomic_bool m_changed;
    UNIQ_PTR<DUNE::IMC::OperationalLimits> m_limits;
  };
  
} // trex_lsts

#endif // H_TREX_dune_private_op_limits_pimpl

