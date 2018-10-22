#ifndef H_TREX_dune_operational_limits
# define H_TREX_dune_operational_limits

# include <trex/utils/SingletonUse.hh>
# include <trex/utils/platform/memory.hh>
# include <trex/domain/FloatDomain.hh>


namespace trex_lsts {
  
  class op_limits :boost::noncopyable {
  public:
    typedef TREX::transaction::FloatDomain range;
    typedef range::base_type float_type;

    struct safe_area {
      float_type latitude, longitude;
      float_type half_width, half_length;
      float_type orientation;
    };
    
    
    bool is_set() const;
    
    range valid_depths() const;
    range valid_altitudes() const;
    range valid_speed() const;
    
    bool is_valid(float_type lat, float_type lon) const;
    
    
  private:
    class pimpl;
    
    static range const s_positives;
    
    boost::optional<safe_area> get_area() const;
    
    UNIQ_PTR<pimpl> m_impl;
    mutable UNIQ_PTR<range> m_depths, m_altitudes, m_speeds;
    mutable boost::optional<safe_area> m_area;
    
    void refresh() const;
    
    op_limits();
    ~op_limits();
    
    
    
    
    friend TREX::utils::SingletonWrapper<op_limits>;
  }; // class trex_lsts::op_limits
  
}

#endif // H_TREX_dune_operational_limits
