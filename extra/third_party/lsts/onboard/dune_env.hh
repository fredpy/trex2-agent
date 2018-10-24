#ifndef H_TREX_dune_dune_env
# define H_TREX_dune_dune_env

# include <trex/utils/SingletonUse.hh>

namespace trex_lsts {
  
  class dune_platform;
  class control_interface;
  
  
  class dune_env {
  public:
    bool platform_set() const;
    dune_platform &platform() const;
    
    
  private:
    dune_env();
    ~dune_env();
    
    dune_platform     *m_platform;
    control_interface *m_ctrl;
    
    void reset_platform(dune_platform *me=NULL);
    
    
    friend class TREX::utils::SingletonWrapper<dune_env>;
    friend class dune_platform;
  }; // class trex_lsts::dune_env
  
  
} // trex_lsts

#endif // H_TREX_dune_dune_env
