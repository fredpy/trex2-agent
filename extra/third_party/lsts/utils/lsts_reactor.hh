#ifndef H_TREX_dune_lsts_reactor
# define H_TREX_dune_lsts_reactor

# include <trex/transaction/TeleoReactor.hh>

namespace trex_lsts {
  
  class lsts_reactor :public TREX::transaction::TeleoReactor {
  public:
    lsts_reactor(xml_arg_type arg);
    virtual ~lsts_reactor();
    
  protected:
    bool is_fresh(TREX::transaction::Observation const &obs) const;
    bool post_unique(TREX::transaction::Observation const &obs, bool verbose=false);
    
  private:
    typedef std::map<TREX::utils::Symbol, SHARED_PTR<TREX::transaction::Observation> > obs_cache;
    obs_cache m_posted_obs;
  }; // class trex_lsts::lsts_reactor
  
} // trex_lsts

#endif // H_TREX_dune_lsts_reactor
