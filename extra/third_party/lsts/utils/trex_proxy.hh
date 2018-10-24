#ifndef H_TREX_dune_trex_proxy
# define H_TREX_dune_trex_proxy

# include <trex/transaction/TeleoReactor.hh>
# include <trex/domain/IntegerDomain.hh>

namespace trex_lsts {
  
  class trex_proxy {
    typedef TREX::transaction::graph trex_graph;
  public:
    typedef trex_graph::date_type     date_type;
    typedef trex_graph::duration_type duration_type;
    typedef TREX::transaction::TICK   tick_type;
    
    virtual ~trex_proxy();
    
    virtual tick_type current_tick() const =0;
    virtual date_type tick_to_date(tick_type) const =0;
    virtual tick_type date_to_tick(date_type const &) const =0;
    
    virtual std::string date_str(tick_type) const =0;
    virtual std::string duration_str(tick_type) const =0;
    std::string date_str(TREX::transaction::IntegerDomain::bound const &b) const;
    std::string duration_str(TREX::transaction::IntegerDomain::bound const &b) const;

    virtual tick_type as_date(std::string const &) const =0;
    virtual tick_type as_duration(std::string const &) const =0;
    
    virtual boost::asio::io_service &service() =0;
    virtual TREX::utils::log::stream log(TREX::utils::Symbol const &type) =0;
    
  protected:
    trex_proxy();
    
    
  }; // class trex_lsts::trex_proxy
  
} // trex_lsts

#endif // H_TREX_dune_trex_proxy
