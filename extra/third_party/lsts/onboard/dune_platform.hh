#ifndef H_TREX_dune_dune_platform
# define H_TREX_dune_dune_platform

# include <DUNE/IMC/Definitions.hpp>

# include <boost/function.hpp>
# include <queue>

# include "../utils/lsts_reactor.hh"
# include "../utils/imc_adapter.hh"
# include "dune_env.hh"


namespace trex_lsts {
  
  class dune_platform :public lsts_reactor {
  public:
    dune_platform(xml_arg_type arg);
    ~dune_platform();
    
    static TREX::utils::Symbol const s_reference;
    static TREX::utils::Symbol const s_go;
    static TREX::utils::Symbol const s_at;
    static TREX::utils::Symbol const s_lat;
    static TREX::utils::Symbol const s_lon;

    
    
  private:
    // send message to Dune
    bool send(imc_adapter::message &m);
    
    
    
    // TREX callbacks
    void handleInit();
    void handleTickStart();
    void handleRequest(TREX::transaction::goal_id const &g);
    void handleRecall(TREX::transaction::goal_id const &g);
    bool synchronize();
    
    bool auv_go(TREX::transaction::goal_id g);
    bool uav_go(TREX::transaction::goal_id g);
    
    void process_goals();
    
    void on_estate(TREX::utils::Symbol const &tl, SHARED_PTR<DUNE::IMC::EstimatedState> state);
    void on_medium(TREX::utils::Symbol const &tl, SHARED_PTR<DUNE::IMC::VehicleMedium> medium);
    void on_control(TREX::utils::Symbol const &tl, SHARED_PTR<DUNE::IMC::PlanControl> control);
    void on_limits(TREX::utils::Symbol const &tl, SHARED_PTR<DUNE::IMC::OperationalLimits> limits);

    typedef std::multimap<TREX::utils::Symbol, imc_adapter::connection> conn_map;
    conn_map m_handlers;

    template<class Msg, class Fn>
    void imc_provide(TREX::utils::Symbol const &name, bool control, Fn handle, TREX::utils::Symbol const &boot) {
      conn_map::iterator pos = m_handlers.find(name);
      if( m_handlers.end()!=pos ) {
        provide(name, control);
        TREX::transaction::Observation o(name, boot);
        postObservation(o);
      }
      m_handlers.insert(conn_map::value_type(name, m_adapter->typed_connect<Msg>(boost::bind(handle, this, name, _1))));
    }

    boost::function<bool (TREX::transaction::goal_id)> m_go;
    TREX::transaction::TICK m_max_silent;
    boost::optional<TREX::transaction::TICK> m_last_message;
    
    bool        m_first_tick, m_blocked, m_connected;
    
    std::list<TREX::transaction::goal_id> m_pending;
    
    std::queue<TREX::transaction::Observation> m_expected_refs;

    TREX::utils::SingletonUse<imc_adapter> m_adapter;
    TREX::utils::SingletonUse<dune_env>    m_env;
  }; // class trex_lsts::dune_platform
  
  
} // trex_lsts

#endif // H_TREX_dune_dune_platform
