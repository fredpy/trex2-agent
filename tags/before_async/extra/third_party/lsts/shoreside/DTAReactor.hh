#ifndef H_lsts_DTAReactor
# define H_lsts_DTAReactor

# include <trex/transaction/reactor.hh>

namespace TREX {
  namespace LSTS {
    

    class DTAReactor :public TREX::transaction::reactor {
    public:
      DTAReactor(TREX::transaction::reactor::xml_arg_type arg);
      ~DTAReactor();

    private:
      bool synchronize();
      void handle_request(TREX::transaction::token_id const &g);
      void notify(TREX::transaction::token const &obs);

      bool m_active;
      TREX::utils::symbol m_drifter;
      TREX::utils::symbol m_path;
      double              m_factor;
      bool                m_lagrangian;
      
      std::pair<double, double> m_pos, m_speed;
      bool m_have_pos;
      bool m_have_speed;
      
      TREX::utils::symbol m_proxy_timeline;
      TREX::utils::symbol m_asset_id;
      TREX::utils::symbol m_survey_tl, m_state_tl;

      enum vehicle_mode {
	UNKNOWN =0,
	WAITING,
	GOAL_SENT,
	RUNNING
      };

      vehicle_mode m_trex_state;      
    };

  }
}

#endif // H_mbari_DTAReactor 
