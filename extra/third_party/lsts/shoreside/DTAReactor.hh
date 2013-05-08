#ifndef H_lsts_DTAReactor
# define H_lsts_DTAReactor

# include <trex/transaction/TeleoReactor.hh>

namespace TREX {
  namespace LSTS {
    

    class DTAReactor :public TREX::transaction::TeleoReactor {
    public:
      DTAReactor(TREX::transaction::TeleoReactor::xml_arg_type arg);
      ~DTAReactor();

    private:
      bool synchronize();
      void handleRequest(TREX::transaction::goal_id const &g);
      void notify(TREX::transaction::Observation const &obs);

      bool m_active;
      TREX::utils::Symbol m_drifter;
      TREX::utils::Symbol m_path;
      double              m_factor;
      bool                m_lagrangian;
      
      std::pair<double, double> m_pos, m_speed;
      bool m_have_pos;
      bool m_have_speed;

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
