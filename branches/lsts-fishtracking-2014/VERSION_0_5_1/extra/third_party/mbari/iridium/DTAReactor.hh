#ifndef H_mbari_DTAReactor
# define H_mbari_DTAReactor

# include "SbdMailer.hh"
# include <trex/transaction/TeleoReactor.hh>

namespace mbari {
  namespace iridium {

    class DTAReactor :public TREX::transaction::TeleoReactor {
    public:
      DTAReactor(TREX::transaction::TeleoReactor::xml_arg_type arg);
      ~DTAReactor();

    private:
      bool synchronize();
      void handleRequest(TREX::transaction::goal_id const &g);
      void notify(TREX::transaction::Observation const &obs);
      
      TREX::utils::Symbol tl_name(TREX::utils::Symbol const &name) const {
        return m_drifter_pfx+name.str();
      }
      
      void update_state(TREX::transaction::TICK date);
      void set_state(TREX::transaction::Goal const &g) {
        TREX::transaction::Observation o(g);
        set_state(o, g.getEnd());
      }
      void set_state(TREX::transaction::Observation obs,
                     TREX::transaction::IntegerDomain const &end);
      void send_cmd();

      bool m_active;
      TREX::utils::Symbol m_drifter;
      TREX::utils::Symbol m_path;
      double              m_factor;
      bool                m_lagrangian;
      
      
      UNIQ_PTR<TREX::transaction::IntegerDomain> m_current_end;
      std::list<TREX::transaction::goal_id> m_pending_goals;
      
      std::pair<double, double> m_pos, m_speed, m_shift;
      bool m_have_pos;
      bool m_have_speed;
      // bool m_waiting;
      bool m_use_iridium;

      enum vehicle_mode {
	UNKNOWN =0,
	WAITING,
	GOAL_SENT,
	RUNNING
      };

      vehicle_mode m_trex_state;
      
      std::string    m_imei;
      SbdMailer      m_iridium;

      std::string  m_drifter_pfx;
    };

  }
}

#endif // H_mbari_DTAReactor 
