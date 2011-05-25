#ifndef H_EuropaReactor 
# define H_EuropaReactor 

# include "Assembly.hh"

# include <trex/transaction/TeleoReactor.hh>

namespace TREX {
  namespace europa {
    
    class EuropaReactor :public TREX::transaction::TeleoReactor {
    public:
      EuropaReactor(TREX::transaction::TeleoReactor::xml_arg_type arg);
      ~EuropaReactor();

      // EUROPA callbacks
      void removed(EUROPA::TokenId const &tok);
      void request(TREX::utils::Symbol const &tl, EUROPA::TokenId const &tok);
      void recall(EUROPA::TokenId const &tok);
      void notify(TREX::utils::Symbol const &tl, EUROPA::TokenId const &tok);

      Assembly &assembly() {
	return m_assembly;
      }

    private:
      // TREX transaction callbacks
      void notify(TREX::transaction::Observation const &o);
      void handleRequest(TREX::transaction::goal_id const &g);
      void handleRecall(TREX::transaction::goal_id const &g);

      // TREX execution callbacks
      void handleInit();
      void handleTickStart();
      bool synchronize();
      bool hasWork();
      void resume();
      
      typedef std::map<EUROPA::eint, TREX::transaction::goal_id> europa_mapping;

      europa_mapping m_external_goals;
      europa_mapping m_internal_goals;

      Assembly m_assembly;

      void reset_deliberation() {
	m_planStart = getInitialTick();
	m_steps = 0;
      }

      TREX::transaction::TICK m_planStart;
      unsigned long           m_steps;
      
      friend class Assembly;
    }; //TREX::europa::EuropaReactor

  } // TREX::europa
} // TREX

#endif // H_EuropaReactor
