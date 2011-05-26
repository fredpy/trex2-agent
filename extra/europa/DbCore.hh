#ifndef H_DbCore 
# define H_DbCore

# include "Assembly.hh"

# include <PLASMA/PlanDatabaseListener.hh>
# include <PLASMA/Token.hh>

namespace TREX {
  namespace europa {
    
    class DbCore :public EUROPA::PlanDatabaseListener {
    public:
      explicit DbCore(EuropaReactor &owner);
      ~DbCore();

      bool initialize();
      void notify(EUROPA::ObjectId const &obj, EUROPA::TokenId const &tok);
      void doNotify();

      bool complete_externals();
      bool synchronize();
      void commit();
      bool propagate();
      void processPending();

      bool is_goal(EUROPA::TokenId const &tok) const {
	return m_goals.end()!=m_goals.find(tok);
      }
      bool is_observation(EUROPA::TokenId const &tok) const {
	return m_observations.end()!=m_observations.find(tok);
      }
      
      void set_internal(EUROPA::ObjectId const &obj);

    private:
      void commit_and_restrict(EUROPA::TokenId const &token);
      bool merge_token(EUROPA::TokenId const &tok, EUROPA::TokenId const &cand);
      bool insert_token(EUROPA::TokenId const &tok, size_t &steps);
      bool resolve_token(EUROPA::TokenId const &tok, size_t &steps, 
			 EUROPA::TokenId const &cand);
      bool resolve_tokens(size_t &steps);
      bool insert_default(EUROPA::ObjectId const &obj, EUROPA::TokenId &res, 
			  size_t &steps);
      bool complete_internals(size_t &steps);

      bool in_tick_horizon(EUROPA::TokenId const &tok) const;
      bool in_scope(EUROPA::TokenId const &tok) const;
      bool is_unit(EUROPA::TokenId const &tok, EUROPA::TokenId &cand);
      bool in_synch_scope(EUROPA::TokenId const &tok, EUROPA::TokenId &cand);

      // europa callbacks
      void notifyAdded(EUROPA::TokenId const &token);
      void notfiyRemoved(EUROPA::TokenId const &Token);

      void notifyActivated(EUROPA::TokenId const &token);
      void notifyDeactivated(EUROPA::TokenId const &token);

      void notifyMerged(EUROPA::TokenId const &token);
      void notifySplit(EUROPA::TokenId const &token);

      void notifyCommitted(EUROPA::TokenId const &token);
      void notifyRejected(EUROPA::TokenId const &token);
      void notifyTerminated(EUROPA::TokenId const &token);

      void ignore_token(EUROPA::TokenId const &token);

      void add_to_agenda(EUROPA::TokenId const &token);
      void remove_from_agenda(EUROPA::TokenId const &token);

      void apply_facts(std::list<EUROPA::TokenId> const &facts);

      EuropaReactor &m_reactor;
      
      EUROPA::TokenSet m_pending;
      EUROPA::TokenSet m_agenda;
      
      EUROPA::TokenSet m_goals;
      EUROPA::TokenSet m_observations;
      EUROPA::TokenSet m_completed_obs;
      
      typedef std::map<EUROPA::ObjectId, EUROPA::TokenId> current_state_map;
      
      current_state_map m_external_obs;
      current_state_map m_internal_obs;
    }; // TREX::europa::DbCore
    

  } // TREX::europa
} // TREX 

#endif // H_DbCore
