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

      bool complete_externals();
      void commit();
      bool propagate();
      void processPending();

      bool is_goal(EUROPA::TokenId const &tok) const {
	return m_goals.end()!=m_goals.find(tok);
      }
      bool is_observation(EUROPA::TokenId const &tok) const {
	return m_observations.end()!=m_observations.find(tok);
      }
      

    private:
      void commit_and_restrict(EUROPA::TokenId const &token);

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
      std::map<EUROPA::ObjectId, EUROPA::TokenId> m_last_obs;
    }; // TREX::europa::DbCore
    

  } // TREX::europa
} // TREX 

#endif // H_DbCore
