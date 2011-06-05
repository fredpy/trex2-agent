#ifndef H_DbCore 
# define H_DbCore

# include "Assembly.hh"

# include <PLASMA/PlanDatabaseListener.hh>
# include <PLASMA/Token.hh>

namespace TREX {
  namespace europa {
    
    class DbCore :public EUROPA::PlanDatabaseListener {
      typedef std::map<EUROPA::ObjectId, EUROPA::TokenId> state_map;

    public:
      explicit DbCore(EuropaReactor &owner);
      ~DbCore();
      
      void set_internal(EUROPA::ObjectId const &obj);

      void notify(state_map::value_type const &obs);
      bool update_externals();
      bool synchronize();
      void doNotify();
      
      bool relax(bool aggressive);
      
    private:
      bool is_current(EUROPA::TokenId const &tok) const;
      bool is_goal(EUROPA::TokenId const &tok) const {
        return tok->master().isNoId() && m_goals.end()!=m_goals.find(tok); 
      }
      
      bool propagate();
      void process_pendings();
      bool update_internals(size_t &steps);
      
      void reset_observations();
      bool reset_goal(EUROPA::TokenId const &tok, bool discard_current);
      
      void add_to_agenda(EUROPA::TokenId const &tok);
      bool resolve(size_t &steps);
      void remove_from_agenda(EUROPA::TokenId const &tok);
      
      
      // europa callbacks
      void notifyAdded(EUROPA::TokenId const &token);
      void notifyRemoved(EUROPA::TokenId const &Token);
      
      void notifyActivated(EUROPA::TokenId const &token);
      void notifyDeactivated(EUROPA::TokenId const &token);
      
      void notifyMerged(EUROPA::TokenId const &token);
      void notifySplit(EUROPA::TokenId const &token);
      
      void notifyCommitted(EUROPA::TokenId const &token);
      void notifyRejected(EUROPA::TokenId const &token);
      void notifyTerminated(EUROPA::TokenId const &token);
            
      EuropaReactor &m_reactor;

      state_map m_externals;
      state_map m_internals;

      EUROPA::TokenSet m_pending;
      EUROPA::TokenSet m_goals;
      EUROPA::TokenSet m_agenda;
      EUROPA::TokenSet m_observations;
      EUROPA::TokenSet m_completed_obs;
    }; // TREX::europa::DbCore
    
    
  } // TREX::europa
} // TREX 

#endif // H_DbCore
