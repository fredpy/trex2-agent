#ifndef H_DbCore
# define H_DbCore

# include "Assembly.hh"

# include <trex/transaction/Tick.hh>

# include <PLASMA/PlanDatabaseListener.hh>
# include <PLASMA/Token.hh>

namespace TREX {
  namespace europa {

    class DbCore :public EUROPA::PlanDatabaseListener {
      typedef std::map<EUROPA::ObjectId, EUROPA::TokenId> state_map;
    public:
      explicit DbCore(EuropaReactor &owner);
      ~DbCore();

      void initialize();
      // public reactor interfaces
      // - internal timelines
      void set_internal(EUROPA::ObjectId const &obj);
      void doNotify();
      // - external timelines
      void notify(state_map::value_type const &obs);
      bool update_externals();
      void doDispatch();

      // synchronization 
      bool synchronize();
      bool relax(bool aggressive = false);

      // deliberation
      void step();
      
    private:
      typedef std::list<EUROPA::TokenId>    sequence_type;
      typedef sequence_type::const_iterator seq_iter;
      static std::pair<seq_iter, seq_iter> find_after(EUROPA::TimelineId const &tl,
						      TREX::transaction::TICK now);

      static EUROPA::TokenId find_current(EUROPA::TimelineId const &tl, 
					  TREX::transaction::TICK now);
      bool propagate();
      bool resolve(size_t &steps);
      bool update_internals(size_t &steps);

      bool process_agenda(EUROPA::TokenSet agenda, size_t &steps);

      // connection info to TREX
      EuropaReactor &m_reactor;

      state_map m_internals;
      state_map m_externals;

      bool is_goal(EUROPA::TokenId const &tok) const {
	return m_goals.end()!=m_goals.find(tok);
      }
      bool reset_goal(EUROPA::TokenId const &tok, bool aggressive);

      EUROPA::TokenSet m_goals;
      EUROPA::TokenSet m_completed_obs;
      
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

      void process_pending();

      EUROPA::TokenSet m_pending;

      void add_to_agenda(EUROPA::TokenId const &token);
      void remove_from_agenda(EUROPA::TokenId const &token);

      EUROPA::TokenSet m_facts_agenda;      
      EUROPA::TokenSet m_agenda;
    }; 


  }
}

#endif 
