#ifndef H_DbCore 
# define H_DbCore

# include "Assembly.hh"

# include <PLASMA/PlanDatabaseListener.hh>
# include <PLASMA/Token.hh>

namespace TREX {
  namespace europa {
    
		/** @brief Core deliberation @& synchronization class
		 *
		 * This class embeds the core functionalities for both deliberation and 
		 * synchronization of the EuropaReactor.
		 * 
		 * @ingroup europa
		 * @author Frederic Py <fpy@mbari.org> lrgelly inspired by Connor McGann 
		 *         code on previous TREX versions 
		 */
    class DbCore :public EUROPA::PlanDatabaseListener {
    public:
			/** @brief Constructor
			 * 
			 * @param[in] owner A Europareactor
			 *
			 * Create e new instance associted to @p owner
			 */
      explicit DbCore(EuropaReactor &owner);
      ~DbCore();
			
			/** @brief Goal recall callback
			 *
			 * @param[in] g A token key
			 *
			 * Notifies the class that the token with the key @p g is no longer a 
			 * goal
			 *
			 * @post @p g is added to the recall queue
			 * 
			 * @sa EuropaReactor::handleRecall(TREX::transaction::goal_id const &)
			 * @sa process_recalls()
			 */
      void recall(EUROPA::eint const &g);
			/** @brief Recalls processing 
			 *
			 * Allow the class to process all the recalls received ansd waiting in the 
			 * recall queue.
			 * 
			 * All the tokens on the recall queue are removed from the planner goal 
			 * set for future deliberation
			 * 
			 * @retval true  There was valid tokens in the recall queue
			 * @retval false None of the keys of the recall queue were valid tokens 
			 *               in the plan
			 * @post the recall queue is empty
			 */
      bool process_recalls();
			/** @brief Initialize the plan
			 *
			 * This method complete the reactor initialization by inserting all the 
			 * model specified facts and activating them
			 * 
			 * @pre the facts are valid and do not aplly on an @e External timeline 
			 *      of this reactor
			 * @throw TokenError one of the goal specified in the model did apply to 
		   *        a timeline which is not owned by this reactor
			 * @retval true the initialization was sucessfull
			 * @retval false the intialization resulted in a inconsitent plan
			 * @sa apply_facts(std::list<EUROPA::TokenId> const &)
			 */
      bool initialize();
			/** @brief New observation notification
			 * 
			 * @param[in] obj A timeline
			 * @param[in] tok A token
			 * 
			 * This callback notfies the class that the token @p tok is a new 
			 * observation on the timeline @p obj
			 * 
			 * @pre @p obj is an @e External timeline
			 * @pre @p tok the object of @p tok is @p obj
			 * 
			 * @sa EuropaReactor::notify(TREX::transaction::Observation const &)
			 * @sa Assembly::isExternal(EUROPA::ObjectId const &) const
			 */
      void notify(EUROPA::ObjectId const &obj, EUROPA::TokenId const &tok);
			/** @brief Request for new observation
			 * 
			 * Asks this class to porduce new observations on its internal timlines 
			 * if any
			 */
      void doNotify();
			
      bool complete_externals();
      bool synchronize();
      void commit();
      bool propagate();
      void processPending();
      
      bool relax(bool discard);
			
      bool is_goal(EUROPA::TokenId const &tok) const {
				return m_goals.end()!=m_goals.find(tok);
      }
      bool is_observation(EUROPA::TokenId const &tok) const {
				return m_observations.end()!=m_observations.find(tok);
      }
      
      void set_internal(EUROPA::ObjectId const &obj);
			
    private:
      bool is_current(EUROPA::TokenId const &tok) const;
      void copy_value(EUROPA::TokenId const &tok);
			
      void reset_observations();
      void reset_goals(bool discard);
      void reset_other_tokens(bool discard);
			
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
      bool insert_copies();
			
      EuropaReactor &m_reactor;
      
      EUROPA::TokenSet m_pending;
      EUROPA::TokenSet m_agenda;
      
      EUROPA::TokenSet m_goals;
      EUROPA::TokenSet m_observations;
      EUROPA::TokenSet m_completed_obs;
      EUROPA::TokenSet m_commited;
      
      std::set<EUROPA::eint> m_recalls;
      
      typedef std::map<EUROPA::ObjectId, EUROPA::TokenId> current_state_map;      
      
      current_state_map m_external_obs;
      current_state_map m_internal_obs;
    }; // TREX::europa::DbCore
    
		
  } // TREX::europa
} // TREX 

#endif // H_DbCore
