/** @file extra/europa/EuropaReactor.hh
 * @brief Definition of an europa based reactor
 * 
 * This files defines the europa based deliberative reactor
 * 
 * @author Frederic Py <fpy@mbari.org>
 * @ingroup europa
 */
#ifndef H_EuropaReactor 
# define H_EuropaReactor 

# include "DbCore.hh"
# include "DeliberationFilter.hh"

# include <trex/transaction/TeleoReactor.hh>

namespace TREX {
  namespace europa {
    
    /** @brief Europa based deliberative reactor
     * 
     * This reactor is a reactor that is able to synchronize and deliberate 
     * based on the europa-pso planning engine.
     * It loads a nddl model and uses this model to deduces its internal 
     * timelines values and post goals to its external timelines as execution 
     * frontier advances.
     * 
     * Th EuropaReactor class itself is mostly a proxy that interfaces trex 
     * callbacks with europa calls. and most of the implementation is on the 
     * DbCore and Assembly classes
     * 
     * @author Frederic Py <fpy@mbari.org>
     * @ingroup europa
     * @sa DbCore
     * @sa Assembly
     */
    class EuropaReactor :public TREX::transaction::TeleoReactor {
    public:
      EuropaReactor(TREX::transaction::TeleoReactor::xml_arg_type arg);
      ~EuropaReactor();

      // EUROPA callbacks
      void removed(EUROPA::TokenId const &tok);
      void request(EUROPA::ObjectId const &tl, EUROPA::TokenId const &tok);
      void recall(EUROPA::TokenId const &tok);
      void notify(EUROPA::ObjectId const &tl, EUROPA::TokenId const &tok);

      bool in_scope(EUROPA::TokenId const &tok);

      bool dispatch_window(EUROPA::ObjectId const &obj, 
			   TREX::transaction::TICK &from, 
			   TREX::transaction::TICK &to);

      Assembly &assembly() {
        return m_assembly;
      }

      TREX::utils::internals::LogEntry 
      log(std::string const &context) {
        return syslog(context);
      }

      void end_deliberation() {
        m_assembly.mark_inactive();
        m_completedThisTick = true;
	if( m_steps>1 ) {
	  syslog()<<"Deliberation completed in "<<m_steps<<" steps";
	  logPlan();
	}
      }

      void logPlan() const {
	  std::string dbg_pln = manager().file_name(getName().str()+".plan.dot");
	  std::ofstream of(dbg_pln.c_str());
	  m_assembly.logPlan(of);
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
      DbCore   m_core;

      void reset_deliberation() {
        m_planStart = getInitialTick();
        m_steps = 0;
      }
      bool deactivate_solver();
      void set_filter(DeliberationFilter *filt) {
	m_filter = filt;
      }
      
      TREX::transaction::TICK m_planStart;
      bool                    m_completedThisTick;
      unsigned long           m_steps;

      DeliberationFilter *m_filter;

      friend class Assembly;
      friend class DeliberationFilter;
    }; //TREX::europa::EuropaReactor

  } // TREX::europa
} // TREX

#endif // H_EuropaReactor
