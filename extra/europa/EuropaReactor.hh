/*********************************************************************
 * Software License Agreement (BSD License)
 * 
 *  Copyright (c) 2011, MBARI.
 *  All rights reserved.
 * 
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 * 
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the TREX Project nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 * 
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */
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
      void relax();
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
	  m_core.archive();
	  logPlan();
	}
      }

      void logPlan(std::string fname=std::string()) const {
	if( fname.empty() )
	  fname = "plan";
	std::string dbg_pln = manager().file_name(getName().str()+"."+fname+".dot");
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
