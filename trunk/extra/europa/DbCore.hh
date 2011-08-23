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

      void initialize(EUROPA::ConstrainedVariableId const &clk);
      // public reactor interfaces
      // - internal timelines
      void set_internal(EUROPA::ObjectId const &obj);
      void doNotify();
      void recall(EUROPA::eint const &key);

      // - external timelines
      void set_external(EUROPA::ObjectId const &obj);
      void notify(state_map::value_type const &obs);
      void doDispatch();      

      // synchronization 
      bool synchronize();
      bool relax(bool aggressive = false);
      void archive();

      // deliberation
      bool step();
      
    private:
      typedef std::list<EUROPA::TokenId>    sequence_type;
      typedef sequence_type::const_iterator seq_iter;
      static std::pair<seq_iter, seq_iter> find_after(EUROPA::TimelineId const &tl,
						      TREX::transaction::TICK now);
      void enforce_duration(EUROPA::TokenId const &tok);
      void force_update(EUROPA::TokenId const &active, EUROPA::TokenId const &merged);

      bool update_externals();

      bool propagate();
      bool resolve(size_t &steps);
      bool update_internals(size_t &steps);

      bool process_agenda(EUROPA::TokenSet agenda, size_t &steps);

      // connection info to TREX
      EuropaReactor &m_reactor;

      state_map m_internals;
      state_map m_externals;
      state_map m_new_obs;

      bool is_goal(EUROPA::TokenId const &tok) const {
	return m_goals.end()!=m_goals.find(tok);
      }
      bool remove_goal(EUROPA::TokenId const &tok, bool restrict=true);
      bool reset_goal(EUROPA::TokenId const &tok, bool aggressive);

      EUROPA::TokenSet m_goals;
      EUROPA::TokenSet m_recalled;
      EUROPA::TokenSet m_observations;

      void remove_observation(EUROPA::TokenId const &tok);
      
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
      EUROPA::TokenSet m_terminated;

      EUROPA::ConstrainedVariableId m_clk;
    }; 


  }
}

#endif 
