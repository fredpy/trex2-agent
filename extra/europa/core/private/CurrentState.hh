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
#ifndef H_trex_europa_CurrentState 
# define H_trex_europa_CurrentState

# include <bitset>

# include <trex/europa/config.hh>

# include <PLASMA/LabelStr.hh>
# include <PLASMA/Entity.hh>
# include <PLASMA/Timeline.hh>
# include <PLASMA/Token.hh>
# include <PLASMA/Solver.hh>

namespace TREX {
  namespace europa {

    class Assembly;

    namespace details {
      using EUROPA::LabelStr; // Needed in order to use DECLARE_ENTITY_TYPE

      class CurrentState :public EUROPA::Entity {
      public:
	DECLARE_ENTITY_TYPE(CurrentState);
	
	typedef EUROPA::Id<CurrentState> id_type;

	~CurrentState() {}

	id_type const &getId() const {
	  return m_id;
	}

	EUROPA::TimelineId const &timeline() const {
	  return m_timeline;
	}
	EUROPA::eint now() const;
	bool identified() const;
	bool committed() const;

	EUROPA::TokenId current() const {
	  return m_last_obs;
	}
	std::set<LabelStr> const &predicates() const {
	  return m_pred_names;
	}
	bool has_default() const;
	LabelStr default_pred() const {
	  return m_pred_default;
	}

	void commit();
	bool internal() const;
	bool external() const;

        void do_dispatch(EUROPA::eint lb, EUROPA::eint ub);
        
	class DecisionPoint :public EUROPA::SOLVERS::DecisionPoint {
	public:
	  DecisionPoint(EUROPA::DbClientId const &client, 
			EUROPA::Id<CurrentState> const &timeline,
			EUROPA::TiXmlElement const &config, 
			EUROPA::LabelStr const &explanation = "synchronization");
	  ~DecisionPoint() {}

	  std::string toString() const;
	  std::string toShortString() const;
	  
	  static bool customStaticMatch(EUROPA::EntityId const &entity);

	private:
	  typedef std::bitset<4> choices;
	  
	  enum Choice {
	    EXTEND_CURRENT = 0,
	    START_NEXT =1,
	    CREATE_DEFAULT =2,
	    CREATE_OTHER =3
	  }; // TREX::europa::details::CurrentState::DecisionPoint::Choice

	  choices m_choices;
	  size_t  m_prev_idx, m_idx;
	  
	  void handleInitialize();

	  bool hasNext() const;
	  void handleExecute();

	  bool canUndo() const;
	  void handleUndo();
      
	  void advance();

	  EUROPA::Id<CurrentState> m_target;
	  std::list<EUROPA::TokenId>::const_iterator m_cand_from, m_tok, m_cand_to;
	  std::set<EUROPA::LabelStr>::const_iterator m_next_pred;
	}; // TREX::europa::details::CurrentState::DecisionPoint
	
      private:
	CurrentState(Assembly &assembly, EUROPA::TimelineId const &timeline);

	void push_end(EUROPA::DbClientId const &cli);
	void relax_end(EUROPA::DbClientId const &cli);

	EUROPA::TokenId new_obs(EUROPA::DbClientId const &cli, std::string const &pred,
				bool insert=true);
	void new_token(EUROPA::TokenId const &token);
	void relax_token();

	void erased(EUROPA::TokenId const &token);
	void replaced(EUROPA::TokenId const &token);

	Assembly          &m_assembly;
	EUROPA::TimelineId m_timeline;
	id_type            m_id;

	EUROPA::LabelStr           m_pred_default;
	std::set<EUROPA::LabelStr> m_pred_names;

	EUROPA::TokenId       m_last_obs, m_prev_obs;
	EUROPA::ConstraintId  m_constraint;
	 
	friend class TREX::europa::Assembly;
	friend class DecisionPoint;
      }; // TREX::europa::details::CurrentState
      
      class UpdateMatchFinder :public EUROPA::SOLVERS::MatchFinder {
      public:
        void getMatches(EUROPA::SOLVERS::MatchingEngineId const &engine,
                        EUROPA::EntityId const &entity,
                        std::vector<EUROPA::SOLVERS::MatchingRuleId> &result);
      }; // TREX::europa::details::UpdateMatchFinder

    } // TREX::europa::details    
  } // TREX::europa
} // TREX

#endif // H_trex_europa_CurrentState
