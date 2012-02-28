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
#ifndef H_trex_europa_Assembly
# define H_trex_europa_Assembly

# include "EuropaPlugin.hh"

# include <trex/utils/id_mapper.hh>

# include <PLASMA/PlanDatabase.hh>
# include <PLASMA/RulesEngineDefs.hh>
# include <PLASMA/Module.hh>
# include <PLASMA/Engine.hh>
# include <PLASMA/Solver.hh>

# include <fstream>

namespace TREX {
  namespace europa {

    namespace details {
      class CurrentState;

      typedef EUROPA::Id<CurrentState> CurrentStateId;

      struct CurrentStateId_id_traits {
	typedef CurrentStateId     base_type;
	typedef EUROPA::TimelineId id_type;
	
	static id_type get_id(base_type const &cs);
      }; // TREX::europa::details::CurrentStateId_id_traits

      class UpdateFlawIterator;

    } // TREX::europa::details

    /** @brief Europa to T-REX assembly
     * 
     * This class implements the assembly between europa framework and the
     * T-REX europa reactor. It ties an europa engin to a reactor giving
     * access to each framework to their basic functionalities.
     *
     * @author Frederic Py <fpy@mbari.org>
     */
    class Assembly :public EUROPA::EngineBase, boost::noncopyable {
    public:
      static EUROPA::LabelStr const TREX_TIMELINE;
      static EUROPA::LabelStr const EXTERNAL_MODE;
      static EUROPA::LabelStr const OBSERVE_MODE;
      static EUROPA::LabelStr const INTERNAL_MODE;
      static EUROPA::LabelStr const PRIVATE_MODE;
      static EUROPA::LabelStr const IGNORE_MODE;

      static EUROPA::LabelStr const MISSION_END;
      static EUROPA::LabelStr const TICK_DURATION;
      static EUROPA::LabelStr const CLOCK_VAR;

      static std::string const UNDEFINED_PRED;
      static std::string const FAILED_PRED;
      
      Assembly(std::string const &name);
      ~Assembly();

      bool playTransaction(std::string const &nddl);
      void configure_solvers(std::string const &cfg);

      EUROPA::SchemaId const &schema() const {
	return m_schema;
      }
      EUROPA::ConstraintEngineId const &constraint_engine() const {
	return m_cstr_engine;
      }
      EUROPA::PlanDatabaseId const &plan_db() const {
	return m_plan_db;
      }
      EUROPA::RulesEngineId const &rules_engine() const {
	return m_rules_engine;
      }

      bool internal(EUROPA::TokenId const &tok) const;
      bool internal(EUROPA::ObjectId const &obj) const;

      bool external(EUROPA::TokenId const &tok) const;
      bool external(EUROPA::ObjectId const &obj) const;

      bool ignored(EUROPA::TokenId const &tok) const;
      bool ignored(EUROPA::ObjectId const &obj) const {
	return m_ignored.end()!=m_ignored.find(obj);
      }

      virtual EUROPA::eint now() const =0;
      virtual EUROPA::IntervalIntDomain plan_scope() const =0;
      virtual EUROPA::eint initial_tick() const =0;
      virtual EUROPA::eint final_tick() const =0;
      virtual EUROPA::edouble tick_duration() const =0;

      EUROPA::ConstrainedVariableId clock() const {
	return m_clock;
      }

      bool is_agent_timeline(EUROPA::ObjectId const &obj) const {
	return schema()->isA(obj->getType(), TREX_TIMELINE);
      }
      bool is_agent_timeline(EUROPA::DataTypeId const &type) const {
	return schema()->isA(type->getName(), TREX_TIMELINE);
      }

    protected:
      EUROPA::ConstrainedVariableId mode(EUROPA::ObjectId const &obj) const {
	return attribute(obj, MODE_ATTR);
      }
      EUROPA::ConstrainedVariableId default_pred(EUROPA::ObjectId const &obj) const {
	return attribute(obj, DEFAULT_ATTR);
      }
      void trex_timelines(std::list<EUROPA::ObjectId> &objs) const {
	return plan_db()->getObjectsByType(TREX_TIMELINE, objs);
      }

      
      virtual bool is_internal(EUROPA::LabelStr const &name) const =0;
      virtual bool is_external(EUROPA::LabelStr const &name) const =0;

      void setStream() const;

      bool active() const;
      bool inactive() const;
      bool invalid() const;

      void ignore(EUROPA::ObjectId const &obj) {
	m_ignored.insert(obj);
      }
      virtual void notify(EUROPA::LabelStr const &object, EUROPA::TokenId const &obs) =0;

      bool have_predicate(EUROPA::ObjectId const &obj, 
			  std::string &name) const;
			  

      EUROPA::TokenId create_token(EUROPA::ObjectId const &obj, 
				   std::string const &name,
				   bool fact);

      EUROPA::SOLVERS::SolverId const &planner() const {
	return m_deliberation_solver;
      }
      EUROPA::SOLVERS::SolverId const &synchronizer() const {
	return m_synchronization_solver;
      }

      void init_clock_vars();
      
    private:
      enum State {
	INACTIVE = 0,
	ACTIVE,
	INVALID
      }; 

      static std::string const MODE_ATTR;
      static std::string const DEFAULT_ATTR;

      EUROPA::ConstrainedVariableId attribute(EUROPA::ObjectId const &obj,
                                              std::string const &attr) const;
      void predicates(EUROPA::ObjectId const &obj, std::set<EUROPA::LabelStr> &pred) const {
	return schema()->getPredicates(obj->getObjectType(), pred);
      }

      bool internal(details::CurrentState const &state) const;
      void notify(details::CurrentState const &state);

      EUROPA::SchemaId           m_schema;
      EUROPA::ConstraintEngineId m_cstr_engine;
      EUROPA::PlanDatabaseId     m_plan_db;
      EUROPA::RulesEngineId      m_rules_engine;

      TREX::utils::SingletonUse<details::Schema> m_trex_schema;
      mutable std::ofstream                      m_debug;

      EUROPA::ObjectSet             m_ignored;
      EUROPA::ConstrainedVariableId m_clock;
      
      Assembly();

      typedef TREX::utils::list_set<details::CurrentStateId_id_traits> state_map;
      typedef state_map::const_iterator state_iterator;

      state_map m_agent_timelines;

      state_iterator begin() const {
	m_agent_timelines.begin();
      }
      state_iterator end() const {
	m_agent_timelines.end();
      }

      EUROPA::SOLVERS::SolverId m_deliberation_solver;
      EUROPA::SOLVERS::SolverId m_synchronization_solver;

      friend class TREX::europa::details::UpdateFlawIterator;
      friend class TREX::europa::details::CurrentState;
    }; // TREX::europa::Assembly

  } // TREX::europa
} // TREX 

#endif // H_trex_europa_Assembly
