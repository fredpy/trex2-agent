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
# include "config.hh"

# include <trex/utils/id_mapper.hh>

# include <PLASMA/PlanDatabase.hh>
# include <PLASMA/RulesEngineDefs.hh>
# include <PLASMA/Module.hh>
# include <PLASMA/Engine.hh>
# include <PLASMA/Solver.hh>

# include <boost/iterator/filter_iterator.hpp>

# include <fstream>
# include <memory>

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

      struct is_external {

	bool operator()(CurrentStateId const &timeline) const;
      }; // TREX::europa::details::is_external

      struct is_internal {

	bool operator()(CurrentStateId const &timeline) const;
      }; // TREX::europa::details::is_internal

      struct token_id_traits {
	typedef EUROPA::TokenId base_type;
	typedef EUROPA::eint id_type;

	static id_type get_id(base_type const &t);
      }; // TREX::europa::details::token_id_traits

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
    private:
      typedef TREX::utils::list_set<details::CurrentStateId_id_traits> state_map;

    protected:
      typedef state_map::const_iterator state_iterator;
      typedef boost::filter_iterator<TREX::europa::details::is_external,
				     state_iterator> external_iterator;
      typedef boost::filter_iterator<TREX::europa::details::is_internal,
				     state_iterator> internal_iterator;
    public:
      /** @brief TREX timeline nddl type
       * The name of the type in nddl for a TREX timeline
       */
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

      /** @brief Constructor
       * @param[in] name A symbolic name
       *
       * Create a new instance with the associated name @p name
       */
      Assembly(std::string const &name);
      /** @brief Destructor
       */
      ~Assembly();

      /** @brief Load the associated model
       * @param[in] nddl A nddl file name
       *
       * Load the nddl model described in the file @p nddl in this Assembly
       * plan database.
       *
       * @throw EuropaException An erro occured while trying to parse the model
       *
       * @retval true if the model was succesfully loaded and is consitent
       * @retval false the loaded model generated an inconsistency
       */
      bool playTransaction(std::string const &nddl);
      /** @brief Configure solvers
       * @param[in] cfg A xml solver configuration file
       *
       * Create and intialize the planning and synchronization solvers for this
       * Assembly using @p cfg configuration file.
       *
       * @note this method injects extra component in @p cfg to both solvers.
       * Namely:
       * @li A @c DeliberationScope filter to the @e planniing solver; used in
       *     order to exclude tokens that are outside of the planning scope
       *     of the reactor.
       * @li A @c SynchronizationScope filter to the @c synchronization solver;
       *     used to restrict the synchronization to only tokens that overlaps
       *     the current tick
       * @li A @c SynchronizationManager along with its @p
       *     CurrentState::DecisionPoint to the @c synchropnization solver; both
       *     used in concert to identify the current @e Internal state of the
       *     reactor
       */
      void configure_solvers(std::string const &cfg);

      /** @brief Get europa schema
       * @return the Europa schema associated to this Assembly
       */
      EUROPA::SchemaId const &schema() const {
	return m_schema;
      }

      /** @brief Get europa constraint engine
       * @return the Europa constraint engine associated to this Assembly
       */
      EUROPA::ConstraintEngineId const &constraint_engine() const {
	return m_cstr_engine;
      }
      /** @brief Get europa plan database
       * @return the Europa plan database associated to this Assembly
       */
      EUROPA::PlanDatabaseId const &plan_db() const {
	return m_plan_db;
      }
      /** @brief Get europa rules engine
       * @return the Europa rules engine associated to this Assembly
       */
      EUROPA::RulesEngineId const &rules_engine() const {
	return m_rules_engine;
      }

      /** @brief Check if internal
       * @param[in] tok A token
       *
       * This method is used in order to check if the token @p tok is
       * necessarily associated to a T-REX @e internal timeline
       * @retval true if all the possible objects for @p tok are T-REX
       *              @e internal timelines
       * @retval false otherwise
       *
       * @sa internal(EUROPA::ObjectId const &) const
       * @sa is_internal(EUROPA::LabelStr const &) const
       * @sa external(EUROPA::TokenId const &) const
       * @sa external(EUROPA::ObjectId const &) const
       */
      bool internal(EUROPA::TokenId const &tok) const;
      /** @brief Check if internal
       * @param[in] obj An object
       *
       * This method is used in order to check if the object @p obj is
       * a T-REX @e internal timeline
       * @retval true if @p obj is an @e internal timeline
       * @retval false otherwise
       *
       * @sa internal(EUROPA::TokenId const &) const
       * @sa is_internal(EUROPA::LabelStr const &) const
       * @sa external(EUROPA::TokenId const &) const
       * @sa external(EUROPA::ObjectId const &) const
       */
      bool internal(EUROPA::ObjectId const &obj) const;

      /** @brief Check if external
       * @param[in] tok A token
       *
       * This method is used in order to check if the token @p tok is
       * necessarily associated to a T-REX @e external timeline
       * @retval true if all the possible objects for @p tok are T-REX
       *              @e external timelines
       * @retval false otherwise
       *
       * @sa external(EUROPA::ObjectId const &) const
       * @sa is_external(EUROPA::LabelStr const &) const
       * @sa internal(EUROPA::TokenId const &) const
       * @sa internal(EUROPA::ObjectId const &) const
       */
      bool external(EUROPA::TokenId const &tok) const;
      /** @brief Check if external
       * @param[in] obj An object
       *
       * This method is used in order to check if the object @p obj is
       * a T-REX @e external timeline
       * @retval true if @p obj is an @e external timeline
       * @retval false otherwise
       *
       * @sa external(EUROPA::TokenId const &) const
       * @sa is_internal(EUROPA::LabelStr const &) const
       * @sa internal(EUROPA::TokenId const &) const
       * @sa internal(EUROPA::ObjectId const &) const
       */
      bool external(EUROPA::ObjectId const &obj) const;

      /** @brief Check if ignored
       * @param[in] tok A token
       *
       * Checks if the token @p tok is marked as being ignored by the solvers.
       * A token is ignored when all its possible objects are ignored
       *
       * @retval true if the token is marked as ignored
       * @retval false otherwise
       *
       * @sa ignored(EUROPA::ObjectId const &) const
       */
      bool ignored(EUROPA::TokenId const &tok) const;
      /** @brief Check if ignored
       * @param[in] obj An object
       *
       * Checks if the object @p obj is marked as ignored by the solvers.
       *
       * @retval true if @p is to be ignored
       * @retval false otherwise
       *
       * @sa ignored(EUROPA::TokenId const &) const
       */
      bool ignored(EUROPA::ObjectId const &obj) const {
	return m_ignored.end()!=m_ignored.find(obj);
      }

      /** @brief Execution frontier
       *
       * This method is used by the Assembly in order to get the current T-REX
       * tick.
       *
       * @return the current tick
       */
      virtual EUROPA::eint now() const =0;

      virtual EUROPA::eint latency() const =0;
      virtual EUROPA::eint look_ahead() const =0;

      /** @brief Deliberation scope
       *
       * This method is used by the Assembly -- or more accvurately by the
       * associated @c DeliberationScope -- in  order to identify the reactor
       * current planning scope. The planning scope is the time interval for
       * for which the reactor needs to deliberate
       *
       * @return the planning scope
       */
      virtual EUROPA::IntervalIntDomain plan_scope() const =0;
      /** @brief initial tick
       *
       * Indicates this reactor initial tick. The initial tick is the first
       * tick this reactor started to execute.
       *
       * @note It is assumed that any tick before this are not relevant for the
       * reactor.
       *
       * @return reactor initial tick
       */
      virtual EUROPA::eint initial_tick() const =0;
      /** @brief final tick
       *
       * Indicates this reactor final tick. The final tick reflects the T-REX
       * agent life time.
       *
       * @note It is assumed that any tick after this are not relevant for the
       * reactor.
       *
       * @return reactor final tick
       */
      virtual EUROPA::eint final_tick() const =0;
      /** @brief duration of the tick
       *
       * This value is used in order to convert T-REX ticks into a real-time
       * value. They reflect the duration -- usually in seconds -- of a single
       * tick as sepcified by the T-REX clock
       *
       * @return the duration of a tick
       */
      virtual EUROPA::edouble tick_duration() const =0;

      /** @brief T-REX clock variable
       *
       * This method give access to the varable in the model used to
       * represent the T-REX clock.
       *
       * @return the variable in nddl for T_REX clock
       */
      EUROPA::ConstrainedVariableId clock() const {
	return m_clock;
      }

      /** @brief Check if T-REX timeline
       * @param[in] obj An object
       *
       * Checks if @p obj type is the type used for T-REX timelines in nddl.
       * Meaning that it derives from @c TREX_TIMELINE
       *
       * @retval true   if @p obj is a trex timeline
       * @retval false  otherwise
       */
      bool is_agent_timeline(EUROPA::ObjectId const &obj) const {
	return schema()->isA(obj->getType(), TREX_TIMELINE);
      }
      /** @brief Check if T-REX timeline
       * @param[in] type A europa object type
       *
       * Checks if @p type is the type used for T-REX timelines in nddl.
       * Meaning that it derives from @c TREX_TIMELINE
       *
       * @retval true   if @p type is a trex timeline
       * @retval false  otherwise
       */
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

      virtual void discard(EUROPA::TokenId const &tok) {}
      virtual void cancel(EUROPA::TokenId const &tok) {}
      virtual bool dispatch(EUROPA::TimelineId const &tl,
                            EUROPA::TokenId const &tok) =0;
      void recalled(EUROPA::TokenId const &tok);

      void setStream() const;

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
      void add_state_var(EUROPA::TimelineId const &obj);

      bool relax(bool destructive, std::string const &fname) {
        if( relax(destructive) ) {
          std::ofstream out(fname.c_str());
          print_plan(out);
          return true;
        }
        return false;
      }
      bool relax(bool destructive);
      void archive();

      void print_plan(std::ostream &out, bool expanded=false) const;

    private:
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


      state_map m_agent_timelines;

      class root_tokens :public EUROPA::PlanDatabaseListener {
      public:
        root_tokens(EUROPA::PlanDatabaseId const &db)
          :EUROPA::PlanDatabaseListener(db), m_owner(NULL) {}
        ~root_tokens() {}

        EUROPA::TokenSet const &roots() const {
          return m_roots;
        }
        EUROPA::TokenSet const &committed() const {
          return m_committed;
        }

        void attach(Assembly *owner) {
          m_owner = owner;
        }
      private:
        void notifyAdded(EUROPA::TokenId const & token);
        void notifyRemoved(EUROPA::TokenId const & token);
        void notifyDeactivated(EUROPA::TokenId const & token);
        void notifyCommitted(EUROPA::TokenId const & token);


        EUROPA::TokenSet m_roots;
        EUROPA::TokenSet m_committed;
        Assembly *m_owner;
      }; // TREX::europa::Assembly::root_tokens

      std::auto_ptr<root_tokens> m_roots;


    protected:
      state_iterator begin() const {
	return m_agent_timelines.begin();
      }
      state_iterator end() const {
	return m_agent_timelines.end();
      }

      EUROPA::TokenId new_obs(EUROPA::ObjectId const &obj,
			      std::string &pred,
			      bool &undefined);

    private:
      void terminate(EUROPA::TokenId const &token);
      void doDiscard(EUROPA::TokenId const &tok);




      typedef TREX::utils::list_set<details::token_id_traits> token_set;
      token_set m_terminated;

      EUROPA::SOLVERS::SolverId m_deliberation_solver;
      EUROPA::SOLVERS::SolverId m_synchronization_solver;

      friend class TREX::europa::details::UpdateFlawIterator;
      friend class TREX::europa::details::CurrentState;
      friend class root_tokens;
    }; // TREX::europa::Assembly

  } // TREX::europa
} // TREX

#endif // H_trex_europa_Assembly
