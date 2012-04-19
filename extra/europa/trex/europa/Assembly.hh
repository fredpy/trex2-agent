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

    class Assembly :public EUROPA::EngineBase, boost::noncopyable {
    private:
      typedef TREX::utils::list_set<details::CurrentStateId_id_traits> state_map;
      static std::string const MODE_ATTR;
      static std::string const DEFAULT_ATTR;
      static std::string const PLAN_ATTR;

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

      /** @brief Constructor
       * @param[in] name A symbolic name
       *
       * Create a new instance named @p name
       */
      Assembly(std::string const &name);
      /** @brief Destructor */
      virtual ~Assembly();

      /** @brief Check if internal
       * @param[in] tok A token
       *
       * Checks if the token @p tok necessarily belong to an Internal timeline
       *
       * @retval true if all the possible objects of @p tok ar Internal timelines
       * @retval false otherwise
       *
       * @sa internal(EUROPA::ObjectId const &) const
       */
      bool internal(EUROPA::TokenId const &tok) const;
      /** @brief Check if internal
       * @param[in] obj An object
       *
       * Check if @p obj is a TREX internal timeline.
       *
       * @retval true if @p obj is an internal timeline
       * @retval false otherwise
       *
       * @sa is_internal(EUROPA::LabelStr const &) const
       * @sa is_agent_timeline(EUROPA::ObjectId const &) const
       */
      bool internal(EUROPA::ObjectId const &obj) const;
      /** @brief Check if external
       * @param[in] tok A token
       *
       * Checks if the token @p tok necessarily belong to an External timeline
       *
       * @retval true if all the possible objects of @p tok ar External timelines
       * @retval false otherwise
       *
       * @sa external(EUROPA::ObjectId const &) const
       */
      bool external(EUROPA::TokenId const &tok) const;
      /** @brief Check if external
       * @param[in] obj An object
       *
       * Check if @p obj is a TREX external timeline.
       *
       * @retval true if @p obj is an external timeline
       * @retval false otherwise
       *
       * @sa is_external(EUROPA::LabelStr const &) const
       * @sa is_agent_timeline(EUROPA::ObjectId const &) const
       */
      bool external(EUROPA::ObjectId const &obj) const;
      /** @brief Check if ignored
       * @param[in] tok A token
       *
       * Checks if the token @p tok does necessarily belong to an
       * object ignored by the solvers.
       *
       * Such token will be ignored during deliberation
       *
       * @retval true if all the possible objects of @p tok are ignored timelines
       * @retval false otherwise
       *
       * @sa ignored(EUROPA::ObjectId const &) const
       */
      bool ignored(EUROPA::TokenId const &tok) const;
      /** @brief Check if ignored
       * @param[in] obj An object
       *
       * Check if the object @p obj is to be ignored by the solvers
       *
       * @retval true if @p obj is ignored
       * @retval false otherwise
       */
      bool ignored(EUROPA::ObjectId const &obj) const {
        return m_ignored.end()!=m_ignored.find(obj);
      }

      bool is_agent_timeline(EUROPA::ObjectId const &obj) const {
        return schema()->isA(obj->getType(), TREX_TIMELINE);
      }
      bool is_agent_timeline(EUROPA::DataTypeId const &type) const {
        return schema()->isA(type->getName(), TREX_TIMELINE);
      }
      bool is_agent_timeline(EUROPA::TokenId const &token) const;

      /** @brief Current tick
       * @return the current tick date
       *
       * @note this class assumes that this tick value is always between the
       * initial and final tick. It also assumes that it monotonocally increase
       * during the mission.
       *
       * @sa initial_tick()
       * @sa final_tick()
       */
      virtual EUROPA::eint now()              const =0;
      /** @brief execution latency
       *
       * @return the expected latency (in tick) for this engine
       *         to resolve its objectives.
       */
      virtual EUROPA::eint latency()          const =0;
      /** @brief planning look ahead
       *
       * This indicates how many ticks ahead in the future this engine is
       * evaluating during planning
       *
       * @return the planning  look-ahead for this engine
       */
      virtual EUROPA::eint look_ahead()       const =0;
      /** @brief Initial mission tick
       *
       * Indicate the first tick of the mission for this engine.
       * @return mission initial tick
       * @sa now()
       * @sa final_tick()
       */
      virtual EUROPA::eint initial_tick()     const =0;
      /** @brief Final tick of the mission
       *
       * Indicate the last tick of the mission for this engine.
       * @return mission final tick
       * @sa intial_tick()
       * @sa now()
       */
      virtual EUROPA::eint final_tick()       const =0;
      /** @brief Tick duration
       *
       * An arbitrary value that allow to convert a tick into a real-time value
       *
       * @return the duration of a single tick
       *
       * @note As it is a duration, we assume that this value is strictly
       *      positive.
       */
      virtual EUROPA::edouble tick_duration() const =0;

      /** @brief Clock variable
       *
       * This method gives an easy access to the varaiable used in the model to
       * represent the mission clock. This variable is updated as time advance
       * and can be used to idenyify whether an instant is in the past or
       * future
       *
       * @pre the clock has been initialized
       *
       * @return A reference to the clock variable
       *
       * @sa init_clock_vars()
       */
      EUROPA::ConstrainedVariableId const &clock() const {
        return m_clock;
      }

      /** @brief Current scope of the plan
       *
       * Identifies the current planning window used by the planner.
       * This window is the interval between @c now() and
       * @c latency()+look_ahead() in the future bounded by @c final_tick()
       *
       * @return An interval giving the window the planner will focus on
       *
       * @sa planner() const
       * @sa now() const
       * @sa latency() const
       * @sa look_ahead() const
       * @sa final_tick() const
       * @sa DeliberationFilter
       */
      EUROPA::IntervalIntDomain plan_scope() const {
        EUROPA::eint end_plan(now()+latency()+look_ahead());
        return EUROPA::IntervalIntDomain(now(), std::min(end_plan,
                                                         final_tick()));
      }

    protected:
      /** @brief Set debug output stream
       *
       * This emthod allow to reset the europa debug output
       * stream to the file dedicated to this engine. As all the @c debugMsg
       * of Europa are managed by a central singleton, his call allow to put
       * all the messages for this sepcific engine into a special file.
       *
       * @post all the @c debugMsg are now redirected to the file m_debug
       *
       * @sa m_debug
       */
      void setStream() const;

      /** @brief Initialize plan clock varaiables
       *
       * Extract and initialize the variables in the model used to represent
       * the agent mission time. These varaiables (mostly constants) allow the
       * modeler to integrate mission timeling information into their model.
       */
      void init_clock_vars();
      /** @brief Add TREX state variable
       * @param[in] obj An europa timeline
       *
       * Notifies this class that the timeline @p obj is a TREX timeline
       * connected to the agent.
       *
       * @post @p obj is now marked as a state variable which can trigger new
       *       current state flaws in the @c synchronizer() if it eever becomes
       *       internal
       * @sa synchronizer()
       * @sa CurrentState
       * @sa internal(EUROPA::ObjectId const &) const
       * @sa external(EUROPA::ObjectId const &) const
       */
      void add_state_var(EUROPA::TimelineId const &obj);

      EUROPA::TokenId create_token(EUROPA::ObjectId const &obj,
				   std::string const &name,
				   bool fact);
      EUROPA::TokenId new_obs(EUROPA::ObjectId const &obj,
			      std::string &pred,
			      bool &undefined);
      void recalled(EUROPA::TokenId const &tok);


      /** @brief Mark as ignored
       *
       * @param[in] obj An object
       *
       * Exclude @p obj from deliberation. Thids object and its attached tokens
       * will be ignored from both the planner and synchronizer
       *
       * @sa ignored(EUROPA::ObjectId const &obj) const
       */
      void ignore(EUROPA::ObjectId const &obj) {
        m_ignored.insert(obj);
      }

      EUROPA::SchemaId           const &schema() const {
        return m_schema;
      }
      EUROPA::PlanDatabaseId     const &plan_db() const {
        return m_plan;
      }
      EUROPA::ConstraintEngineId const &constraint_engine() const {
        return m_cstr_engine;
      }

      EUROPA::ConstrainedVariableId mode(EUROPA::ObjectId const &obj) const {
        return attribute(obj, MODE_ATTR);
      }
      EUROPA::ConstrainedVariableId default_pred(EUROPA::ObjectId const &obj) const {
        return attribute(obj, DEFAULT_ATTR);
      }
      bool with_plan(EUROPA::ObjectId const &obj) const;
      void trex_timelines(std::list<EUROPA::ObjectId> &timelines) const {
        plan_db()->getObjectsByType(TREX_TIMELINE, timelines);
      }
      bool have_predicate(EUROPA::ObjectId const &obj,
			  std::string &name) const;


      virtual bool is_internal(EUROPA::LabelStr const &name) const =0;
      virtual bool is_external(EUROPA::LabelStr const &name) const =0;

      virtual void notify(EUROPA::LabelStr const &object, EUROPA::TokenId const &obs) =0;
      virtual bool dispatch(EUROPA::TimelineId const &tl, EUROPA::TokenId const &tok) =0;
      virtual bool discard(EUROPA::TokenId const &tok) { return false; }
      virtual void cancel(EUROPA::TokenId const &tok) {}
      virtual void plan_dispatch(EUROPA::TimelineId const &tl, EUROPA::TokenId const &tok) =0;

      /** @brief Iterator through TREX state variables
       *
       * The type used to iterate through all the CurrentState instances
       * maintaining the Internal and External state -- in TREX sense -- of
       * this engine.
       *
       * @sa external_iterator
       * @sa internal_iterator
       * @sa TREX::europa::details::CurrentState
       */
      typedef state_map::const_iterator state_iterator;
      /** @brief Iterator through TREX external state variables
       *
       * The type used to iterate through all the CurrentState instances
       * maintaining the External state -- in TREX sense -- of
       * this engine.
       *
       * @sa state_iterator
       * @sa internal_iterator
       * @sa TREX::europa::details::CurrentState
       */
      typedef boost::filter_iterator<details::is_external, state_iterator> external_iterator;
      /** @brief Iterator through TREX internal state variables
       *
       * The type used to iterate through all the CurrentState instances
       * maintaining the Internal state -- in TREX sense -- of this engine.
       *
       * @sa internal_iterator
       * @sa state_iterator
       * @sa TREX::europa::details::CurrentState
       */
      typedef boost::filter_iterator<details::is_internal, state_iterator> internal_iterator;

      state_iterator begin() const {
        return m_agent_timelines.begin();
      }
      state_iterator end() const {
        return m_agent_timelines.end();
      }
      external_iterator begin_external() const {
        return external_iterator(begin(), end());
      }
      external_iterator end_external() const {
        return external_iterator(end(), end());
      }
      internal_iterator begin_internal() const {
        return internal_iterator(begin(), end());
      }
      internal_iterator end_internal() const {
        return internal_iterator(end(), end());
      }

      /** @brief Load a model
       * @param[in] nddl A nddl model file
       *
       * Load the model described in the file @p nddl
       *
       * @pre @p nddl must be a valid nddl file
       * @throw EuropaException Failed to load the file @p nddl
       * @post the model described in @p nddl is loaded in the engine
       *
       * @retval true   the plan database is consistent
       * @retval false  the plan database is now inconsistent
       */
      bool playTransaction(std::string const &nddl);
      /** @brief Configure solvers
       * @param[in] cfg A XML solver configuration file name
       *
       * Configure both the planner and synchronizer solvers using the
       * file @p cfg as a basis
       *
       * @pre @p cfg is a valid XML solver configuration file
       * @post bothe the planner and synchronizer solvers are configured using
       *      @p cfg as a basis with the addition of their dedicated filters
       *      and flawmanagers
       *
       * @sa DeliberationFilter
       * @sa SynchronizationFilter
       * @sa SynchronizationManager
       */
      void configure_solvers(std::string const &cfg);

      /** @brief New tick notification
       *
       * Notifies this solver that the tick date has been updated. It can then
       * updates the clock variable to be greater or equal to now()
       *
       * @pre The clock has been initialized
       * @post the clock varaiable is updated to the interval [@c now(), @c final_tick()]
       *
       * @sa clock()
       * @sa now()
       */
      void new_tick();
      /** @brief Synchronizer
       *
       * Give acces to the solver dedicated for synchronization.
       * This solver is a regular Europa solver that filters out from its
       * deliberation all tokens that do not overlap the current tick and
       * also resolve current state flaws for all the internal timelines
       * of this engine
       *
       * @sa now() const
       * @sa SynchronizationManager
       * @sa planner()
       */
      EUROPA::SOLVERS::SolverId synchronizer() {
        return m_synchronizer;
      }
      /** @brief Planner
       *
       * Give access to the solver dedicated for planning. This solver is a regular Europa
       * solver that filters out from its deliberation all tokens outside of the plan_scope
       *
       * @return the planner
       *
       * @sa plan_scope() const
       * @sa synchronizer()
       */
      EUROPA::SOLVERS::SolverId planner() {
        return m_planner;
      }
      /** @brief Gets the Future plan
       *
       * For now it prints the furture plan
       *
       */
      void getFuturePlan();

      bool do_synchronize();
      bool relax(bool aggressive);
      void archive();

      void print_plan(std::ostream &out, bool expanded=false) const;
    private:
      class listener_proxy :public EUROPA::PlanDatabaseListener {
      public:
        listener_proxy(Assembly &owner):EUROPA::PlanDatabaseListener(owner.plan_db()), m_owner(owner) {}
        ~listener_proxy() {}

      private:
        void notifyAdded(EUROPA::TokenId const &token);
        void notifyRemoved(EUROPA::TokenId const &token);

        void notifyActivated(EUROPA::TokenId const &token);
        void notifyDeactivated(EUROPA::TokenId const &token);

        void notifyMerged(EUROPA::TokenId const &token);
        void notifySplit(EUROPA::TokenId const &token);

        void notifyRejected(EUROPA::TokenId const &token);
        void notifyReinstated(EUROPA::TokenId const &token);

        void notifyCommitted(EUROPA::TokenId const &token);
        void notifyTerminated(EUROPA::TokenId const &token);

        Assembly &m_owner;
      };

      std::auto_ptr<listener_proxy> m_proxy;
      std::string m_name;
      TREX::utils::SingletonUse<details::Schema> m_trex_schema;

      EUROPA::SchemaId           m_schema;
      EUROPA::PlanDatabaseId     m_plan;
      EUROPA::ConstraintEngineId m_cstr_engine;

      EUROPA::SOLVERS::SolverId m_planner;
      EUROPA::SOLVERS::SolverId m_synchronizer;

      // Plan database special objects
      EUROPA::ConstrainedVariableId m_clock;
      EUROPA::ObjectSet m_ignored;
      state_map         m_agent_timelines;
      EUROPA::TokenSet  m_roots, m_completed;

      mutable std::ofstream m_debug;

      EUROPA::ConstrainedVariableId attribute(EUROPA::ObjectId const &obj,
                                              std::string const &attr) const;
      void predicates(EUROPA::ObjectId const &obj, std::set<EUROPA::LabelStr> &pred) const {
        return schema()->getPredicates(obj->getObjectType(), pred);
      }
      void notify(details::CurrentState const &state);
      void terminate(EUROPA::TokenId const &tok);


      friend class TREX::europa::details::Schema;
      friend class TREX::europa::details::UpdateFlawIterator;
      friend class TREX::europa::details::CurrentState;
      friend class listener_proxy;
    }; // TREX::europa::Assembly


/*    class Assembly_old :public EUROPA::EngineBase, boost::noncopyable {




      virtual void discard(EUROPA::TokenId const &tok) {}
      virtual void cancel(EUROPA::TokenId const &tok) {}
      virtual bool dispatch(EUROPA::TimelineId const &tl,
                            EUROPA::TokenId const &tok) =0;
      void recalled(EUROPA::TokenId const &tok);


      virtual void notify(EUROPA::LabelStr const &object, EUROPA::TokenId const &obs) =0;



      EUROPA::TokenId create_token(EUROPA::ObjectId const &obj,
				   std::string const &name,
				   bool fact);


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

      void print_plan(std::ostream &out, bool expanded=true) const;

    private:


      bool internal(details::CurrentState const &state) const;
      void notify(details::CurrentState const &state);

      EUROPA::RulesEngineId      m_rules_engine;





    protected:

      EUROPA::TokenId new_obs(EUROPA::ObjectId const &obj,
			      std::string &pred,
			      bool &undefined);

    private:
      void doDiscard(EUROPA::TokenId const &tok);

    }; // TREX::europa::Assembly
 */

  } // TREX::europa
} // TREX

#endif // H_trex_europa_Assembly
