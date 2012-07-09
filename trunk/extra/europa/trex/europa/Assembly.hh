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

      /** @brief list set accessor for CurrentStateId
       *
       * This class is used in order to idenitfy how to organize CurrentState
       * europa Id intpo a list_set. All of them are identified by the timeline
       * their relate to.
       *
       * @author Frederic Py <fpy@mbari.org>
       * @ingroup europa
       * @relates CurrentState
       */
      struct CurrentStateId_id_traits {
        typedef CurrentStateId base_type;
        typedef EUROPA::eint   id_type;

        /** @brief Key accessor
         *
         * @param[in] cs A CurrentState instance
         * @return the timeline associated to @p cs
         */
        static id_type get_id(base_type const &cs);
      }; // TREX::europa::details::CurrentStateId_id_traits

      class UpdateFlawIterator;

      /** @brief External timeline predicate
       *
       * A functor that tests if a CurrentState instance refers to T-REX
       * External timeline
       *
       * @author Frederic Py <fpy@mbari.org>
       * @ingroup europa
       * @relates CurrentState
       * @sa is_internal
       */
      struct is_external {
        /** @brief Test operator
         *
         * @param[in] timeline a CurrentState instance
         *
         * @retavl true if the timleine of @p timeline is currently a T-REX External timeline
         * @retval false oterhwise
         */
        bool operator()(CurrentStateId const &timeline) const;
      }; // TREX::europa::details::is_external

      /** @brief Internal timeline predicate
       *
       * A functor that tests if a CurrentState instance refers to T-REX
       * Internal timeline
       *
       * @author Frederic Py <fpy@mbari.org>
       * @ingroup europa
       * @relates CurrentState
       * @sa is_external
       */
      struct is_internal {

        /** @brief Test operator
         *
         * @param[in] timeline a CurrentState instance
         *
         * @retavl true if the timleine of @p timeline is currently a T-REX Internal timeline
         * @retval false oterhwise
         */
        bool operator()(CurrentStateId const &timeline) const;
      }; // TREX::europa::details::is_internal

      /** @brief Token key extractor
       *
       * This class is used in order to implement a list_set that store europa
       * Tokens sorted by their key
       *
       * @author Frederic Py <fpy@mbari.org>
       * @ingroup europa
       */
      struct token_id_traits {
        typedef EUROPA::TokenId base_type;
        typedef EUROPA::eint id_type;

        /** @brief Key accessor
         *
         * @param[in] t A token
         * @return The key of @p t
         */
        static id_type get_id(base_type const &t);
      }; // TREX::europa::details::token_id_traits

    } // TREX::europa::details

    /** @brief T-REX/europa Deliberation/execution assembly
     *
     * This class bridges the gap between T-REX and Europa. While it does not
     * uses directly the representation from T-REX for Tokens, timelines, ...
     * and focuses on the europa internal representations; it provides an abstract
     * view of the core europa extensions required in order to implement a reactor
     * that handles to europa solvers for planning and synchronization.
     * As result the EuropaReactor class that derives from this class is mostly
     * a wrapper that handle T-REX execution and connection within an agent through
     * calls provided by this class which abstract most of the europa manipulations
     * required.
     *
     * In that sense, one can see this class as the core engine of the EuropaReactor.
     *
     * @author Frederic Py <fpy@mbari.org>
     * @ingroup europa
     */
    class Assembly :public EUROPA::EngineBase, boost::noncopyable {
    private:
      typedef TREX::utils::list_set<details::CurrentStateId_id_traits> state_map;
      static std::string const MODE_ATTR;
      static std::string const DEFAULT_ATTR;
      static std::string const PLAN_ATTR;

    public:
      /** @brief T-REX timeline type name
       *
       * The symbolic name in NDDL for timelines that are meant to be shared
       * with other reactors
       */
      static EUROPA::LabelStr const TREX_TIMELINE;
      /** @brief External T-REX timeline name
       *
       * The symbolic name in NDDL used to declare T-REX timelines as External
       */
      static EUROPA::LabelStr const EXTERNAL_MODE;
      /** @brief Observe T-REX timeline name
       *
       * The symbolic name in NDDL used to declare T-REX timelines as Observer.
       * An observer timeline is an External timeline that do not accept goal.
       */
      static EUROPA::LabelStr const OBSERVE_MODE;
      /** @brief Internal T-REX timeline name
       *
       * The symbolic name in NDDL used to declare T-REX timelines as Internal
       */
      static EUROPA::LabelStr const INTERNAL_MODE;
      /** @brief Private "T-REX" timeline name
       *
       * The symbolic name in NDDL used to declare T-REX timelines as Private.
       * This allow to use a  T-REX timeline in the model while not sharing it
       * with the other reactors.
       */
      static EUROPA::LabelStr const PRIVATE_MODE;
      /** @brief Ignored "T-REX" timeline name
       *
       * The symbolic name in NDDL used to declare T-REX timelines as Ignored.
       *
       * @deprecated This cvonstrauct is inherited from older versions of T-REX
       * where all the reactors where loading the same model instantianted
       * differently. For this reason the Ignored timelines where aloowing to
       * filter out some part of the rules. Now every europa reactor can load a
       * different model which made this construct useless appart for backward
       * compatibility.
       */
      static EUROPA::LabelStr const IGNORE_MODE;

      /** @brief Mission end model variable
       *
       * The name of the variable in the model that is used by T-REX to indicate
       * the agent mission end tick.
       */
      static EUROPA::LabelStr const MISSION_END;

      /** @brief Mission start model variable
       *
       * The name of the variable in the model that is used by T-REX to indicate
       * the reactor initial tick.
       */
      static EUROPA::LabelStr const MISSION_START;
      /** @brief Tick duration model variable
       *
       * The name of the variable in the model that is used by T-REX to indicate
       * the agent tick duration. This varaiable can be used by the agent to
       * convert ticks into a real-time value by a simple multiplication.
       */
      static EUROPA::LabelStr const TICK_DURATION;
      /** @brief T-REX Clock model variable
       *
       * The name of the variable in the model that is used by T-REX to indicate
       * the advance of time. This varaiable lower bound is updated by T-REX to
       * be always greater or equal to the current tick.
       */
      static EUROPA::LabelStr const CLOCK_VAR;

      /** @brief undefined predicate
       *
       * This is the name of the predicate T-REX will use when it is unable to
       * translate an observation into its europa equivalent.
       */
      static std::string const UNDEFINED_PRED;
      /** @brief Failed predicate
       *
       * This is the name of the predicate T_REX will use to indicate that the
       * owner of a timeline Failed to synchtonize. Such observation often
       * reflects that this timeline  is not owned by any reactor at this point.
       */
      static std::string const FAILED_PRED;

      /** @brief Constructor
       * @param[in] name A symbolic name
       *
       * Create a new instance named @p name
       */
      Assembly(std::string const &name);
      /** @brief Destructor */
      virtual ~Assembly();

      /** @brief Check for support of actions
       *
       * This method checks if this europa plugin have been
       * compiled with support of actions.
       * It is equivalent to check if the preprocessing macro
       * EUROPA_HAVE_EFFECT exists
       *
       * @note if you use europa 2.6 or above this method should return true
       *
       * @retval true if support actions
       * @retval false otherwise
       */
      static bool actions_supported();

      /** @brief Check if action
       * @param[in] tok A token
       *
       * Check if @p tok is an action token.
       *
       * @note if actions are not supported then this method will always
       * return @c false
       *
       * @retval true if @p tok is an action
       * @retval false otherwise
       * @sa is_predicate(EUROPA::TokenId const &) const
       * @sa actions_supported()
       */
      bool is_action(EUROPA::TokenId const &tok) const;
      /** @brief Check if predicate
       * @param[in] tok A token
       *
       * Check if @p tok is a predicate token
       *
       * @note if actions are not supported then this method will always
       * return @c true
       *
       * @retval true if @p tok is an action
       * @retval false otherwise
       * @sa is_action(EUROPA::TokenId const &) const
       * @sa actions_supported()
       */
      bool is_predicate(EUROPA::TokenId const &tok) const;

      /** @brief Check for condition attribute
       * @param[in] tok A token
       *
       * Check if @p tok is a condition token. An effect token means that this
       * token is semantically a condition necessary for "executing" its master
       * (which usually will be an action)
       *
       * @note if actions are not supported then this method will always
       * return @c false
       *
       * @retval true if @p tok is a condition
       * @retval false otherwise
       * @sa is_effect(EUROPA::TokenId const &) const
       * @sa actions_supported()
       */
      bool is_condition(EUROPA::TokenId const &tok) const;
      /** @brief Check for effect attribute
       * @param[in] tok A token
       *
       * Check if @p tok is an effect token. An effect token means that this
       * token is semantically yhe resulting effect of "executing" its master
       * (which usually will be an action)
       *
       * @note if actions are not supported then this method will always
       * return @c false
       *
       * @retval true if @p tok is an effect
       * @retval false otherwise
       * @sa is_condition(EUROPA::TokenId const &) const
       * @sa actions_supported()
       */
      bool is_effect(EUROPA::TokenId const &tok) const;


      /** @brief Test if goal
       *
       * @param[in] tok A token
       *
       * Test if @p tok is a goal token meaning that it is rejectable and
       * associated to an internal timeline.
       *
       * @retval true if @p tok is a goal
       * @retval false otherwise
       *
       * @sa internal(EUROPA::TokenId const &) const
       */
      bool is_goal(EUROPA::TokenId const &tok) const;

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

      size_t look_ahead(EUROPA::ObjectId const &obj);
      /** @Brief Check if ignored
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

      /** @brief Check if T-REX timeline
       *
       * @param[in] obj An europa object
       *
       * Check if @p obj is a T-REX timeline. This timeline can be shared with
       * other reactors or not.
       *
       * @retval true  if @p obj is a T-REX timeline.
       * @retval false otherwise
       */
      bool is_agent_timeline(EUROPA::ObjectId const &obj) const {
        return schema()->isA(obj->getType(), TREX_TIMELINE);
      }
      /** @brief Check if T-REX timeline
       *
       * @param[in] type An europa object type
       *
       * Check if @p type is a T-REX timeline type.
       *
       * @retval true  if @p type is a T-REX timeline
       * @retval false otherwise
       */
      bool is_agent_timeline(EUROPA::DataTypeId const &type) const {
        return schema()->isA(type->getName(), TREX_TIMELINE);
      }
      /** @brief Check if T-REX timeline
       *
       * @param[in] toke An europa token
       *
       * Check if @p tok belongs to a T-REX timeline.
       *
       * @retval true  if @p tok belongs to a T-REX timeline.
       * @retval false otherwise
       */
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

      /** @brief Tick to date
       * @param[in] tick A Europa tick
       *
       * Convert @p tick to real-time date as provided by the clock
       *
       * @return the date for @p tick
       *
       * @note While the conversion is highly dependent on the Clock used by
       * the agent. This conversion is often into a unix time tag format.
       */
      virtual EUROPA::edouble tick_to_date(EUROPA::eint tick) const =0;
      /** @brief Date to tick
       * @param[in] date A real-time date
       *
       * Convert @p date to a Euroap tick as provided by the clock
       *
       * @return the tick for @p date
       *
       * @note While the conversion is highly dependent on the Clock used by
       * the agent. This conversion is often from a unix time tag format.
       */
      virtual EUROPA::eint date_to_tick(EUROPA::edouble date) const =0;

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

      /** @brief Create a token
       * @param[in] obj  A europa object
       * @param[in] name A predicate name
       * @param[in] fact A boolean flag
       *
       * Create a new token with the predicate type @p name and associate it
       * to the object @p obj. If @p fact is @c true this token will be created
       * as a fact, otherwise it swill be a rejectable goal.
       *
       * @pre The predicate @p name should exists for the object @p obj
       *
       * @throw EuropaException Failed to create the token. Probaly due to the
       * fact that @p obj does not decalre a predicate type @p name
       *
       * @return the newly created token
       */
      EUROPA::TokenId create_token(EUROPA::ObjectId const &obj,
                                   std::string const &name,
                                   bool fact);
      /** @brief Create a new observation
       *
       * @param[in] obj A europa object
       * @param[in] pred A predciate name
       * @param[out] undefined undefined predicate indicator
       *
       * Attempt to create the observation @p pred for the object @p obj as
       * a fact. If the predicate @p pred does not exists then is will create
       * the special observation @c undefined and set @p undefined to @c true
       * (otherwise @p undefined is set to false)
       *
       *
       * @pre @p obj is a T-REX External timeline.
       *
       * @throw EuropaException @p obj is not External
       * @throw EuropaException failed to create the new token
       *
       * @return The newly created token
       * @sa create_token(EUROPA::ObjectId const &, std::string const &, bool)
       */
      EUROPA::TokenId new_obs(EUROPA::ObjectId const &obj,
                              std::string &pred,
                              bool &undefined);
      /** @brief Goal recall notification
       *
       * @param[in] tok A goal token
       *
       * This method is used by the EuropaReactor to notify europa that the
       * token @p tok is not areuqested anymore.
       *
       * @post @p tok is removed from the plan and deleted
       */
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

      /** @brief Get europa schema
       *
       * Gives access to the Europa schema associated to this plan database.
       * The Europa schema is usefull to directly manipulate tokens within the
       * plan
       *
       * @return The europa schema for this plan database
       */
      EUROPA::SchemaId           const &schema() const {
        return m_schema;
      }
      /** @brief Get europa plan database
       *
       * Gives access to the europa plan database maintined by this instance
       *
       * @return The europa plan database
       */
      EUROPA::PlanDatabaseId     const &plan_db() const {
        return m_plan;
      }
      /** @brief Constraint engine
       *
       * Gives access to the europa constraint engine for this instance
       *
       * @return The constraint engine
       */
      EUROPA::ConstraintEngineId const &constraint_engine() const {
        return m_cstr_engine;
      }

      /** @brief Get declared mode of a T-REX timeline
       *
       * @param[in] obj An europa object
       *
       * Extract the variable that indicate the declared mode for @p obj
       * timeline. This method is used in order to identify what was the desired
       * mode for the T-REX timeline @p obj
       *
       * @pre @p obj is a T-REX timeline
       * @return The variable defining the mode of the timleine @p obj
       * @sa is_agent_timeline(EUROPA::ObjectId const &obj) const
       */
      EUROPA::ConstrainedVariableId mode(EUROPA::ObjectId const &obj) const {
        return attribute(obj, MODE_ATTR);
      }
      /** @brief Get default token type of a T-REX timeline
       *
       * @param[in] obj An europa object
       *
       * Extract the variable that indicates the declared defaul predicate for @p obj
       * timeline. This method is used in order to identify what was the desired
       * default predicate value for the T-REX timeline @p obj. When the reactor
       * is not able to identify it directly
       *
       * @pre @p obj is a T-REX timeline
       * @return The variable defining the default predciate of the timleine @p obj
       * @sa is_agent_timeline(EUROPA::ObjectId const &obj) const
       *
       * @note While in past T-REX version the default predicate was the only
       * choice evaluated during synchronization whan a Internal timeline did not
       * have a current state, this is now a less restrictive choice. Indeed our
       * solver will evaluate this default predicate first in such case but still
       * allow to evaluate other predicates (except Failed) if this solution was
       * still not valid.
       */
      EUROPA::ConstrainedVariableId default_pred(EUROPA::ObjectId const &obj) const {
        return attribute(obj, DEFAULT_ATTR);
      }
      /** @brief Check for plan publication/subscription
       *
       * @param[in] obj An object.
       *
       * This method check for the new feature currently under experimenation. Indeed,
       * since version 0.3.4 we allow timelines to publish their future to the
       * T-REX agent. Ths method chacks if the corresponbding T-REX timeline class
       * in europa is flagged as willing to publish/subscribe (depending if it is Internal/External)
       * to this future plan.
       *
       * @retval true If @p obj is a T-REX timeline with its plan attribute set to @c true
       * @retval false otherwise
       */
      bool with_plan(EUROPA::ObjectId const &obj) const;
      /** @brief Get T-REX timelines
       *
       * @param[out] timelines A list of europa objects
       *
       * Get all the T-REX timelines declared in the europa plan database and append
       * them at the end of @p timelines
       */
      void trex_timelines(std::list<EUROPA::ObjectId> &timelines) const {
        plan_db()->getObjectsByType(TREX_TIMELINE, timelines);
      }
      /** @brief Check for token type
       *
       * @param[in] obj A europa object
       * @param[in] name A predciate name
       *
       * @retval true if @p name is a valid ptoken type for @p obj
       * @retval false otherwise
       */
      bool have_predicate(EUROPA::ObjectId const &obj,
                          std::string &name) const;


      /** @brief Check for internal timeline
       *
       * @param[in] name A timeline name
       *
       * This virtual method allow the assembly to ask to T-REX if the timeline
       * @p name is currently declared as Internal by this reactor
       *
       * @retval true if @p name is currently Internal to this reactor
       * @retval false otherwise
       * @sa is_external(EUROPA::LabelStr const &) const
       */
      virtual bool is_internal(EUROPA::LabelStr const &name) const =0;
      /** @brief Check for external timeline
       *
       * @param[in] name A timeline name
       *
       * This virtual method allow the assembly to ask to T-REX if the timeline
       * @p name is currently declared as External by this reactor
       *
       * @retval true if @p name is currently External to this reactor
       * @retval false otherwise
       * @sa is_internal(EUROPA::LabelStr const &) const
       */
      virtual bool is_external(EUROPA::LabelStr const &name) const =0;
      virtual size_t look_ahead(EUROPA::LabelStr const &name) =0;

      /** @brief New internal state notification
       *
       * @param[in] object The name of a timeline
       * @param[in] obs A europa token
       *
       * This method is used as a callback to notify the reactor that the Internal
       * timeline @p object has changed its state to @p obs
       */
      virtual void notify(EUROPA::LabelStr const &object, EUROPA::TokenId const &obs) =0;
      /** @brief New external goal request
       *
       * @param[in] tl The name of a timeline
       * @param[in] tok A europa token
       *
       * This method is used as a callback to notify the reactor that the token
       * @p tok is a new future goal for the External timeline @p tl
       */
      virtual bool dispatch(EUROPA::TimelineId const &tl, EUROPA::TokenId const &tok) =0;
      /** @brief Token destruction notification
       * @param[in] tok A token
       *
       * A callback that notifies the reactor that the token @p
       * tok. Is now marked as completed in the past.
       *
       * @retval true @p tok was part of the reactor requested goals
       * @retval false @p tok is not known by the reactor
       */
      virtual bool discard(EUROPA::TokenId const &tok) { return false; }
      /** @brief Token cancelation notification
       * @param[in] tok A token
       *
       * A callback that notifies the reactor that the token @p tok has been
       * cancelled from the plan. If @p tok had be dispatched it is then the
       * responsibility of the reactor to recall it.
       */
      virtual void cancel(EUROPA::TokenId const &tok) {}

      /* @brief Goal rejection notfication 
       *
       * @param[in] tok A token
       *
       * A callback that notifies that the token @p tok is currently
       * rejected. For now the only interrest of this is to indicate
       * through a message that the corresponding trex request has
       * been rejected it may do more in the future.
       */
      virtual void rejected(EUROPA::TokenId const &tok) {}
      /** @brief New Internal plan future
       *
       * @param[in] tl An Internal timeline
       * @param[in] tok A token
       *
       * Notifies the reactor that the token @p tok is now part of the future
       * plan of timeline @p tl.
       * This is used in order to implement our new faeture where reactors are
       * able to publish their plan whenever they are done planning.
       */
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

      /** @brief T-REX timelines start iterator
       *
       * @return An iterator that points to the firrs element of the T-REX
       * timelines for this reactor. This set include all the timelines that
       * are presently shared by the reactor in T-REX.
       *
       * @sa end() const
       * @sa begin_external() const
       * @sa end_external() const
       * @sa begin_internal() const
       * @sa end_internal() const
       */
      state_iterator begin() const {
        return m_agent_timelines.begin();
      }
      /** @brief T-REX timelines end iterator
       *
       * @return An iterator that points to the end of the T-REX
       * timelines set for this reactor. This set include all the timelines that
       * are presently shared by the reactor in T-REX.
       *
       * @sa begin() const
       * @sa begin_external() const
       * @sa end_external() const
       * @sa begin_internal() const
       * @sa end_internal() const
       */
      state_iterator end() const {
        return m_agent_timelines.end();
      }
      /** @brief T-REX external timelines start iterator
       *
       * @return An iterator that points to the first element of the T-REX
       * external timelines for this reactor.
       *
       * @sa end_external() const
       * @sa begin() const
       * @sa end() const
       * @sa begin_internal() const
       * @sa end_internal() const
       */
      external_iterator begin_external() const {
        return external_iterator(begin(), end());
      }
      /** @brief T-REX external timelines end iterator
       *
       * @return An iterator that points to the end of the T-REX external
       * timelines set for this reactor.
       *
       * @sa begin_external() const
       * @sa begin() const
       * @sa end() const
       * @sa begin_internal() const
       * @sa end_internal() const
       */
      external_iterator end_external() const {
        return external_iterator(end(), end());
      }
      /** @brief T-REX internal timelines start iterator
       *
       * @return An iterator that points to the first element of the T-REX
       * internal timelines for this reactor.
       *
       * @sa end_internal() const
       * @sa begin() const
       * @sa end() const
       * @sa begin_external() const
       * @sa end_external() const
       */
      internal_iterator begin_internal() const {
        return internal_iterator(begin(), end());
      }
      /** @brief T-REX internal timelines end iterator
       *
       * @return An iterator that points to the end of the T-REX internal
       * timelines set for this reactor.
       *
       * @sa begin_internal() const
       * @sa begin() const
       * @sa end() const
       * @sa begin_external() const
       * @sa end_external() const
       */
      internal_iterator end_internal() const {
        return internal_iterator(end(), end());
      }
      
      bool locate_nddl(std::string &nddl) const;

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
       * @param[in] synchronizer A XML solver configuration file name
       * @param[in] planner A XML solver configuration file name
       *
       * Configure both the planner and synchronizer solvers using the
       * file @p planner and @c synchronizer files as respective basis
       *
       * @pre @p planner is a valid XML solver configuration file
       * @pre @p synchronizer is a valid XML solver configuration file
       * @post both the planner and synchronizer solvers are configured using
       *      their repective configuration file as a basis with the addition 
       *      of their dedicated filters and flawmanagers
       *
       * @par What does trex inject in the configuration files ?
       *
       * In order to handle properly both synchronization and deliberation, 
       * trex needs to add extra components to the solver. These allow to 
       * focus the europa solver to only the flaws that are relevant to the 
       * current problem Firo this reason the first thing that is injected
       * is a planning horizon filter acting as follow:
       * @li for the @p synchronizer the filter accepts only tokens that 
       *     may overlap the current tick (ie start can be less or equal 
       *     to current tick and @p end can be greater or equal to current 
       *     tick)
       * @li for the @p planner the filter accept all the tokens that can 
       *     overlap the window [current, current+latency+lookahead]
       * 
       * @par
       *
       * In addition the @p synchronizer needs to introduce a new type of 
       *    flaw called current state flaw that enforces the planner to identify 
       *    for each of its @e Internal tokens their state at the current tick.
       *
       * @sa DeliberationFilter
       * @sa SynchronizationFilter
       * @sa SynchronizationManager
       */
      void configure_solvers(std::string const &synchronizer, 
			     std::string const &planner);
      /** @brief Configure solvers
       * @param[in] cfg A XML solver configuration file name
       * @overload configure_solvers(std::string const &, std::stribng const &)
       *
       * A legacy method that allows to configure both solvers using @p cfg as a basis.
       */
      void configure_solvers(std::string const &cfg) {
	configure_solvers(cfg, cfg);
      }

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
       * Dispatches the plan to the plan_dispatch function
       *
       */
      void getFuturePlan();

      bool commit_externals();
      
      /** @brief Execute synchronization solver
       *
       * This method execute all the basic steps for a simple synchronization.
       * The synchronization steps are sequenced as follow:
       * @li Enforce current external state value; by either inserting new
       * observations or extending the duration of the previous observation)
       * @li Execute the synchronization solver, this solver also identifies the
       * current state value of each of the internal timelines
       * @li commit on the new state of each Internal timelines
       *
       * @pre The plan databse is in a consistent state
       * @retval true This synchronization was sucessful
       * @retval false One of the two first steps above did fail and the plan
       * database is potetntially in an inconsitent state. In any case, if the
       * returned value is @c false the synchronization failed to be resolved
       * and the reactor need to recover from this failure in order to succeed
       * its synchronization (often by calling relax)
       *
       * @sa relax(bool)
       */
      bool do_synchronize();
      /** @brief relax plan database
       *
       * @param[in] aggressive A boolean flag
       *
       * Relax the plan database constraints in order to bring back the plan in
       * a consitent state. If @p aggressive is @c true then this method will in
       * addition remove from the plan all the tokens that necessarily end in the
       * past (ie before now())
       *
       * @retval true The plan database is consistent after relaxation
       * @retval false The plan database is still inconsitent
       */
      bool relax(bool aggressive);
      /** @brief Archival of the past
       *
       * This method is called in order to allow the plan database to forget
       * about the past and by doing so reduce the tokens to the solvers and
       * constraint engine manipulate. Such operation is necessaary as europa
       * operations complexity is driectly linked to the number of tokens to the
       * plan.
       *
       * @note As of today the approach used in this implementation (based on the
       * Europa archive method) does not appear to be efficient on cleaning the
       * plan
       */
      void archive();

      /** @brief Log the plan structure
       *
       * @param[in,out] out An output stream
       * @param[in] expanded A flag
       *
       * Write the current plan in the plan database in a graphviz format to the
       * output stream @p out. @p expended flag indicates whether the display should be
       * "compact" -- where all the merged tokens are seen as one -- or expanded
       */
      void print_plan(std::ostream &out, bool expanded=false) const;
    private:
      /** @brief Synchronization backtrack
       *
       * This method is called when synchronization_listener identifies that
       * synchronization search is backtracking.
       *
       * @param[in] dp The decision point that caused the backtrack
       *
       * @sa synchronization_listener
       */
      void backtracking(EUROPA::SOLVERS::DecisionPointId &dp);

      /** @brief synchronization search events listener
       *
       * A search listener used by the Assembly to keep track of search events
       * occuring during synchronization
       *
       * @relates Assembly
       * @ingroup europa
       * @author Frederic Py
       *
       * @note This listener is meant to detect backtracks during synchronization
       * The issue is that -- up to europa 2.6 -- notifyStepFailed was not called
       * by the solver. As an alternate solution we use the ce_listener to detect
       * emptied domains.
       *
       * @sa class Assembly::ce_listener
       */
      class synchronization_listener :public EUROPA::SOLVERS::SearchListener {
      public:
        /** @brief Constructor
         *
         * @param[in] owner The creator of this instance
         */
        synchronization_listener(Assembly &owner)
          :m_owner(owner), m_progress(true) {}
        /** @brief Destructor. */
        ~synchronization_listener() {}

        void notifyCreated(EUROPA::SOLVERS::DecisionPointId& dp);
        void notifyDeleted(EUROPA::SOLVERS::DecisionPointId& dp);

        void notifyStepSucceeded(EUROPA::SOLVERS::DecisionPointId& dp);
        void notifyStepFailed(EUROPA::SOLVERS::DecisionPointId &dp);

        void notifyUndone(EUROPA::SOLVERS::DecisionPointId &dp);
        void notifyRetractSucceeded(EUROPA::SOLVERS::DecisionPointId& dp);
        void notifyRetractNotDone(EUROPA::SOLVERS::DecisionPointId& dp);

      private:
        Assembly &m_owner;
        bool m_progress;
      }; // class TREX::europa::Assembly::synchronization_listener

      friend class synchronization_listener;
      std::auto_ptr<synchronization_listener> m_synchListener;

      void print_context(std::ostream &out, EUROPA::ConstrainedVariableId const &v) const;

      class ce_listener :public EUROPA::ConstraintEngineListener {
      public:
        ce_listener(Assembly &owner);
        ~ce_listener() {}

        void notifyPropagationPreempted();
        void notifyRemoved(EUROPA::ConstrainedVariableId const &var) {
          m_empty_vars.erase(var);
        }
        void notifyChanged(EUROPA::ConstrainedVariableId const &variable,
                           EUROPA::DomainListener::ChangeType const& changeType) {
          if( changeType==EUROPA::DomainListener::EMPTIED )
            m_empty_vars.insert(variable);
          else if( !variable->lastDomain().isEmpty() )
            m_empty_vars.erase(variable);
        }

      private:
        Assembly &m_owner;
        EUROPA::ConstrainedVariableSet m_empty_vars;

      }; // class TREX::europa::Assembly::ce_listener

      friend class ce_listener;
      std::auto_ptr<ce_listener> m_ce_listener;

      bool m_in_synchronization;

      /** @brief Token manipulation event listener
       *
       * @relates Assembly
       *
       * This class is used by Assembly in order to listen to the token
       * manipulation events from the europa plan database.
       *
       * @author Frederic Py <fpy@mbari.org>
       */
      class listener_proxy :public EUROPA::PlanDatabaseListener {
      public:
        /** @brief Constructor
         * @param[in] owner the creator Assembly
         */
        listener_proxy(Assembly &owner)
        :EUROPA::PlanDatabaseListener(owner.plan_db()), m_owner(owner) {}
        /** @brief Destructor
         */
        ~listener_proxy() {}

      private:
        /** @brief Token addition callback
         * @param[in] token A Token
         *
         * Notifies that the token @p token has just been created
         * in the plan database
         */
        void notifyAdded(EUROPA::TokenId const &token);
        /** @brief Token destruction callback
         * @param[in] token A Token
         *
         * Notifies that the token @p token has just been removed from
         * the plan database
         */
        void notifyRemoved(EUROPA::TokenId const &token);

        /** @brief Token activation callback
         * @param[in] token A Token
         *
         * Notifies that the token @p token has just been made active in the
         * plan.
         */
        void notifyActivated(EUROPA::TokenId const &token);
        /** @brief Token deactivation callback
         * @param[in] token A Token
         *
         * Notifies that the token @p token has just been deactivated from the
         * plan.
         */
        void notifyDeactivated(EUROPA::TokenId const &token);

        /** @brief Merged token callback
         * @param[in] token A Token
         *
         * Notifies that the token @p token has just been merged with an active
         * token from the plan.
         */
        void notifyMerged(EUROPA::TokenId const &token);
        /** @brief Split token callback
         * @param[in] token A Token
         *
         * Notifies that the token @p token has just been split from an active
         * token from the plan.
         */
        void notifySplit(EUROPA::TokenId const &token);

        /** @brief Token rejection callback
         * @param[in] token A Token
         *
         * Notifies that the token @p token has just been rejected from the plan.
         */
        void notifyRejected(EUROPA::TokenId const &token);
        /** @brief Token reinstatiation callback
         * @param[in] token A Token
         *
         * Notifies that the token @p token is no longer rejected from the plan.
         */
        void notifyReinstated(EUROPA::TokenId const &token);

        /** @brief Token commitment callback
         * @param[in] token A Token
         *
         * Notifies that the token @p token is now committed. A committed token
         * is considered as fully proved and allow constraint propagation to
         * stop.
         */
        void notifyCommitted(EUROPA::TokenId const &token);
        /** @brief Token termination callback
         * @param[in] token A Token
         *
         * Notifies that the token @p token is now terminated. A terminated token
         * is a good candidate to be archived by the europa archive() primitive
         */
        void notifyTerminated(EUROPA::TokenId const &token);

        Assembly &m_owner;
      };

      /** @brief Checks if token is an effect of an action
       *
       * Used by the listeners to check if the token passed in is an
       * effect of an action. Returns true if it is and false otherwises
       */
      bool actionEffect(const EUROPA::TokenId& token);

      /** @brief Token Listner proxy
       *
       * A pointter to a plan database listener notifying events related
       * to token manipulation. This proxy is used for the assembly to update
       * the current status of specific tokens of the plan.
       *
       * @sa listener_proxy
       */
      std::auto_ptr<listener_proxy> m_proxy;
      /** @brief name of the Assembly
       *
       * This is the name of the assembly which should be the name of the
       * reactor within T-REX.
       */
      std::string m_name;
      /** @brief europa plugins/logging schema
       *
       * A reference to the Schema class. This class is used for basic
       * manipulation within europa. Such as :
       * @li loading Europa extension though EuropaPlugin exisiting instances
       * @li switching the europa debug log file
       */
      TREX::utils::SingletonUse<details::Schema> m_trex_schema;

      EUROPA::SchemaId           m_schema;
      EUROPA::PlanDatabaseId     m_plan;
      EUROPA::ConstraintEngineId m_cstr_engine;

      /** @brief Deliberation solver
       *
       * The solver used by the reactor during deliberation steps
       */
      EUROPA::SOLVERS::SolverId m_planner;
      /** @brief Synchronization solver
       *
       * The solver used by the reactor during its synchronization
       */
      EUROPA::SOLVERS::SolverId m_synchronizer;

      // Plan database special objects
      /** @brief clock variable
       *
       * The variable within the model used to represent the T-REX clock as
       * it advance
       */
      EUROPA::ConstrainedVariableId m_clock;
      /** @brief Ignored objects
       *
       * A set of al the objects in the plan database that are ignored
       * by the solvers
       */
      EUROPA::ObjectSet m_ignored;
      /** @rbrief T-REX shared timelines
       *
       * The set of all the T-REX timelines that are either Internal or External
       * to the reactor
       */
      state_map         m_agent_timelines;
      /** @brief Root tokens
       *
       * The set of all the tokens which have no master. Often these tokens
       * correspond to T-REX tokens such as the goals or observations received
       * from other reactor or produced to the agent.
       */
      EUROPA::TokenSet  m_roots;
      /** @brief Completed tokens
       *
       * The set of all the tokens which have been identifed as completed before
       * the current tick and are not yet committed
       *
       * @sa m_committed
       */
      EUROPA::TokenSet  m_completed;

      /** @brief Committed tokens
       *
       * The set of all the tokens which have been committed
       *
       * @sa m_completed
       */
      EUROPA::TokenSet m_committed;

      /** @brief Goal tokens
       *
       * The set of all the tokens which are goals otherwords
       * they are rejectable or tokens that are connected to a goal
       * by a condtion(token)->action->effect(goal) relationship
       *
       * @sa m_goals
       */
      EUROPA::TokenSet m_goals;

      /** @brief Current iterator
       *
       * This iterator is used to iterate through the diverse TokenSet
       * mainteined by this class. It cvan be dynamically modified whenever
       * a callback modifies one of these tokenset ensuring that this iterator
       * will not point to the element removed from the set when it occurs
       *
       * @sa archive()
       * @sa relax()
       * @sa erase(EUROPA::TokenSet &,
       */
      EUROPA::TokenSet::const_iterator m_iter;

      void erase(EUROPA::TokenSet &set, EUROPA::TokenId const &tok);

      /** @brief Europa debug file
       *
       * The file this assembly will use for europa debug log messages.
       */
      mutable std::ofstream m_debug;

      /** @brief Object attroibute extraction
       * @param[in] obj An object
       * @param[in] attr The name of an aattribute
       *
       * Extract the variable representing the attribute @p attr of @p obj
       *
       * @pre @p obj has an attribute named @p attr
       * @throe EuropaException Failed to extract the attribute @p attr from @p obj
       *
       * @return The variable for the attribute @p attr of @p obj
       */
      EUROPA::ConstrainedVariableId attribute(EUROPA::ObjectId const &obj,
                                              std::string const &attr) const;
      /** @brief Object predicates
       *
       * @param[in] obj An object
       * @param[out] pred A set
       *
       * Store all the possible predicates of @p obj in the set @p pred
       */
      void predicates(EUROPA::ObjectId const &obj, std::set<EUROPA::LabelStr> &pred) const {
        return schema()->getPredicates(obj->getObjectType(), pred);
      }
      /** @brief New observation notification
       *
       * @param[in] state A timeline maintainer
       *
       * Notifies the assembly that the timeline referred by @p state just started
       * a new token for this tick.
       */
      void notify(details::CurrentState const &state);
      /** @brief token termination
       *
       * @param[in] tok A token
       * Notifies that the token @p tok has a end  time which is before the current tick
       */
      void terminate(EUROPA::TokenId const &tok);


      friend class TREX::europa::details::Schema;
      friend class TREX::europa::details::UpdateFlawIterator;
      friend class TREX::europa::details::CurrentState;
      friend class listener_proxy;
    }; // TREX::europa::Assembly

  } // TREX::europa
} // TREX

#endif // H_trex_europa_Assembly
