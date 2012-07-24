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

# include "TimePoint.hh"

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


      /** @brief Current state flaw handler
       *
       * This class is used to keep track and resolve the current
       * state of all the reactor T-REX timelines. It is also the way
       * we implement the synchronization as a set of new flaws to be
       * resolved by the synchronizer.  The nature of these flaws is
       * to ensure that all the Internal timelines of a reactor have a
       * fully defined state for the current tick
       *
       * @author Frederic Py <fpy@mbari.org>
       * @ingroup europa
       * @relates TREX::europa::Assembly
       * @sa TREX::europa::SynchronizationManager
       */
      class CurrentState :public EUROPA::Entity {
      public:
        DECLARE_ENTITY_TYPE(CurrentState);

        typedef EUROPA::Id<CurrentState> id_type;

        /** @brief Destructor */
        ~CurrentState() {}

        /** @brief Get europa Id
         * @return An Id referring to this instance
         */
        id_type const &getId() const {
          return m_id;
        }

        /** @brief Timeline
         *
         * Give access to the timeline managed by this instance
         *
         * @return The timeline managed by this instance
         */
        EUROPA::TimelineId const &timeline() const {
          return m_timeline;
        }
        /** @brief Current date
         * @return The current execution date
         */
        EUROPA::eint now() const;
        /** @brief Check for current state
         *
         * Checks if this timeline have already a fully determined
         * state for the current date
         *
         * @retval true if there's one and only one token on this
         *         timeline that necessarily overlaps the current
         *         tick.
         * @retval false otherwise
         */
        bool identified() const;
        /** @brief Check for current state
         *
         * Checks if this tiemline have already a fully determined
         * state for the current date and have its start and end base
         * domains are already set to enforce this.
         *
         * @retval true if there's one and only one token on this
         *         timeline that necessarily overlaps the current tick
         *         even when fully relaxed.
         * @retval false otherwise
         */
        bool committed() const;

        /** @brief Current state
         *
         * @return The Token reflecting the current state
         * @pre This instance is identified
         * @sa identified
         */
        EUROPA::TokenId current() const {
          return m_last_obs;
        }
        /** @brief Previous state
         *
         * @return The Token reflecting the previous state
         * @pre This instance is identified
         * @sa identified
         */
        EUROPA::TokenId previous() const {
          return m_prev_obs;
        }
        /** @brief Posible predicates
         *
         * @return The set of all the possible predicate names that
         *         can be created for this timeline
         */
        std::set<LabelStr> const &predicates() const {
          return m_pred_names;
        }
        /** @brief Check for default predicate
         *
         * @retval true if this timeline provide a valid default
         *         predicate
         * @retval false otherwise
         *
         * @sa default_pred() const
         */
        bool has_default() const;
        /** @brief Default predicate
         *
         * @pre has_default() is @c true
         *
         * Gives the name of the defaultp predicate. The default
         * predicate of a timeline is the first predicate to be
         * attempted to be created if no exisiting token allow to
         * resolve the current state of this timeline.
         *
         * @return The name of the default predicate for this timeline
         */
        LabelStr default_pred() const {
          return m_pred_default;
        }

        /** @brief Commmit last decision
         *
         * Enforce that the currently identified state for this
         * timeline will not be relaxed in the future by restricting
         * its start end end base domains
         *
         * @pre identified() is @c true
         * @post committed() is @c true
         */
        bool commit();
        /** @brief Check if Internal
         *
         * @retval true if this timeline is Internal
         * @retval false otherwise
         * @sa external() const
         */
        bool internal() const;
        /** @brief Check if External
         *
         * @retval true if this timeline is External
         * @retval false otherwise
         * @sa internal() const
         */
        bool external() const;

        /** @brief Dispatch exisiting plan
         *
         * @param[in] lb A tick
         * @param[in] ub A tick
         *
         * @pre external() is @c true
         *
         * Dispatch all the exisiting tokens of the plan for this
         * timeline overlapping the window [@p lb, @p ub] @note The
         * algorihtm for dispatching is slightly more complex as we
         * need to avoid to dispatch tokens that are not necessary to
         * the plan goals or are guarded by uncontrolable conditions.
         */
        void do_dispatch(EUROPA::eint lb, EUROPA::eint ub);

        bool dispatch_token(const EUROPA::TokenId& token,
                            EUROPA::eint lb, EUROPA::eint ub);
        EUROPA::TokenId getGoal(const EUROPA::TokenId& token, 
                                EUROPA::eint lb, EUROPA::eint ub);
        EUROPA::TokenId searchGoal(EUROPA::TokenSet actions);
        EUROPA::TokenSet getAllTokens(const EUROPA::TokenId& token);

        /** @brief Current state decision point
         *
         * This class implements the heuristic to resolve the current
         * state of a T-REX timeline within an europa solver.
         *
         * @ingroup europa
         * @author Frederic Py <fpy@mbari.org>
         * @relates CurrentState
         */
        class DecisionPoint :public EUROPA::SOLVERS::DecisionPoint {
        public:
          /** @brief Constructor
           *
           * @param[in] client      A plan database client
           * @param[in] timeline    Reference to the flawed CurrentState
           * @param[in] config      XML configuration
           * @param[in] explanation A flaw explanation message
           */
          DecisionPoint(EUROPA::DbClientId const &client,
                        EUROPA::Id<CurrentState> const &timeline,
                        EUROPA::TiXmlElement const &config,
                        EUROPA::LabelStr const &explanation = "synchronization");
          /** @brief Destructor */
          ~DecisionPoint() {}

          /** @brief string display
           *
           * @return A string describing this decision point
           */
          std::string toString() const;
          /** @brief short string display
           *
           * @return A string giving a short description of this
           *    decision point
           * @sa toString() const
           */
          std::string toShortString() const;

          /** @brief Check for entity
           *
           * @param[in] entity A europa entity
           *
           * This method is used by europa to check if @p entity can
           * be handled by this class.
	   *
           * @retval true if @p entity is a CurrentState
           * @retval false otherwise
           */
          static bool customStaticMatch(EUROPA::EntityId const &entity);

        private:
          /** @brief Decision choices
           *
           * This set maintains all the possible high level choices
           * for this decision point.
           */
          typedef std::bitset<4> choices;

          /** @brief High level choices types
           *
           * This enum gives all the possible high level choices to
           * resolve a CurrentState flaw.
           */
          enum Choice {
            /** @brief extend current observation
             *
             * This choice will attempt to extend the curren
             * observation so it ends after the current tick
             */
            EXTEND_CURRENT = 0,
            /** @brief Start next token
             *
             * This choice will attempt to start the token directly
             * following the last observation produced
             */
            START_NEXT =1,
            /** @brief Create default token
             *
             * This choice will attempt to create the default
             * predicate and starts it a current tick
             */
            CREATE_DEFAULT =2,
            /** @brief Create a token
             *
             * This choice will attempt to create any possible tokens
             * (except the default predicate otr thje Failed token)
             * and start it at the current tick.
             */
            CREATE_OTHER =3
          }; // TREX::europa::details::CurrentState::DecisionPoint::Choice

          choices m_choices;
          size_t  m_prev_idx, m_idx;

          /** @brief Initialization
           *
           * Initialize the decision point by identifyin all the
           * choices that are relevant for the current flaw.
           */
          void handleInitialize();

          /** @brief Check for remaining choices
           *
           * @retval true if there's a possible decision left
           * @retval false if this decision point is exhausted
           */
          bool hasNext() const;
          /** @brief Execute next choice
           *
           * @pre hasNext() is @c true
           *
           * Apply the next possible choice of this decision point
           * @throw EuropaExcetionnoo decision choice available
           */
          void handleExecute();

          /** @brief Check if last decision can be undone
           *
           * @pre A decision has been executed and not undone for this
           *      instance
           * @retval true If the last decision can be undone
           * @retval false otherwise
           */
          bool canUndo() const;
          /** @brief Undo last decision
           *
           * Undo the last decision executed by this decision point.
           */
          void handleUndo();
          /** @brief Advance to next decision
           *
           * @pre hasNext() is true
           */
          void advance();

          EUROPA::Id<CurrentState> m_target;
          std::list<EUROPA::TokenId>::const_iterator 
            m_cand_from, m_tok, m_cand_to;
          std::set<EUROPA::LabelStr>::const_iterator m_next_pred;
        }; // TREX::europa::details::CurrentState::DecisionPoint

      private:
        /** @brief Constructor
         * @param[in] assembly The creator Assembly
         * @param[in] timeline A timeline
         *
         * Create a new instance associated to @p timeline
         */
        CurrentState(Assembly &assembly, EUROPA::TimelineId const &timeline);

        /** @brief push end time
         *
         * Apply a constraint that enforces that the end time of the
         * current token is greater than the current tick
         *
         * @sa relax_end()
         */
        void push_end();
        /** @brief relax end time
         *
         * Relax the constraint that enforce that the current token
         * end time is greater than the curren tick
         * @sa push_end()
         */
        void relax_end();

        /** @brief New observation
         *
         * @param[in] pred A predicate name
         * @param[in] insert Insertion flag
         *
         * Create a new observation @p pred and activate this
         * observation if @p insert is @c true.
         *
         * @pre @p pred is a valid predicate name for this timeline
         *
         * @return The newly created token with its start time
         * restricted to the current tick
         *
         * @post The current observation for this instance is the
         * newly created token
         */
        EUROPA::TokenId new_obs(std::string const &pred,
                                bool insert=true);
        /** @brief Set observation
         *
         * @param[in] token A token
         *
         * @pre @p token applies to this timeline
         *
         * @post @p token is now the current observation
         */
        void new_token(EUROPA::TokenId const &token);
        /** @brief Relax last observation
         *
         * Destroy the current observation and replace it by the
         * previous one.
         */
        void relax_token();

        /** @brief Check if committed
         *
         * Check if the current observation is properly committed or
         * not. This test goes more in depth than the committed()
         * method.
         *
         * @retval true if the current observation is committed and
         *    necessarily overlaps current tick
         * @retval false otherwise
         */
        bool check_committed() const;

        /** @brief Erased token notification
         *
         * @param[in] token A token
         *
         * Notifies this instance that the token @p token has been
         * erased from the plan database.  If this token correspond to
         * any tokens maintained by this instance, this method will
         * make the proper cleanup.
         */
        void erased(EUROPA::TokenId const &token);
        /** @brief Replace token notification
         *
         * @param[in] token A token
         *
         * Notifies that the token @p token should now be repaced by
         * its active counterpart.  If this token is miainitned by
         * this instance, this call will ensure that : @li the base
         * domains of the active token are the same as the ones of @p
         * token @li the reference of @p token in this insstance is
         * repaced by its active counterpart @post After this
         * operation @p token can be safely deleted from the plan.
         */
        void replaced(EUROPA::TokenId const &token);
        /** @brief Base domains propagation
         *
         * @param[in,out] merged A merged token
         *
         * Restrict al the based domains of the active token of @p
         * merged by the base domains of @p merged and repaced @p
         * merged by this active token.  This method is used in order
         * to update current and past observation during the repaced
         * call
         * @sa replaced(EUROPA::TokenId const &)
         */
        static void apply_base(EUROPA::TokenId &merged,
			       EUROPA::Id<TimePoint> &frontier);

        Assembly          &m_assembly;
        EUROPA::DbClientId m_client;
        EUROPA::TimelineId m_timeline;
        id_type            m_id;

        EUROPA::LabelStr           m_pred_default;
        std::set<EUROPA::LabelStr> m_pred_names;

        EUROPA::TokenId       m_last_obs, m_prev_obs;
	EUROPA::Id<TimePoint>  m_frontier, m_prev_frontier;

        friend class TREX::europa::Assembly;
        friend class DecisionPoint;
      }; // TREX::europa::details::CurrentState

      /** @brief Current state flaw handler finder
       *
       * This class is used internally by europa to associate CurrentState
       * flaws as deliberation srules and decision points.
       *
       * @relates CurrentState
       * @author Frederic Py <fpy@mbari.org>
       * @ingroup europa
       */
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
