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
#ifndef H_trex_europa_TimePoint
# define H_trex_europa_TimePoint

# include <trex/europa/config.hh>

// include plasma header as system files in order to disable warnings
# define TREX_PP_SYSTEM_FILE <PLASMA/Variable.hh>
# include <trex/europa/bits/system_header.hh>
# define TREX_PP_SYSTEM_FILE <PLASMA/Token.hh>
# include <trex/europa/bits/system_header.hh>


namespace TREX {
  namespace europa {
    namespace details {
      
      class CurrentState;
      
      /** @brief T-REX injected time point
       *
       * This class is used by T-REX to inject timepoints in the plan database
       * reflecting the advance of time within a token.
       *
       * In general this time point is related to the execution frontier
       * (or current tick) as given by the agent clock. T-REX updates these
       * timepoints for each tokens to reflect their current evolution and allow
       * europa solver to porpagate these values within the plan.
       *
       * For example when an External observation is still pending. Its
       * corresponding token will be related to one instance of this class
       * for which its value is greater or equal to the current tick of the
       * agent. This timepoint itself is related to the token start and end
       * dates ('start<this' and 'this==end'). As T-REX update the domain of
       * this timepoint this result on the porpagation of the advnace of time
       * allowing EUROPA to identify potential new laws or inconsitencies with
       * its current plan.
       *
       * @author Frederic Py
       * @ingroup europa
       */
      class TimePoint :public EUROPA::Variable<EUROPA::IntervalIntDomain> {
      public:
        /** @brief Constructor
         *
         * @param tok A EUROPA token
         * @param now Current T_REX tick date
         * @param name A symbolic name
         *
         * Create a new instance associated to the token @p tok and with a
         * value greater than @p now
         * If @p tok is a valid token it will call create_constraint to relate
         * this timepoint to @p tok `start`, `duration` and `end` variables.
         */
        TimePoint(EUROPA::TokenId const &tok,
                  EUROPA::eint now,
                  EUROPA::LabelStr const &name);
        /** @brief Destructor */
        virtual ~TimePoint();
        
        /** @brief Update current tick
         *
         * @param[in] now The current tick
         *
         * reduce the current domain of this variable by constraining it to be
         * greater than @p now
         *
         * @retval true if success
         * @retval false if this operaion resulted on an empty domain
         *
         * @sa relax_date()
         * @sa commit_date()
         */
        bool set_date(EUROPA::eint now);
        /** @brief relax domain
         *
         * This method allow to relax the current instance value to its base
         * domain. It is usually called after a failed attempt to restrict the
         * variable domain.
         *
         * @sa set_date(EUROPA::eint)
         */
        void relax_date();
        /** @brief Commit current tick date
         *
         * @param[in] now the current date
         *
         * This method attempt to commit the fact that this timepoint extend
         * beyond the tick @p now by restricting its /base/ domain to be greater
         * than @p now.
         *
         * @note As this operation as a definitive impact of the variable domain
         * it is often preferred to attempt this restriction first with `set_date`
         * and check it success before committing with this call
         *
         * @retval true if the restriction was valid and applied
         * @retval false this restriction would result on an inconsitency
         *
         * @sa set_date(EUROPA::eint)
         * @sa commit_end(EUROPA::eint)
         */
        bool commit_date(EUROPA::eint now);
        /** @brief Commit current tick date
         *
         * @param[in] now the current date
         *
         * This method attempt to commit the fact that this timepoint value is
         * exactly @p now
         *
         * @note As this operation as a definitive impact of the variable domain
         * it is often preferred to attempt this restriction first by checking
         * that it is possible to do so after full propagation of other timepoints
         * relative to @p now
         *
         * @retval true if the restriction was valid and applied
         * @retval false this restriction would result on an inconsitency
         *
         * @sa set_date(EUROPA::eint)
         * @sa commit_date(EUROPA::eint)
         */
        bool commit_end(EUROPA::eint now);
        
        /** @brief Change related token
         *
         * @param[in] tok A token
         *
         * Attatch this instance to the token @p tok. If the varaibale was already
         * associated to another token the ownership is now transferred to @p tok
         * along with the realted  constraints
         *
         * @note The main reason to this method is to ensure that the timepoint
         * is maintained event when the plan database is cleaned up. Indeed, it
         * can happen that the token which was associated to this timepoint has
         * been destroyed for the profit of one of its merged (or active) equivalent.
         * In such a case the timepoint is transferred o this new token.
         */
        void setToken(EUROPA::TokenId tok);
        /** @brief Associated token
         *
         * @return The token this timepoint is currently associated to
         */
        EUROPA::TokenId const &token() const {
          return m_tok;
        }
        /** @brief Release timepoint
         *
         * Notify this instance that it is no longer required. The call of this
         * method indicates that trex do not necessitate it to be maintained
         * anymore as a result this timepoint is released and can then be garbage
         * collected by EUROPA whenever the plan database do not refer to it
         * anymore.
         */
        void pending_discard();
        
      private:
        void notifyDiscarded(Entity const *entity);
        void updateBase(EUROPA::IntervalIntDomain const &dom);
        
        static EUROPA::IntervalIntDomain future(EUROPA::eint now);
        void create_constraint();
        void detach(bool cstr=true);
        
        EUROPA::ConstraintId m_dur_cstr, m_end_cstr;
        EUROPA::TokenId      m_tok;
        bool m_pending;
        
      }; // TREX::europa::details::TimePoint
      
    } // TREX::europa::details
  } // TREX::europa
} // TREX

#endif // H_trex_europa_TimePoint
