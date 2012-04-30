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
#ifndef H_trex_EuropaReactor
# define H_trex_EuropaReactor

# include <trex/europa/Assembly.hh>

# include <trex/transaction/TeleoReactor.hh>

# include <boost/bimap.hpp>

namespace TREX {
  namespace europa {

    class EuropaReactor:public TREX::transaction::TeleoReactor, protected Assembly {
    public:
      explicit EuropaReactor(TREX::transaction::TeleoReactor::xml_arg_type arg);
      ~EuropaReactor();

    protected:
      // TREX transaction callbacks
      void notify(TREX::transaction::Observation const &obs);
      void handleRequest(TREX::transaction::goal_id const &request);
      void handleRecall(TREX::transaction::goal_id const &request);

      void newPlanToken(TREX::transaction::goal_id const &t);
      void cancelledPlanToken(TREX::transaction::goal_id const &t);
      
      // TREX execution callbacks
      bool hasWork();

      void handleInit();
      void handleTickStart();
      bool synchronize();
      void resume();

    private:
      bool discard(EUROPA::TokenId const &tok);
      void cancel(EUROPA::TokenId const &tok);
      bool dispatch(EUROPA::TimelineId const &tl,
                    EUROPA::TokenId const &tok);

      void plan_dispatch(EUROPA::TimelineId const &tl,
                         EUROPA::TokenId const &tok);

      bool restrict_token(EUROPA::TokenId &tok,
			  TREX::transaction::Predicate const &pred);

      bool is_internal(EUROPA::LabelStr const &name) const {
	return isInternal(TREX::utils::Symbol(name.c_str()));
      }
      bool is_external(EUROPA::LabelStr const &name) const {
	return isExternal(TREX::utils::Symbol(name.c_str()));
      }

      bool do_relax(bool full);

      EUROPA::eint now() const {
	return getCurrentTick();
      }
      EUROPA::eint latency() const {
        return getExecLatency();
      }
      EUROPA::eint look_ahead() const {
        return getLookAhead();
      }
      EUROPA::IntervalIntDomain plan_scope() const;
      EUROPA::eint initial_tick() const {
	return getInitialTick();
      }
      EUROPA::eint final_tick() const {
	return getFinalTick();
      }
      EUROPA::edouble tick_duration() const {
	return tickDuration();
      }
      void notify(EUROPA::LabelStr const &object, EUROPA::TokenId const &obs);

      void logPlan(std::string const &base_name) const;

      typedef boost::bimap<EUROPA::eint, TREX::transaction::goal_id> goal_map;
      goal_map m_active_requests;
      goal_map m_dispatched;
      goal_map m_plan_tokens;

      bool m_completed_this_tick;
    }; // TREX::europa::EuropaReactor

  } // TREX::europa
} // TREX

#endif // H_trex_EuropaReactor
