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

# include <trex/transaction/reactor.hh>
# include <trex/utils/asio_fstream.hh>


# include <boost/bimap.hpp>

namespace TREX {
  namespace europa {

    /** @brief Europa reactor
     *
     * This class implement the europa based reactor. This reactor interfaces between 
     * T-REX and europa through its Assembly mother class in order to plan execute 
     * actions in the agent using the Europa solver.
     *
     * @author Frederic Py <fpy@mbari.org>
     * @ingroup europa
     */
    class EuropaReactor:public TREX::transaction::reactor, protected Assembly {
    public:
      /** @brief XML constructor
       *
       * @param[in] arg XML informations
       *
       * Create a new instance using the information provide by @p arg
       * The XML format for this reactor is as follow :
       * @code
       *  <EuropaReactor name="<name>" latency="<int>" lookahead="<int>"
       *                 solverConfig="<cfg-file>" />
       * @endcode 
       *
       * The reactor will then load a nddl model named @c <agent-name>.<name>.nddl
       * -- or @c <name>.nddl if the former fle does not exist -- and configure its 
       * solvers using <cfg-file> as the basis. 
       *
       * Optionally one can specify the model file to be used if he does not want 
       * to used the same name for the reactor as the name of the model file. 
       * Then the xml will be as follow :
       * @code
       *  <EuropaReactor name="<name>" latency="<int>" lookahead="<int>"
       *                 solverConfig="<cfg-file>" model="<nddl-file>" />
       * @endcode 
       *
       * @pre <cfg-file> is a valid XML europa solver configuration file
       * @pre the specified or deduced nddl file name exists and is a valid ndddl file
       *
       * @throw TREX::utils::XmlError An error occured while trying to pars the XML definition of this reactor
       * @throw TREX::transaction::ReactorException An error occured while trying to intialize this reactor
       * @throw EuropaException europa related error while trying to load the model or solver configuration
       */
      explicit EuropaReactor(TREX::transaction::reactor::xml_arg_type arg);
      /** @brief Destructor */
      ~EuropaReactor();

    protected:
      // TREX transaction callbacks
      void notify(TREX::transaction::token const &obs);
      void handle_request(TREX::transaction::token_id const &request);
      void handle_recall(TREX::transaction::token_id const &request);

      void new_plan_token(TREX::transaction::token_id const &t);
      void cancelled_plan_token(TREX::transaction::token_id const &t);

      // TREX execution callbacks
      bool has_work();

      void handle_init();
      void handle_tick_start();
      bool synchronize();
      void resume();

    private:
      bool discard(EUROPA::TokenId const &tok);
      void cancel(EUROPA::TokenId const &tok);
      void rejected(EUROPA::TokenId const &tok);
      bool dispatch(EUROPA::TimelineId const &tl,
                    EUROPA::TokenId const &tok);

      void plan_dispatch(EUROPA::TimelineId const &tl,
                         EUROPA::TokenId const &tok);

      void restrict_goal(TREX::transaction::token& goal,
                          EUROPA::TokenId const &tok);

      bool restrict_token(EUROPA::TokenId &tok,
			  TREX::transaction::token const &pred);

      bool check_internal(EUROPA::LabelStr const &tl) const {
	return is_internal(TREX::utils::symbol(tl.c_str()));
      }
      bool check_external(EUROPA::LabelStr const &tl) const {
	return is_external(TREX::utils::symbol(tl.c_str()));
      }
      size_t tl_look_ahead(EUROPA::LabelStr const &name);

      bool do_relax(bool full);
      bool synch();

      EUROPA::eint now() const {
	return static_cast<EUROPA::eint::basis_type>(current_tick());
      }
      EUROPA::eint eu_latency() const {
        return static_cast<EUROPA::eint::basis_type>(exec_latency());
      }
      EUROPA::eint eu_look_ahead() const {
        return static_cast<EUROPA::eint::basis_type>(look_ahead());
      }
      EUROPA::edouble eu_tick_to_date(EUROPA::eint tick) const;
      EUROPA::eint eu_date_to_tick(EUROPA::edouble date) const;



      EUROPA::IntervalIntDomain plan_scope() const;
      EUROPA::eint eu_initial_tick() const {
	return static_cast<EUROPA::eint::basis_type>(initial_tick());
      }
      EUROPA::eint eu_final_tick() const {
	return static_cast<EUROPA::eint::basis_type>(final_tick());
      }
      EUROPA::edouble eu_tick_duration() const {
	return CHRONO::duration<EUROPA::edouble::basis_type>(tick_duration()).count();
      }
      void notify(EUROPA::LabelStr const &object, EUROPA::TokenId const &obs);

      void logPlan(std::string const &base_name) const;

      typedef boost::bimap<EUROPA::eint, TREX::transaction::token_id> goal_map;
      goal_map m_active_requests;
      goal_map m_dispatched;
      goal_map m_plan_tokens;

      bool m_completed_this_tick;
      EUROPA::eint m_last_complete;
      
      void print_stats(std::string const &what, size_t steps, size_t depth,
		       stat_clock::duration const &dur);
      utils::async_ofstream m_stats;
      bool m_old_plan_style, m_full_log;
      mutable size_t m_plan_counter;
    }; // TREX::europa::EuropaReactor

  } // TREX::europa
  
  namespace transaction {
    
    template<>
    struct exec_policy<europa::EuropaReactor> :public class_scope_exec<europa::EuropaReactor> {};
    
    
  } // TREX::transaction
} // TREX

#endif // H_trex_EuropaReactor
