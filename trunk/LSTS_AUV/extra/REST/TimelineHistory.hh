/*********************************************************************
 * Software License Agreement (BSD License)
 * 
 *  Copyright (c) 2013, MBARI.
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
#ifndef H_trex_rest_TimelineHistory
# define H_trex_rest_TimelineHistory

# include "db_manager.hh"
# include "timeline_wrap.hh"

# include <boost/operators.hpp>

namespace TREX {
  namespace REST {
    
    class REST_reactor;
    
    class TimelineHistory :public transaction::graph::timelines_listener {
    public:
      TimelineHistory(REST_reactor &creator);
      ~TimelineHistory();
      
      void new_obs(transaction::Observation const &obs,
                   transaction::TICK cur);
      void update_tick(transaction::TICK cur);
      
      void list_timelines(std::ostream &out, std::set<std::string> const &select, bool hidden,
                          transaction::IntegerDomain const &range);
      
      transaction::TICK get_date(std::string const &date);
      void get_tokens(std::string const &timeline,
                      transaction::IntegerDomain::bound &lo,
                      transaction::IntegerDomain::bound const &hi,
                      std::ostream &dest, bool first,
                      size_t max);
      bool exists(std::string const &name);
      
      boost::property_tree::ptree get_goal(transaction::goal_id g) const;
      boost::property_tree::ptree goals();
      
      transaction::goal_id add_goal(std::string const &file);
      transaction::goal_id get_goal(std::string const &id);
      bool                 delete_goal(std::string const &id);
      
      bool fancy() const {
        return m_fancy;
      }
      transaction::TICK now() const {
        return m_cur;
      }
      
    private:
      boost::property_tree::ptree get_token(transaction::goal_id const &tok) const;
      unsigned long long count_tokens(helpers::timeline_wrap const &tl,
                                      transaction::IntegerDomain const &dom,
                                      transaction::TICK &delta_t);
      
      void declared(transaction::details::timeline const &timeline);
      
      // Bunch of internl calls that need to be thread protected
      void add_obs_sync(transaction::goal_id tok,
                        transaction::TICK date);
      void ext_obs_sync(transaction::TICK date);
      void add_tl_sync(transaction::details::timeline const &tl);
      size_t get_tok_sync(utils::Symbol tl,
                          transaction::IntegerDomain::bound &lo,
                          transaction::IntegerDomain::bound hi,
                          std::ostream &out, bool first, size_t max);
      bool exists_sync(utils::Symbol name);
            
      boost::property_tree::ptree goals_sync();
      
      void add_goal_sync(transaction::goal_id g);
      transaction::goal_id get_goal_sync(std::string id);
      transaction::goal_id del_goal_sync(std::string id);
      
      
      size_t list_tl_sync(std::ostream &out, std::set<std::string> const &select, bool hidden,
                          transaction::IntegerDomain rng);
      
      bool const m_fancy;
      
      transaction::TICK   m_cur;
      REST_reactor       &m_reactor;
      boost::asio::strand m_strand;
      
      helpers::db_manager          m_db;
      helpers::rest_tl_set         m_timelines;
      
      typedef std::map<std::string, transaction::goal_id> goal_map;
      goal_map m_goals;
    };
    
  }
}

#endif
