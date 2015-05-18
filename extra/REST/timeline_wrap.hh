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
#ifndef H_trex_rest_timeline_wrap
# define H_trex_rest_timeline_wrap

# include <trex/transaction/TeleoReactor.hh>

namespace TREX {
  namespace REST {
    namespace helpers {
      
      class timeline_wrap {
      public:
        typedef timeline_wrap base_type;
        typedef transaction::details::timeline::id_type id_type;
        
        static id_type const &get_id(base_type const &tl) {
          return tl.name();
        }
        
        timeline_wrap(transaction::details::timeline const &tl):m_tl(tl),m_count(0) {}
        ~timeline_wrap() {}
        
        utils::Symbol const &name() const {
          return m_tl.name();
        }
        
        bool alive() const {
          return m_tl.owned();
        }
        bool accept_goals() const {
          return look_ahead()>0;
        }
        bool publish_plan() const {
          return m_tl.publish_plan();
        }
        
        transaction::TICK latency()    const {
          return m_tl.latency();
        }
        transaction::TICK look_ahead() const {
          return m_tl.look_ahead();
        }
        
        std::pair<transaction::TICK, transaction::goal_id>
        new_obs(transaction::TICK cur, transaction::goal_id tok) {
          std::pair<transaction::TICK, transaction::goal_id> ret(m_date, m_obs);
          m_date = cur;
          if( 0==m_count )
            m_initial = m_date;
          m_obs = tok;
          ++m_count;
          return ret;
        }
        
        bool has_observation() const {
          return bool(m_obs);
        }
        transaction::TICK obs_date() const {
          return m_date;
        }
        transaction::TICK initial() const {
          return m_initial;
        }
        
        transaction::goal_id obs() const {
          return m_obs;
        }
        unsigned long long count() const {
          return m_count;
        }
        
        
      private:
        transaction::details::timeline const &m_tl;
        
        transaction::TICK    m_initial, m_date;
        transaction::goal_id m_obs;
        unsigned long long   m_count;
      };
      
      typedef utils::pointer_id_traits<timeline_wrap> tw_ptr_id_traits;
      typedef utils::list_set<tw_ptr_id_traits>       rest_tl_set;

    }
  }
}


#endif // H_trex_rest_timeline_wrap
