/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, Frederic Py.
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
#ifndef H_trex_transaction_private_external_impl
# define H_trex_transaction_private_external_impl

# include "internal_impl.hh"

namespace trex {
  namespace transaction {
    namespace details {
      
      class external_impl :boost::noncopyable,
      public ENABLE_SHARED_FROM_THIS<external_impl> {
      public:
        external_impl(SHARED_PTR<node_impl>     const &cli,
                      SHARED_PTR<internal_impl> const &tl,
                      transaction_flags gp);
        ~external_impl();
        
        void g_strand_connect();
        
        utils::symbol const &name() const {
          return m_timeline->name();
        }
        bool accept_goals() const;
        bool publish_plan() const;
        date_type latency() const {
          return m_timeline->latency();
        }
        date_type lookahead() const {
          return m_timeline->lookahead();
        }
        boost::optional<date_type> now() const {
          return m_timeline->now();
        }
        boost::optional<date_type> last_synch() const {
          return m_timeline->last_synch();
        }
        utils::log::stream syslog(utils::symbol const &kind) const;
        token_ref goal(utils::symbol const &pred) const;
        
      private:
        WEAK_PTR<node_impl>       m_client;
        SHARED_PTR<internal_impl> const m_timeline;
        
        mutable boost::shared_mutex m_mtx;
        transaction_flags           m_gp;
        boost::optional<date_type>  m_last_tick;
        token_id                    m_state;
        
        void g_strand_obs(boost::signals2::connection const &c,
                          date_type tick, token_id obs);
        
      }; // TREX::transaction::details::external_impl
      
      
    } // TREX::transaction::details
  } // TREX::transaction
} // TREX

#endif // H_trex_transaction_private_external_impl
