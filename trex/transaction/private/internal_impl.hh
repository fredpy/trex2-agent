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
#ifndef H_trex_transaction_private_internal_impl
# define H_trex_transaction_private_internal_impl

# include "../bits/transaction_fwd.hh"

# include <trex/domain/token.hh>

# include <boost/thread/shared_mutex.hpp>


namespace TREX {
  namespace transaction {
    namespace details {
      
      class internal_impl :boost::noncopyable,
      public ENABLE_SHARED_FROM_THIS<internal_impl> {
      public:
        typedef boost::signals2::signal<void(date_type, token_id)> synch_event;
        static utils::symbol const s_failed;
        
        internal_impl(SHARED_PTR<graph_impl> const &g,
                      utils::symbol const &name);
        ~internal_impl();
        
        utils::symbol const &name() const {
          return m_name;
        }
        SHARED_PTR<node_impl> owner() const {
          return m_owner.lock();
        }
        bool owned() const {
          return bool(owner());
        }
        bool accept_goals() const;
        bool publish_plan() const;
        date_type latency() const;
        date_type lookahead() const;
        
        boost::optional<date_type> now() const;
        boost::optional<date_type> last_synch(token_id &obs) const;
        boost::optional<date_type> last_synch() const {
          token_id ignore;
          return last_synch(ignore);
        }
        utils::log::stream syslog(utils::symbol const &kind) const;
        token_ref obs(utils::symbol const &pred) const;
        
        ERROR_CODE g_strand_post(token_ref obs);
        void synchronized(date_type tick);
        synch_event &on_synch() {
          return m_synch;
        }
        
        std::string access_str() const;
        
        ERROR_CODE g_strand_reset(SHARED_PTR<node_impl> const &r);
      private:
        WEAK_PTR<graph_impl> const m_graph;
        WEAK_PTR<node_impl>        m_owner;
        utils::symbol        const m_name;
        
        mutable boost::shared_mutex m_mtx;
        boost::optional<date_type> m_synch_date;
        token_ref         m_last_obs, m_next_obs;
        synch_event       m_synch;
        transaction_flags m_gp;
        
        // graph strand calls
        ERROR_CODE g_strand_set_owner(SHARED_PTR<node_impl> const &r,
                                      transaction_flags const &gp);
        void g_strand_tick(boost::signals2::connection const &c,
                           date_type tick);
        void g_strand_synch(date_type date);
        
        internal_impl() DELETED;
        
        friend class graph_impl;
      };

      template<class Iter>
      Iter name_lower_bound(Iter from, Iter const to,
                            utils::symbol const &n) {
        for( ; to!=from && (*from)->name()<n; ++from);
        return from;
      }
      
    } // TREX::transaction::details
  } // TREX::transaction
} // TREX

#endif // H_trex_transaction_private_internal_impl
