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

namespace TREX {
  namespace transaction {
    namespace details {
      
      class external_impl
      :boost::noncopyable, public ENABLE_SHARED_FROM_THIS<external_impl> {
      public:
        external_impl(SHARED_PTR<node_impl> cli,
                      SHARED_PTR<internal_impl> tl,
                      transaction_flags const &fl);
        ~external_impl() {}
        
        utils::symbol const &name() const {
          return m_timeline->name();
        }
        SHARED_PTR<graph_impl> graph() const;
        
        bool accept_goals() const;
        bool publish_plan() const;
        
        void on_synch(TICK date, boost::optional<Observation> o);
        void connect() {
          m_timeline->connect(shared_from_this());
        }
        
        void reset();
        
      private:
        SHARED_PTR<internal_impl>  m_timeline;
        WEAK_PTR<node_impl>      m_client;

        transaction_flags m_flags;
        
        external_impl() DELETED;
      }; // TREX::transaction::details::external_impl

    }
  }
}

#endif // H_trex_transaction_private_external_impl
