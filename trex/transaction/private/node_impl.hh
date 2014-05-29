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
#ifndef H_trex_transaction_node_impl
# define H_trex_transaction_node_impl

# include "../bits/transaction_fwd.hh"
# include <trex/utils/symbol.hh>
# include <boost/noncopyable.hpp>

# include "../Observation.hh"
# include "../Tick.hh"

namespace TREX {
  namespace transaction {
    namespace details {

      class node_impl :boost::noncopyable,
      public ENABLE_SHARED_FROM_THIS<node_impl> {
      public:
        node_impl(WEAK_PTR<graph_impl> const &g);
        ~node_impl();

        void set_name(utils::symbol const &name);
        utils::symbol const &name() const;
        
        void reset();
        void tick(boost::signals2::connection const &c,
                  date_type const &date);
        
        utils::log::stream syslog(utils::symbol const &ctx,
                                  utils::symbol const &kind) const;
        SHARED_PTR<graph_impl> graph() const;
        void notify(TICK date, utils::symbol const &tl,
                    boost::optional<Observation> const &o);
        
        void removed();
        
     private:
        WEAK_PTR<graph_impl> m_graph;
        utils::symbol        m_name;
      
      }; // node_impl
      
    } // TREX::transaction::details
  } // TREX::transaction
} // TREX

#endif // H_trex_transaction_node_impl
