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
#ifndef H_trex_transaction_graph
# define H_trex_transaction_graph

# include "reactor_fwd.hh"
# include "bits/reactor_policies.hh"
# include "Tick.hh"

# include <trex/utils/log_manager.hh>

namespace trex {
  namespace transaction {
    
    class graph :boost::noncopyable {
    public:
      explicit graph(utils::symbol const &name);
      explicit graph(boost::property_tree::ptree &xml);
      virtual ~graph();
      
      utils::symbol const &name() const;
      
      utils::log_manager &manager() const;
      boost::asio::io_service &service() const;
      utils::log::stream syslog(utils::symbol const &kind) const;
      
      
      template<class Iter>
      size_t add_reactors(Iter from, Iter to);
      size_t add_reactors(boost::property_tree::ptree &xml) {
        return add_reactors(xml.begin(), xml.end());
      }
      bool add_reactor(boost::property_tree::ptree::value_type &xml);
      
      size_t count_reactors() const;
      
      WEAK_PTR<details::graph_impl> tracked() const {
        return m_impl;
      }
      
      void tick(TICK date);
      
    private:
      SHARED_PTR<details::graph_impl>                 m_impl;
      utils::singleton::use<details::reactor_factory> m_factory;
      
      bool add_reactor(SHARED_PTR<reactor> r);
      
      graph() DELETED;
    };
# define IN_trex_transaction_graph
#  include "bits/graph.tcc"
# undef IN_trex_transaction_graph
    
  }
}

#endif // H_trex_transaction_graph
