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
# include <trex/domain/token.hh>

# include <boost/noncopyable.hpp>
# include <boost/thread/shared_mutex.hpp>


# include "../Tick.hh"

namespace TREX {
  namespace transaction {
    
    class reactor;
    
    namespace details {

      /** @brief transaction graph node
       *
       * This class manages all transactions of the reactor
       * with the rest of the agent graph.
       * It manages the conections to timelines (whether @e internal 
       * or @e external) along with the management of time as provided 
       * by the agent. 
       *
       * Any reactor is associated to a node which is basically 
       * its connection with the rest of the agent.
       *
       * @note This class is for internal use and is not directly 
       *   exposed in T_REX API.
       *
       * @author Frederic Py
       */
      class node_impl :boost::noncopyable,
      public ENABLE_SHARED_FROM_THIS<node_impl> {
      public:
        /** @brief Constructor
         *
         * @param[in] g A graph
         *
         * Create a new node associated to graph @p g.
         * @note This constructor is not meant to be called outside
         * of the graph class and is public only for unit testing
         * purposes
         */
        explicit node_impl(WEAK_PTR<graph_impl> const &g);
        /** @brief Destructor */
        ~node_impl();
        
        /** @brief Set node name
         *
         * @param[in] name A symbolic name
         *
         * Set the name of this node to @p name.
         *
         * @sa node_impl::name() const
         */
        void set_name(utils::symbol const &name);
        /** @brief Get node name
         *
         * Gets the symbolic name of this node.
         *
         * @note The name of a node is not an absolute idenitfier
         *   for this node. Indeed you can have potentially multiple
         *   nodes with the same name. This differ from previous
         *   versions of TREX (0.5.x or anterior)
         *
         * @return The name of this node
         * @sa node_impl::set_name(TREX::utils::symbol const &)
         */
        utils::symbol const &name() const;
        
        boost::optional<TICK> internal_date() const;
        boost::optional<TICK> external_date() const;
        boost::optional<TICK> current_tick() const;
        
        void synchronized(TICK date);
        
        /** @brief Detach node from graph
         *
         * Detach this node from its current transaction graph.
         * This operation result on the node no longer being
         * receiving or able to post transactions and often
         * immediately precedes the node destruction.
         */
        void reset();
        /** @brief Tick notification
         *
         * @param[in] c The connection that emitted this signal
         * @param[in] date A tick date
         *
         * This method is called whenever a new tick update
         * is emitted. It updates the current tick date to
         * @p date for this node.
         */
        void tick(boost::signals2::connection const &c,
                  date_type const &date);
        
        utils::log_manager &manager() const;
        
        /** @brief New log entry
         *
         * @param[in] ctx  Message source context
         * @param[in] kind Message type
         *
         * Create a new log entry for this node.
         *
         * @return A stream to receive the entry content
         */
        utils::log::stream syslog(utils::symbol const &ctx,
                                  utils::symbol const &kind) const;
        /** @brief Node's graph
         *
         * Give the graph associated to this node if any
         *
         * @retval null if the node is no longer associated to a graph
         * @retval a pointer to the graph otherwise
         *
         * @sa node_impl::reset()
         */
        SHARED_PTR<graph_impl> graph() const;
        /** @brief Observation update signal
         *
         * @param[in] date A tick date
         * @param[in] tl   A timeline name
         * @param[in] o    An optional observation
         *
         * @pre tl is @e external to this node
         * @pre if o is an observation then o->object()==tl
         *
         * This method is called through a signal whenever the
         * timeline @p tl have resolved its state for the tick
         * @p date. If @p observation is empty then the previous
         * observation extends other this tick, otherwise @p *o
         * is the new state staring at tick @p date
         */
        void notify(TICK date, utils::symbol const &tl,
                    transaction::token_id const &o);
        
        bool use(utils::symbol const &tl, transaction_flags fl);
        bool provide(utils::symbol const &tl, transaction_flags fl);
                
      private:
        mutable boost::shared_mutex  m_mutex;
        boost::optional<TICK> m_internal_date,
                              m_external_date,
                              m_execution_frontier;
        
        WEAK_PTR<graph_impl> m_graph;
        utils::symbol        m_name;
      }; // node_impl
      
    } // TREX::transaction::details
  } // TREX::transaction
} // TREX

#endif // H_trex_transaction_node_impl
