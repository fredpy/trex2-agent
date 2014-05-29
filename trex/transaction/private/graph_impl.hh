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
#ifndef H_trex_transaction_graph_impl
# define H_trex_transaction_graph_impl

# include "../bits/transaction_fwd.hh"

# include <trex/utils/log_manager.hh>
# include <boost/thread/shared_mutex.hpp>
# include <boost/signals2/signal.hpp>


namespace TREX {
  namespace transaction {
    namespace details {
      
      
      /** @brief Transaction graph
       *
       * This class implement the graph used by T-REX to manage 
       * transaction between each reactors through timelines.
       *
       * It handle the creation and lifetime of a reactor along 
       * with its connection with other reactors through 
       * timelines. This is the new implementation with a more 
       * asynchronous approach than what was doe up to T-REX v 
       * 0.5.x
       *
       * @note This implementation is private and is not meant to 
       * be used directly outiside of libTREXtransaction.
       *
       * @author Frederic Py <fredpy@gmail.com>
       */
      class graph_impl :boost::noncopyable,
      public ENABLE_SHARED_FROM_THIS<graph_impl> {
      public:
        /** @brief Default constructor
         *
         * Create a new nameless graph.
         */
        graph_impl();
        /** @brief Constructor
         *
         * @param[in] name A name identifier
         *
         * Create a new graph named @p name
         */
        explicit graph_impl(utils::symbol const &name);
        /** @brief Destructor 
         */
        ~graph_impl();
        
        /** @brief Set graph name
         *
         * @param[in] name A name
         *
         * Set this graph's name to @p name
         *
         * @return The updated name of the graph
         *
         * @sa graph_impl::name() const
         */
        utils::symbol const &set_name(utils::symbol const &name);
        /** @brief Get graph name
         *
         * @return The name of the graph
         *
         * @sa graph_impl::set_name(utils::symbol const &)
         */
        utils::symbol const &name() const;
        
        /** @brief Get log manager
         *
         * Give acces to the log_manager instance for this graph
         *
         * @return The log_manager for this graph
         *
         * @note The class log_manager being tha singleton this 
         * instance is the same for any existing graphs within 
         * the same program.
         */
        utils::log_manager &manager() const;
        
        /** @rbief Update current tick
         *
         * @param[in] date A tick value
         *
         * Notify the class that the new desired tick value is 
         * @p date. This call just scheudlle the update of the 
         * date and is not guaranteed to be immediately effective 
         * after completion. 
         *
         * Indeed, the update is just schedulled asynchronously and
         * will happen only when the thread schedulling can allow it. 
         * Additionally, the update of the clock can occur if and 
         * only if @p date is not less or equal to the previous tick 
         * date
         *
         * @sa date(bool) const
         */
        void set_date(date_type const &date);
        /** Get current tick date
         *
         * @param fast fast access flag
         *
         * Gather the current tick date. The @p fast flag indicate 
         * whether this access should wait for all previous updates 
         * to be done or just directly access the current date value 
         * with faster but less accurate result.
         *
         * For example the @c syslog method gather the date ussettinc @p fast
         * to @c true. As the date for logging is not necessarily critical
         * and we do not want the logging to block the rest of the execution.
         *
         * @return The current tick date if any
         *
         * @sa set_date(date_type const &)
         */
        boost::optional<date_type> date(bool fast=false) const;
        
        /** @brief Create new log entry
         *
         * @param[in] ctx  Log context (or source)
         * @param[in] kind Log message type
         *
         * Create a new log entry to be logged to TREX.log
         *
         * @return The stream for the new log entry
         */
        utils::log::stream syslog(utils::symbol const &ctx,
                                  utils::symbol const &kind) const;
        
        /** @brief Create new node
         *
         * @param[in] desired_name Node name
         *
         * Create and add an  new node to this graph named @p name.
         *
         * A node is the connection of a reactor to its graph and handle 
         * all the transaction of this reactor. This include: 
         * @li its internal timelines
         * @li its external timelines
         * @li the current tick date
         * @li all transaction (ie observations, goal request/recalls, ...)
         * with other nodes
         *
         * @return A reference to the newly created node
         *
         * @note while up to version 0.5.x T-REX did not allow for 
         *       multiple reactors with the same name, this is not
         *       the case anymore: at the current stage the graph 
         *       will allow multiples reactors with same name and 
         *       we plan for the future to dynamically rename 
         *       duplicate names.
         */
        WEAK_PTR<node_impl> add_node(utils::symbol const &desired_name);
        /** @brtief Remove a node for the graph
         *
         * @param[in] n A node
         *
         * Remove @p n for this graph and notifes it that no longer 
         * belongs to a graph.
         */
        void rm_node(WEAK_PTR<node_impl> n);
        
        /** @brief Transaction pseudo thread
         *
         * The execution thread used to manage transaction operation 
         * for this graph. This is implemented as a boost.asio strand 
         * to which one can post new tasks that will be executed in 
         * sequence as they are posted.
         *
         * This thread is used for all transactions operations of this 
         * graph  which include:
         * @li Update of the graph structure 
         * @li Communication through edges (observations, goal requests/recalls)
         * @li Tick updates from the clock
         *
         * @return The strand used by this graph for transaction operation
         */
        boost::asio::strand &strand() const {
          return *m_strand;
        }
        
      private:
        utils::singleton::use<utils::log_manager> m_mgr;

        typedef boost::shared_mutex mutex_type;
        
        mutable mutex_type          m_mutex;
        utils::symbol               m_name;
        boost::optional<date_type>  m_date;
        
        tick_sig m_tick;
        
        std::set< SHARED_PTR<node_impl> > m_nodes;
        
        // async management
        UNIQ_PTR<boost::asio::strand> m_strand;
        
        void set_date_sync(date_type d);
        void notify_date(date_type d);
        void add_node_sync(SHARED_PTR<node_impl> node,
                           utils::symbol desired_name);
        void rm_node_sync(SHARED_PTR<node_impl> node);
        
      }; // TREX::transaction::details::graph_impl
      
    } // TREX::transaction::details
  } // TREX::transaction
} // TREX

#endif // H_trex_transaction_graph_impl
