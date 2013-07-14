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

# include "timeline_impl.hh"

# include <map>

namespace TREX {
  namespace transaction {
    namespace details {

      /** @brief Transaction node internal implementation
       *
       * This class implements tha basic transaction scheme between a node
       * (typically a reactor) and the rest of the agent graph. A transaction
       * node is described by :
       *  @li its unique id (its address in memory)
       *  @li a name identifier
       *  @li the connections it uses
       *  @li the connection it provides
       * The connections are the timelines that connects node one to another.
       * This class sepcifically does manage the handling of connections along 
       * with the message passing through these. It is a rpivate implementation 
       * that should not accessed directly.
       *
       * @note Most of the calls in this implemntaion are handled asynchronously
       *
       * @ingroup transaction 
       * @author Frederic Py
       */
      class node_impl :boost::noncopyable,
      public ENABLE_SHARED_FROM_THIS<node_impl> {
      public:
        /** @brief Destructor 
         */
        ~node_impl();

        /** @brief node id
         *
         * @return the id of this node
         */
        node_id id() {
          return shared_from_this();
        }
        
        /** @brief change node name
         * 
         * @param[in] name A name
         *
         * Set this node name to @p name
         */
        void set_name(utils::Symbol const &name) {
          m_name = name;
        }
        /** @brief Get node name
         *
         * @return The name of the node
         */
        utils::Symbol const &name() const {
          return m_name;
        }
        
        /** @brief Log entry
         * @param[in] ctx A context information 
         * @param[in] kind A message type
         *
         * Create a new log entry ssociated to this node with the context 
         * @p ctx and message type @p kind. If a date is available the message 
         * will also be dated accordingly.
         *
         * @return A temporary stream to receive the message
         */
        utils::log::stream syslog(utils::Symbol const &ctx,
                                  utils::Symbol const &kind) const;
        /** @brief Log manager
         *
         * @return The log manager associated to this node
         */
        utils::LogManager &manager() const;
        
        /** @brief Supporting graph
         *
         * Give access to the graph this node is part of
         *
         * @return A pointer to the graph manageing to this node. 
         *
         * @note when a node has been isolted from iuts managing 
         * graph the returned value is then a null pointer
         */
        SHARED_PTR<graph_impl> graph() const {
          return m_graph.lock();
        }
        
        /** @brief Check if internal
         *
         * @param[in] tl A connection name
         *
         * Tests if the connection named @p tl is internal to this node.
         * An internal connection means that it is managed by this node.
         *
         * @retval true if @p tl is internal to this node 
         * @retval false otherwise 
         * 
         * @note This call will block until all pending transaction 
         * operations posted so far are completed.
         *
         * @sa provide
         * @sa external
         */
        bool internal(utils::Symbol const &tl) const;
        /** @brief Check if external
         *
         * @param[in] tl A connection name
         *
         * Tests if the connection named @p tl is external to this node.
         * An external connection means that the node is client of the 
         * connection which is managed by another entity.
         *
         * @retval true if @p tl is external to this node
         * @retval false otherwise
         *
         * @note This call will block until all pending transaction
         * operations posted so far are completed.
         *
         * @sa use
         * @sa internal
         */
        bool external(utils::Symbol const &tl) const;
        
        /** @brief Request connection ownership
         *
         * @param[in] tl           A connection name
         * @param[in] read_only    External access flag
         * @param[in] publish_plan Plan publication flag
         *
         * This call allow this node to request for taking ownership of the
         * timeline @p tl. should the request succeed This timeline will accept 
         * to refuse goals or not depending on @p read_only and will promise to
         * make its planned evoluytion readable accordingly to @p publish_plan
         *
         * @post The request for this node to decalre @p tl as internal has been 
         * posted 
         */
        void provide(utils::Symbol const &tl, bool read_only, bool publish_plan);
        /** @brief Cancel ownership of a connection
         *
         * @param[in] tl A connection name
         *
         * Notifies the graph that this node no longers owns the timeline @p tl.
         * This method will have an effect only if @p tl was currently internal
         * to this node. It eventual effect will be that @p tl is no longer 
         * owned by this reactor and available for other node who would like 
         * to take its ownership
         *
         * @post The notification has been sent.
         */
        void unprovide(utils::Symbol const &tl);
  
        
        /** @brief Request client connection 
         *
         * @param[in] tl           A connection name
         * @param[in] read_only    External access flag
         * @param[in] publish_plan Plan publication flag
         *
         * This call allow this node to request for baing client of the
         * timeline @p tl. should the request succeed this client connection 
         * will attemtp to post goals or not depending on @p read_only and 
         * will listen to potential plans accordingly to @p publish_plan
         *
         * @post The request for this node to declare @p tl as external has been
         * posted
         */
        void use(utils::Symbol const &tl, bool read_only, bool listen_plan);
        /** @brief Cancel client connection
         *
         * @param[in] tl A connection name
         *
         * Notifies the graph that this node is no longer client of the timeline
         * @p tl.
         * This method will have an effect only if @p tl was currently external
         * to this node. Its eventual effect will be that @p tl is no longer
         * used by this reactor 
         *
         * @post The notification has been sent.
         */
        void unuse(utils::Symbol const &tl);
        
        Observation obs(utils::Symbol const &name, utils::Symbol const &pred);
        void post_observation(Observation const &obs, bool echo = false);
        
        void synchronize(TICK date);
        
      private:
        /** @brief Constructor 
         *
         * @param[in] g A weak reference to the calling graph
         *
         * Create A new instance associted to the graph @p g. 
         *
         * @note this call is only made internally from graph_impl class
         */
        explicit node_impl(WEAK_PTR<graph_impl> const &g);
        
        /** @brief isolarte the node
         *
         * @param[in] g A weak reference to a graph
         *
         * @pre g.lock()==this->graph()
         *
         * Isolate this node within @p g. Isolation implies that this node 
         * will schedule the disconnection from both its internal and 
         * external timeline and will no longer attched to its graph which 
         * should then eventually result on the instance being destroyed
         *
         * @post !this->graph()
         */
        void isolate(SHARED_PTR<graph_impl> const &g);
        
        /** @brief Notification of assignement
         *
         * @param[in] tl An internal connection descriptor
         *
         * This method is called by the graph managing this node to notify 
         * the instance that its previous call to provide has sucessfully
         * completed.
         * It results on the node settign the timeline @p tl as internal
         *
         * @sa provide
         * @sa internal
         */
        void assigned(tl_ref tl);
        /** @brief Notification of subscription
         *
         * @param[in] tl An external connection descriptor
         *
         * This method is called by the graph managing this node to notify
         * the instance that its previous call to use has sucessfully
         * completed.
         * It results on the node setting the timeline @p tl as external
         *
         * @sa use
         * @sa external
         */
        void subscribed(ext_ref tl);
        
        /** @brief Name of the node 
         */
        utils::Symbol        m_name;
        /** @brief Graph associated to the node 
         *
         * A weak reference to the graph managing this node. The validity 
         * of this reference indicate the relvance of the node.
         */
        WEAK_PTR<graph_impl> m_graph;
        
        typedef std::map<utils::Symbol, tl_ref> internal_map;
        typedef std::map<utils::Symbol, ext_ref> external_map;

        /** @brief internal connections of the node
         *
         * A set storing all the internal connection of this node.
         *
         * @note The content of this set is relevant only if m_graph is valid
         */
        internal_map m_internals;
        /** @brief external connections of the node
         *
         * A set storing all the external connection of this node.
         *
         * @note The content of this set is relevant only if m_graph is valid
         */
        external_map m_externals;
        
        /** @brief Disown timeline
         *
         * @param[in] g The calling graph
         * @param[in] tl A connection name
         *
         * @pre g==this->graph() || !this->graph()
         *
         * Notifies the node that it no longer own @p tl.This method is 
         * called by @p g
         * 
         * @post !internal(tl)
         */ 
        void unprovide_sync(SHARED_PTR<graph_impl> g, utils::Symbol tl);
        /** @brief Dsiconnect from timeline
         *
         * @param[in] g The calling graph
         * @param[in] tl A connection name
         *
         * @pre g==this->graph() || !this->graph()
         *
         * Notifies the node that it no longer is subscribed to @p tl.
         * This method is called by @p g
         *
         * @post !external(tl)
         */
        void unuse_sync(utils::Symbol tl);
        
        Observation obs_sync(utils::Symbol name, utils::Symbol pred);
        void post_obs_sync(Observation o, bool echo);
        void synch_sync(TICK date);

        
        /** @brief Check for ownership
         *
         * @param[in] name A timeline name 
         *
         * Called asynchronously by internal to check if @p name is internal
         *
         * @retval true if @p name is internal to this node 
         * @retval false otherwise
         *
         * @note this call is not thread safe and should not be called directly 
         * @sa internal
         */
        bool check_internal_sync(utils::Symbol name) const;
        /** @brief Check for connection
         *
         * @param[in] name A timeline name
         *
         * Called asynchronously by external to check if @p name is external
         *
         * @retval true if @p name is external to this node
         * @retval false otherwise
         *
         * @note this call is not thread safe and should not be called directly
         * @sa external
         */
        bool check_external_sync(utils::Symbol name) const;
        
        friend class graph_impl;

        node_impl() DELETED;
      }; // TREX::transaction::details::node_impl
      
    } // TREX::transaction::details
  } // TREX::transaction
} // TREX

#endif // H_trex_transaction_node_impl