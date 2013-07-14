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
#ifndef H_trex_transaction_timeline_impl
# define H_trex_transaction_timeline_impl

# include "../bits/transaction_fwd.hh"
# include "../Observation.hh"
# include "../Goal.hh"

namespace TREX {
  namespace transaction {
    namespace details {
      
      class external_impl;
      
      typedef SHARED_PTR<external_impl> ext_ref;
      
      /** @brief Internal timeline implementation
       *
       * This class provides the basic implementation for an internal timeline
       * and its transactions management.
       *
       * It is a private implementastion that is used internally by graph_impl
       * and node_impl in order to mananege the connection between reactors 
       * and their communication.
       *
       * @author Frederic Py
       */
      class internal_impl :boost::noncopyable, public ENABLE_SHARED_FROM_THIS<internal_impl> {
      public:
        /** @brief Constructor
         *
         * @param[in] name The timeline name
         * @param[in] g The graph managing this timeline 
         *
         * This constructor is called internally by graph_impl when a new 
         * timeline needs to be created
         */
        internal_impl(utils::Symbol const &name,
                      WEAK_PTR<graph_impl> const &g);
        /** @brief Destructor */
        ~internal_impl();
        
        /** @brief Timeline name
         *
         * Give the nasme of the timeline. Within a graph this name is 
         * unique and can be used to identify the timeline
         *
         * @return the name of the timeline
         */
        utils::Symbol const &name() const {
          return m_name;
        }
        
        /** @brief Owner of the timeline
         *
         * This function identifyies the node that currently maintain this
         * timeline as internal if any.
         *
         * @return A weak reference to the owner fo the timeline or an 
         * null reference if it has no owner
         *
         * @note this call will block until all the pending operations 
         * for this timleine are completed.
         */
        node_id owner() const;
        
        /** @brief Check for goal access
         *
         * This method checks if this timleine currently accept goals 
         * or not.
         *
         * @retval true if the timeline accept goals
         * @retval false otherwise
         *
         * @note This operation will block until pending operations of this
         * timeline are completed.
         */  
        bool accept_goals() const;
        /** @brief Check for plan publication
         *
         * Check if this timelien currently promises to publish its plan or 
         * not
         *
         * @retval true if the timleien currently promises to publish its plan
         * @retval false otherwise
         *
         * @note This call will blciak until all the pending operations on 
         * this timeline are completed
         */
        bool publish_plan() const;
        
        /** @brief Get timeline managing graph
         *
         * @return A pointer to the grtaph managing this timeline or a null
         * pointer if the managing graph has already been destroyed
         */
        SHARED_PTR<graph_impl> graph() const {
          return m_graph.lock();
        }
        
        TICK synch_date() const;
        
        void post_observation(Observation const &obs, bool echo=false);
        void synchronize(TICK date);
        
        Observation obs(utils::Symbol const &pred);
        
      private:
        /** @rbief timeline name */
        utils::Symbol        m_name;
        /** @brief weak reference to the managing graph */
        WEAK_PTR<graph_impl> m_graph;
        
        /** @brief transaction flags
         *
         * This maintin the transaction access rights to this timeline
         * This flags define 2 binary attributes 
         * @li Goal acceptance the timelien will accept to receive goals
         * from other nodes only if this flag is true
         * @li Plan publication the timelien won't publish any potential 
         * tokens in the future when this tag is false
         */
        transaction_flags           m_flags;
        /** @brief owner
         * 
         * A weak reference to the owner of this timeline 
         */
        node_id                     m_owner;
        
        boost::optional<Observation> m_last_obs, m_next_obs;
        TICK m_last_synch;
        bool m_echo;
      
        /** @brief Get owner of the timeline
         *
         * This non thread protected call directlt acces to the current 
         * owner of the timleine
         *
         * @note this call is not thread safe and should not normally be 
         * called directly
         *
         * @sa owner() const
         */
        node_id owner_sync() const;
        /** @brief reset owner
         *
         * This method disown the timelien form its owner. The operation is 
         * not thread protected and is meant to use only internally
         *
         * @retval true if the timleine was owned before tyhis call
         * @retval false otherwise
         *
         * @note this call is not thread safe and is onlyt meant for internal
         * uses
         *
         * @post !this->owner()
         */
        bool reset_sync();
        /** @brief set the timelien owner
         *
         * @param[in] n  The requester of ownership
         * @param[in] fl transaction flags
         *
         * This non thread protected call attempt to change the ownership 
         * of this timeline to the node @p n and the transaction flags to 
         * @p fl. Transaction flags indicate whether the timeline will accept
         * goals or not and whtjer it will publish its plan or not.
         *
         * The operation will success only if the timeline is not currently 
         * owned or @p n is the current owner of this timeline.
         *
         * @retval true if either the owner or the flags have been modified
         * @retval false otherwise
         */
        bool set_sync(SHARED_PTR<node_impl> const &n,
                      transaction_flags const &fl);
        
        void post_obs_sync(SHARED_PTR<node_impl> n, Observation o, bool echo);
        void notify_sync(TICK date);
        
        friend class graph_impl;
        internal_impl() DELETED;
      }; // TREX::transaction::details::internal_impl
      
      
      class external_impl :boost::noncopyable, public ENABLE_SHARED_FROM_THIS<external_impl> {
      public:
        external_impl(SHARED_PTR<node_impl> cli, tl_ref tl, transaction_flags const &fl);
        ~external_impl();
        
        utils::Symbol const &name() const {
          return m_timeline->name();
        }
        
        bool accept_goals() const;
        bool publish_plan() const;
        
        SHARED_PTR<graph_impl> graph() const;

        
      private:
        tl_ref            m_timeline;
        node_id           m_client;
        transaction_flags m_flags;
        
        external_impl() DELETED;
      }; // TREX::transaction::details::external_impl
      

    } // TREX::transaction::details
  } // TREX::transaction
} // TREX

#endif // H_trex_transaction_timeline_impl
