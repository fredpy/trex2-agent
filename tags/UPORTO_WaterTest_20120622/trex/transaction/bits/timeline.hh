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
#ifndef H_BITS_timeline 
# define H_BITS_timeline

# include "../Relation.hh"

namespace TREX {
  namespace transaction {
    namespace details {

      /** @brief TREX timeline representation
       *
       * This class is used by TREX ot represent a timeline and its connection
       * with reactors. It is used internally to miantin the relation graph betwen
       * reactors as well as the observation and goal exchanges for this given
       * timeline
       *
       * @author Frederic Py <fpy@mbari.org>
       */
      class timeline {
      public:
	typedef timeline      base_type;
	typedef utils::Symbol id_type;

	/** @brief Timline symbolic name
	 *
	 * @param[in] tl A timeline
	 *
	 * @return the name of @p tl
	 *
	 * @sa timeline::name() const
	 */
	static id_type const &get_id(base_type const &tl) {
	  return tl.name();
	}

	typedef Relation              iterator;
	typedef client_set::size_type size_type;

	/** @brief Constructor
	 * @param[in] date The current tick
	 * @param[in] name A symbolic name
	 *
	 * Creates the timeline @p name with the initial default @c Failed observation
	 * starting from the tick @p date.
	 *
	 * @post the timeline is not owned
	 *
	 * @sa owned() const
	 */
	timeline(TICK date, utils::Symbol const &name);
	/** @brief Constructor
	 * @param[in] date The current tick
	 * @param[in] name A symbolic name
	 * @param[in] serv A reactor 
	 *
	 * Creates the timeline @p name and makes @p serv its owner. It also creates
	 * the default observation @c Failed staring from @p date which should be
	 * replaced by an observation from @p serv
	 *
	 * @post the timeline is owned by @p serv
	 *
	 * @sa owned() const
	 * @sa owner() const
	 */
	timeline(TICK date, utils::Symbol const &name, TeleoReactor &serv, 
                 transaction_flags const &flags);
	/** @brief Destructor */
	~timeline();

	/** @brief timeline name
	 *
	 * This method gives acces to the timline unique name.
	 *
	 * @return The name of this timeline
	 */
	utils::Symbol const &name() const {
	  return m_name;
	}

	/** @brief Check if owned
	 *
	 * Indicates if this timlien is owned by any reactor
	 *
	 * @retavl true if the timlien have an owner
	 * @retval false otherwise
	 *
	 * @sa owner() const
	 * @sa owned_by(TeleoReactor const &) const
	 */
	bool owned() const {
	  return NULL!=m_owner;
	}
        
	/** @brief Check for ownership
	 *
	 * @param[in] r A reactor
	 *
	 * Checks if the reactor @p r is currently owning this timeline
	 *
	 * @retval true if the timeline is owned by @p r
	 * @retval false otherwise
	 *
	 * @sa owned() const
	 * @sa owner() const
	 */
	bool owned_by(TeleoReactor const &r) const {
	  return &r==m_owner;
	}
	/** @brief Timeline owner
	 * 
	 * Gets the reactor tahat owns this timline. It is usually
	 * the reactor thast first declared this timline as internal
	 *
	 * @pre the timline is owned
	 *
	 * @return the owner fo this timeline
	 *
	 * @sa owned() const
	 * @sa owned_by(TeleoReactor const &) const
	 * @sa assign(TeleoReactor &)
	 */
	TeleoReactor &owner() const {
	  return *m_owner;
	}

	/** @brief Check if active
	 *
	 * Checks if this timeline is owned by a reactor and has at
	 * least one client. In such case the timeline should be explored
	 * during the traversal of the reactors relation graph
	 *
	 * @sa owned() const
	 * @sa no_client() const
	 * @sa size() const
	 */
	bool active() const {
	  return owned() && !no_client();
	}
	/** @brief check if no client
	 *
	 * Indicates if this timeline has no reactor that declared it
	 * as external
	 *
	 * @retval true if it has no client
	 * @retval false oterhwise
	 *
	 * @sa size() const
	 */
	bool no_client() const {
	  return m_clients.empty();
	}
        bool should_publish() const;
        
	/** @brief Number of clients
	 *
	 * Indicates the number of reactors that declared this timline as external
	 *
	 * @return the number of client for this timeline
	 *
	 * @sa no_client() const
	 */
	size_type size() const {
	  return m_clients.size();
	}
	/** @brief Iterator to first client
	 *
	 * @return An iterator that refers to the first client
	 *
	 * @sa end()
	 */
	Relation begin() {
	  return Relation(this, m_clients.begin());
	}
	/** @brief Iterator to end of clients
	 *
	 * @return An iterator that refers to the end of the client list
	 *
	 * @sa begin()
	 */
	Relation end() {
	  return Relation(this, m_clients.end());
	}

	/** @brief last observation date
	 * 
	 * @return the date of the ladst observation
	 * @sa lastObservation() const
	 * @sa postObservation(TICK, Observation const &)
	 */
	TICK lastObsDate() const {
	  return m_obsDate;
	}
	/** @brief last observation 
	 * 
	 * @return the ladst observation
	 * 
	 * @sa lastObsDate() const
	 * @sa postObservation(TICK, Observation const &)
	 */
	Observation const &lastObservation() const {
	  return m_lastObs;
	}

	/** @brief Timeline look ahead
	 *
	 * Reflects the look ahaed of the reactor owning this timeline.
	 * This insdicates how far ahead a goal can be posted to the
	 * timeline. If the timlien does not accpet goals is look ahead
	 * is 0
	 *
	 * @note the look_ahead of a timelien can change. This happens
	 *       specifically when the owner of the timeline changes.
	 *
	 * @return the timeline look-ahead
	 *
	 * @sa TeleoReactor::getLookAhead() const
	 * @ssa TeleoReactor::latency()
	 */
	TICK look_ahead() const;
	/** @brief Timeline latency
	 *
	 * Indicates the timeline latency. As opposed to a reactor latency
	 * This latency not only include the deliberation time to include a
	 * token in a plan but also includes the maximum time it will take
	 * for the requested goal to be observed. It is actually what is called
	 * the execution latency identified by getExecLatency method for a reactor
	 *
	 * @note If the timeline has no reactor its latency is set to the default 
	 * value 0. Thids may change in the future
	 *
	 * @sa TeleoReactor::getExecLatency() const
	 */
	TICK latency() const;
        
        /** @brief Recall a goal
         *
         * @param[in] g A goal
         * 
         * This method inform the timeline (and potentially its owner) that the 
         * previously requested goal @p g is no longer requested
         *
         * @sa request(goal_id const &)
         * @sa TeleoReactor::handleRecall(goal_id const &);
         */
	void recall(goal_id const &g);
        /** @brief Request a new goal
         *
         * @param[in] g A goal
         * 
         * This method inform the timeline (and potentially/eventually its owner) 
         * that the goal @p g is requested for this timeline.
         *
         * @sa recall(goal_id const &)
         * @sa TeleoReactor::handleRequest(goal_id const &);
         */
	void request(goal_id const &g);

      private:
	/** @brief Create ownership
	 * @param[in] r            A reactor
         * @param[in] controllable Flag for goal acceptance
	 *
	 * Make @p r the owner of this timeline. If @p controlable is @c false 
         *      then this timeline will be marked as not accepting goals 
	 *
	 * @pre this timline is not owned or is already owned by @p r
	 *
	 * @retval true if @p r got the ownership 
	 * @retval false if @p r was alredy owning this timline
	 *
	 * @throw MultipleInternals the timline is already owned by
	 *        a reactor which is no @p r
	 * @post the timeline is owned by @p r
	 * @note If the returned value is @p true the reactor @p r is notified
	 *       of its new ownership through the method TeleoReactor::assigned
	 *
	 * @sa owned() const
	 * @sa unassign(TICK)
	 * @sa TeleoReactor::assigned(timeline const &)
	 */
	bool assign(TeleoReactor &r, transaction_flags const &flags);
        
        /** @brief Remove ownership
         *
         * @param[in] date the current tick
         *
         * Indicates that the current owner of this timeline is giving up its 
         * ownership. At the current date @p date
         * If no reactor is currently owning this timeline nothing will happen, 
         * otherwise the current owner will be notified of not owning this 
         * timeline anymore and the Failed observation will be prepapred to be 
         * posted. 
         * 
         * @return The reactor that was owning this timeline or @c NULL if this 
         *         timeline was owned by nobody
         * @post This timeline is not owned anymore
         * @sa owned() const
         * @sa assign(TeleoReactor &, bool)
         * @sa TeleoReactor::unassigned(timeline const &)
         */
        TeleoReactor *unassign(TICK date);
        /** @brief timeline demotion
         *
         * @param[in] date the current tick
         * @param[in] flags subscribtion flags
         *
         * Indicates that the current owner of the timeline demote this timeline 
         * from internal to external
         * 
         * @post This timeline is not owned anymore
         * @post if this taimeline had an owner then this owner has now became 
         *       a client to the timeline with the indicated flags.
         * @sa unassign(TICK)
         * @sa subscribe(TeleoReactor &r, transaction_flags const &)
         */
        void demote(TICK date, transaction_flags const &flags);
	/** @brief Add a new client
	 *
	 * @param[in] r       A reactor
	 * @param[in] control goal posting flag
	 *
	 * Add @p r to the clients of this timeline if it is not the owning it.
	 *
	 * If @p control indicateds whether @p r will post goals on this
	 * timeline or not.
	 *
	 * @note if @p r was already a client of this timeline but its @p control
	 *       flag is now @c true, the ability to post goals will be changed
	 *       accordingly. Otherwise the old parameters will be kept as they were.
	 * @note if the reactor is already the owner of this timeline this operation 
	 *       will have no impact
	 *
	 * @retval true if the reactor was added as a client
	 * @retval false otherwise
	 *
	 * @post if the returned value is @c true the number of clients is
	 *       incremented by 1, a new Relation is created and sent to @p r
	 *       through the TeleoReactor::subscribed method
	 * 
	 * @sa owner() const
	 * @sa assign(TeleoReactor &)
	 * @sa unsubscribe(Relation const &)
	 * @sa TeleoReactor::subscribed(Relation const &)
	 */
	bool subscribe(TeleoReactor &r, transaction_flags const &flags);
	/** @brief Remove a relation
	 *
	 * @param[in] rel A relation
	 *
	 * @pre @p rel is valid
	 * @pre @p rel is associated to this timeline 
	 *
	 * This method indicates that the relation @p rel is not used anymore. It
	 * is called by the client of this relation when this client does not
	 * declare this timline as internal anymore
	 *
	 * @post the relation @p r is removed from this timeline
	 * @post the client of this relation is notified through
	 *       TeleoReactor::unsubscribed(Relation const &) callback
	 *
	 * @sa subscribe(TeleoReactor &,bool)
	 * @sa Relation::client() const
	 * @sa TeleoReactor::unsubscribed(Relation const &)
	 */
	void unsubscribe(Relation const &rel);

	/** @brief Post a new observation
	 *
	 * @param[in] date the current tick
	 * @param[in] obs An observation
	 *
	 * @pre @p date is the current date synchronization
	 * @pre this call should be made only by the wner of this timeline (or the
	 *      agent if there's no owner)
	 *
	 * Notifies the timeline that the observation @p obs should be produced
	 * starting from the tick @p date. The last observation and its data are
	 * updated accrodingly.
	 *
	 * If no new observation is produced before the next synchronization this
	 * observation will be propagated to all the clients of this timeline 
	 *
	 * @post lastObsDate is updated to @p date
	 * @post lastObservation is updated to @p obs
	 *
	 * @sa lastObservation() const
	 * @sa lastObsDate() const
	 */
	void postObservation(TICK date, Observation const &obs);
        
        bool notifyPlan(goal_id const &t);
        bool cancelPlan(goal_id const &t);

	void latency_update(TICK prev);

        /** @brief Name of the timeline
         * 
         * The name identifier for this timeline. 
         */
	utils::Symbol m_name;
        /** @brief owner of the timeline 
         *
         * A pointer to the reactor currently decalring this timeline as 
         * Internal or @c NULL if this timeline is currently not owned
         */
	TeleoReactor *m_owner;
        /** @brief Request acceptance flag
         * 
         * A flag that indicates if the timeline is currently accepting request 
         * or not. No request will be transmitted to the owner of this timeline 
         * as long as this flag is @c false.
         */
        transaction_flags m_transactions;
        size_t            m_plan_listeners;
        /** @brief Clients of the timeline
         *
         * The set of reactors that subscribe to this timeline. This structure 
         * also embed extra information such as whether this reactor will post 
         * goals on this timeline and is expecting to receive plan broadcast.
         */
	client_set    m_clients;
	
        /** @brief Last observation posted
         * 
         * The value of the last observation posted to this timeline.
         */
	Observation   m_lastObs;
        /** @brief last observation posting date.
         * 
         * The tick date when the last observation was posted
         */
	TICK          m_obsDate;
	
	/** @brief Name of the special @c Failed observation
	 *
	 * This constant gives the named associated to the special token
	 * @c Failed. This toke is used by the agent when a timline is not
	 * owned (or lost ownership)
	 *
	 * The lost of ownership often happens when a the owner of this timline
	 * failed to synchronize. But will also happen if the reactor explicitely
	 * unassign itself
	 *
	 * @sa unassign(TICK,bool,bool)
	 */
	static utils::Symbol const  s_failed;
	
	friend class TREX::transaction::TeleoReactor;
	friend class TREX::transaction::Relation;
	friend class TREX::transaction::graph;
      }; //TREX::transaction::details::timeline

      typedef utils::pointer_id_traits<details::timeline> tl_ptr_id_traits;
      typedef utils::list_set<tl_ptr_id_traits>           timeline_set;

      /** @brief Iterator for reactors relations
       *
       * This class is used to iterate through the external reelations
       * of a reactor. It is mostly used by the different algorithms provided
       * by the boost graph library in order to traverse the reactors graph
       *
       * @ingroup transaction
       * @author Frederic Py <fpy@mbari.org>
       * @relates class TeleoReactor
       */
      class relation_iter 
	:public boost::iterator_facade< relation_iter,
					Relation const,
					boost::forward_traversal_tag > {
      public:
	/** @brief Constructor */
	relation_iter() {
	  m_pos = m_last;
	}
	/** @brief Copy Constructor
	 *
	 * @param[in] other the insatnce to copy
	 */
	relation_iter(relation_iter const &other)
	  :m_pos(other.m_pos), m_last(other.m_last), m_rel(other.m_rel) {}
	/** @brief Constructor 
	 *
	 * @param[in] it position for this iterator
	 * @param[in] last position corresponding to the end of the iteration
	 *
	 * This constructor is used by TeleoReactor to create new iterators 
	 */
	relation_iter(timeline_set::const_iterator const &it,
		      timeline_set::const_iterator const &last);
	/** @brief Destructor */
	~relation_iter() {}

	
      private:
	friend class boost::iterator_core_access;

	void increment();
	bool equal(relation_iter const &other) const;
	Relation const &dereference() const {
	  return m_rel;
	}
	
	void next_valid();

	timeline_set::const_iterator m_pos, m_last;
	Relation m_rel;
      }; // relation_iter
      

    } // TREX::transaction::details
  } // TREX::transaction
} // TREX

#endif // H_BITS_timeline
