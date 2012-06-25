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
#ifndef H_trex_transaction_LogPlayer
# define H_trex_transaction_LogPlayer

# include "TeleoReactor.hh"

namespace TREX {
  namespace transaction {

    class LogPlayer;

    namespace details {

      class tr_event {
      public:
	typedef utils::XmlFactory<tr_event, boost::shared_ptr<tr_event>, 
				  LogPlayer *> factory;

	tr_event(factory::argument_type const &arg);
	virtual ~tr_event() {}

	virtual void play() =0;

      protected:
	goal_id get_goal(std::string const &key);
	void set_goal(std::string const &key, goal_id const &g);

	LogPlayer &m_reactor;
      };

    } // TREX::transaction::details

    class LogPlayer :public TeleoReactor {
    public:
      LogPlayer(TeleoReactor::xml_arg_type arg);
      ~LogPlayer(); 

      void set_work(bool val) {
	m_work = val;
      }
      void play_use(utils::Symbol const &tl);
      void play_unuse(utils::Symbol const &tl);
      void play_provide(utils::Symbol const &tl);
      void play_unprovide(utils::Symbol const &tl);
      void play_obs(Observation const &obs);
      void play_request(goal_id const &g);
      void play_recall(goal_id const &g);
      void play_add(goal_id const &g);
      void play_cancel(goal_id const &g);

    private:
      void handleInit();
      void handleTickStart();
      bool synchronize();
      bool hasWork();
      void resume();
      

      static utils::Symbol const s_init;
      static utils::Symbol const s_new_tick;
      static utils::Symbol const s_synchronize;
      static utils::Symbol const s_has_work;
      static utils::Symbol const s_step;
     
      class phase {
      public:
	phase(LogPlayer *owner, boost::property_tree::ptree::value_type &node);
	~phase() {}

	utils::Symbol const &type() const {
	  return m_type;
	}
	void execute() {
	  while( !m_events.empty() ) {
	    m_events.front()->play();
	    m_events.pop_front();
	  }
	}
      private:
	utils::Symbol m_type;
	std::list< boost::shared_ptr<details::tr_event> > m_events;
      };

      typedef std::pair< TICK, boost::shared_ptr<phase> > tick_event;
      std::list<tick_event>    m_log;
      std::map<std::string, goal_id> m_goal_map;

      bool next_phase(TICK tck, utils::Symbol const &kind); 

      bool m_work, m_inited;
      
      friend class details::tr_event;
    }; // TREX::transaction::LogPlayer

  } // TREX::transaction
} // TREX

// ========================================================================
namespace TREX {
  namespace transaction {

    /** @brief Log transaction player
     *
     * This teleoreactor allows user to replay the transactions (observations
     * and goals) produced by a reactor in order to replay a mission. It is
     * especially usefull to debug a situated mission for debugging purpose by
     * replacing all the reactors that are not deterministic (such as the interfaces
     * with external components).
     *
     * The xml specification pf this reactor is as follow
     * @code
     * <LogPlayer name="foo" latency="1" lookahead="1" file="foo.tr.log" />
     * @endcode
     *
     * The @c name, @c latency and @c lookahead attroibutes have the typical meaning. 
     * The @e optional attribute @c file indicates which transaction file should be
     * replayed by this reactor. If not specified  the system will atempt to locate
     * the file @c <name>.tr.log in the TREX_PATH
     *
     * @note As this reactor simply replay a log file it does not process objective
     * recived nor is impacted by external observations. As a result it is really
     * meant  to be used in an agent that uses exeactly the same reactors as the
     * original mission (except of course for potetnail other LogPlayer instances).
     *
     * @todo Implement the support for posting goals and recalls
     * @todo Add full support for being able to undeclare External/Internal timelines
     *
     * @author Frederic Py <fpy@mbari.org>
     */
    class Player :public TeleoReactor {
    public:
      /** @brief Constructor
       * @param[in] arg XML description
       *
       * Create a new instance based on the XML description @p arg
       * The expected fortmat for this description is as follow 
       * @code
       * <LogPlayer name="foo" latency="1" lookahead="1" file="foo.tr.log" />
       * @endcode
       *
       * The @c name, @c latency and @c lookahead attroibutes have the typical
       * meaning.  
       * The @e optional attribute @c file indicates which transaction file should be
       * replayed by this reactor. If not specified  the system will atempt to locate
       * the file @c <name>.tr.log in the TREX_PATH
       *
       * @throw ReactorException Unable to locate the transaction log file
       * @throw ReactorException The transaction log file is empty
       * @throw XmlError Failed to parse the transaction log file
       */
      Player(TeleoReactor::xml_arg_type arg);
      /** @briedf Destructor */
      ~Player();

    private:
      /** @brief Modifies a factory xml argument 
       * 
       * param[in,out] arg The argeument to modify
       *
       * Thsi methods modifies the teleoreactor Xml specification @p arg by forcing
       * the log attribute to false.
       *
       * This operation is done as :
       * @li the default value for log is true
       * @li It does not really make sense to log the transactions of a
       *     LogPlayer (as they are specified in the log file).
       *
       * @return @p arg after modification
       */
      static TeleoReactor::xml_arg_type &transform(TeleoReactor::xml_arg_type &arg);

      void handleTickStart();
      bool synchronize();
      
      /** @brief load transaction file
       * @param[in] root xmla document root
       *
       * This method is used during construction in order to load the
       * transactions to produce from the xml document @p root
       *
       * @throw XmlError error while attempting to extract data from @p root
       */
      void loadTransactions(boost::property_tree::ptree &properties);

      /** @brief Transaction descriptor
       *
       * This base class is used as am placeholder for a reactor transaction.
       * Transactions include all the communication a reactor can do which are
       * related to the timeline vconnection between reactors. It includes :
       * @li declaring a timeline (as Internal or External)
       * @li undeclaring a timeline
       * @li posting a new observation
       * @li posting a new goal
       * @li recalling a goal
       *
       * @relates Player
       * @author Frederic Py <fpy@mbari.org>
       */        
      class transaction {
      public:
	/** @biref Constructor */
	transaction() {}
	/** @brief Destructor */
	virtual ~transaction() {}

	/** @brief Transaction execution
	 *
	 * @param[in] p A transaction player
	 *
	 * Executes this transaction in the reactor @p p
	 */
	virtual void accept(Player &p) = 0;
      }; // TREX::transaction::Player::transaction

      /** @brief Timeline transaction
       *
       * All the transaction related to timeline declaration/deallocation
       *
       * @relates Player
       */
      class timeline_transaction :public transaction {
      public:
	/** @brief Constructor
	 * @param node An xml node
	 *
	 * Create a new instance based on the desciption given by @p node
	 *
	 * @pre @p node has an attribute @c name that indicates the name
	 *      of the timeline concerned
	 *
	 * @throw  XmlError missing @c name attribute for @p node
	 */
	timeline_transaction(boost::property_tree::ptree::value_type &node);
	/** @brief Destructor */
	virtual ~timeline_transaction() {}

	/** @brief Timeline name
	 *
	 * @return the name of the timeline related to this operation
	 */
	TREX::utils::Symbol const &name() const {
	  return m_name;
	}	
      protected:
	/** @brief timeline name */
	TREX::utils::Symbol m_name;
      }; // TREX::transaction::Player::timeline_transaction

      /** @brief Internal timeline declaration
       *
       * Describe an Internal timeline operation
       * 
       * @relates Player
       */
      class op_provide :public timeline_transaction {
      public:
	/** @brief Constructor
	 * @param[in] node xml definition of the transaction
	 *
	 * Ther expected fpormat f @p node is:
	 * @code
	 * <provide name="timeline_name"/>
	 * @endcode
	 */
	op_provide(boost::property_tree::ptree::value_type &node)
	  :timeline_transaction(node) {}
	/** @brief Destcructor */
	~op_provide() {}

	void accept(Player &p) {
	  p.provide(name());
	}
      }; // TREX::transaction::Player::op_provide

      /** @brief External timeline declaration
       *
       * Describe an External timeline operation
       * 
       * @relates Player
       */
      class op_use :public timeline_transaction {
      public:
	/** @brief Constructor
	 * @param[in] node xml definition of the transaction
	 *
	 * Ther expected format for @p node is:
	 * @code
	 * <use name="timeline_name"/>
	 * @endcode
	 */
	op_use(boost::property_tree::ptree::value_type &node)
	  :timeline_transaction(node) {}
	/** @brief Destructor */
	~op_use() {}

	void accept(Player &p) {
	  p.use(name(), true, false); // potential post goals but do not listen to timeline plan
	}
      }; // TREX::transaction::Player::op_use

       /** @brief undeclare Internal timeline 
       *
       * Describe the operation to undeclare an Internal timeline
       *
       * @bug So far this operation is not supported yet. If such operation occurs
       * it will kill the corresponding reactor. 
       * 
       * @relates Player
       */
      class op_unprovide :public timeline_transaction {
      public:
	/** @brief Constructor
	 * @param[in] node xml definition of the transaction
	 *
	 * Ther expected format for @p node is:
	 * @code
	 * <unprovide name="timeline_name"/>
	 * @endcode
	 */
	op_unprovide(boost::property_tree::ptree::value_type &node)
	  :timeline_transaction(node) {}
	/** @brief Destructor */
	~op_unprovide() {}
	
	void accept(Player &p) {
	  p.syslog("WARN")<<"unprovide not replayable yet : I am killing myself.";
	  p.m_continue = false;
	}
      }; // TREX::transaction::Player::op_unprovide

      /** @brief undeclare External timeline 
       *
       * Describe the operation to undeclare an External timeline
       *
       * @bug So far this operation is not supported yet. The operation is
       * simply ignored with a warning message produced in TREX.log
       * 
       * @relates Player
       */
      class op_unuse :public timeline_transaction {
      public:
	/** @brief Constructor
	 * @param[in] node xml definition of the transaction
	 *
	 * Ther expected format for @p node is:
	 * @code
	 * <unuse name="timeline_name"/>
	 * @endcode
	 */
	op_unuse(boost::property_tree::ptree::value_type &node)
	  :timeline_transaction(node) {}
	/** @brief Destructor */
	~op_unuse() {}

	void accept(Player &p) {
	  p.syslog("WARN")<<"unuse not replayable yet : I'll just do nothing.";
	}
      };

       /** @brief Post an observation
       *
       * Describe the operation to post an observation on Internal timeline 
       *
       * @relates Player
       */
      class op_assert :public transaction {
      public:
	/** @brief Constructor
	 *
	 * @param[in] node A xml description for an observation
	 *
	 * Create a new instance describing the action on posting the
	 * observation described by @p node
	 *
	 * @pre @p node is a valid Observation description
	 *
	 * @throw XmlError unable to parse @p node as an Observation
	 *
	 * @sa Observation::Observation(rapidxml::xml_node<> &)
	 */
	op_assert(boost::property_tree::ptree::value_type &node);
	/** @brief Destructor */
	~op_assert() {}
	
	void accept(Player &p) {
	  p.postObservation(m_obs);
	}
	
      private:
	/** @brief The observation
	 * The observation that will be posted by this operation
	 */
	Observation m_obs;
      }; // TREX::transaction::Player::op_assert

      /** @brief Request operation
       *
       * Describe the operation to send a request on an External timeline
       *
       * @relates Player
       */
      class op_request :public transaction {
      public:
	/** @brief Constructor
	 *
	 * @param[in] node A xml description for a request
	 *
	 * Create a new instance describing the action on requesting the
	 * goal described by @p node
	 *
	 * The usual format for a request is
	 * @code
	 * <request id="<an id>">
	 *   <Goal ....>
         *     [...]
	 *   </Goal>
	 * </request>
	 * @endcode 
	 * 
	 * @pre @p node is a valid request description
	 *
	 * @throw XmlError unable to parse @p node as a request
	 *
	 * @sa Goal::Goal(rapidxml::xml_node<> &)
	 */
	op_request(boost::property_tree::ptree::value_type &node);
	/** @brief Destructor */
	~op_request() {}
	
	void accept(Player &p) {
	  p.play_request(m_id, m_request);
	}
	
      private:
	/** @brief logged id
	 *
	 * This string indicates the id value extracted from the log.
	 * This value is used in order to associate the logged id to the real
	 * goal_id used durign this run. It is important to kee p track on that
	 * in order to recall the proper goal if needed
	 * 
	 * @sa Player::op_recall
	 */
	std::string m_id;
	/** @brief The goal
	 *
	 * This is the goal that will be effectively posted durign this execution.
	 */
	goal_id m_request;
      }; // TREX::transaction::Player::op_request

       /** @brief Recall operation
       *
       * Describe the operation to recall a former request on an
       * External timeline
       *
       * @relates Player
       */
      class op_recall :public transaction {
      public:
	/** @brief Constructor
	 *
	 * @param[in] node A xml description for a recall
	 *
	 * Create a new instance describing the action on recalling the
	 * goal described by @p node
	 *
	 * The usual format for a request is
	 * @code
	 * <recall id="<an id>"/>
	 * @endcode 
	 * 
	 * @pre @p node is a valid recall description
	 *
	 * @throw XmlError unable to parse @p node as a recall
	 */
	op_recall(boost::property_tree::ptree::value_type &node);
	/** @brief Destructor */
	~op_recall() {}
	
	void accept(Player &p) {
	  p.play_recall(m_id);
	}
	
      private:
	/** @brief Logged id
	 *
	 * The id extracted from the log. This attribute will be used
	 * during execution in order to identify the real id of the request
	 * to recall
	 *
	 * @sa Player::play_recall(std::string const &)
	 */
	std::string m_id;
      }; // TREX::transaction::Player::op_recall
    
    
      /** @brief Internal token broadcast operation
       *
       * Describe the operation to notify of a new plan token on an internal 
       * timeline
       *
       * @relates Player
       */
      class op_token :public transaction {
      public:
	/** @brief Constructor
	 *
	 * @param[in] node A xml description for a request
	 *
	 * Create a new instance describing the action on requesting the
	 * goal described by @p node
	 *
	 * The usual format for a request is
	 * @code
	 * <token id="<an id>">
	 *   <Goal ....>
         *     [...]
	 *   </Goal>
	 * </token>
	 * @endcode 
	 * 
	 * @pre @p node is a valid request description
	 *
	 * @throw XmlError unable to parse @p node as a request
	 *
	 * @sa Goal::Goal(rapidxml::xml_node<> &)
	 */
	op_token(boost::property_tree::ptree::value_type &node);
	/** @brief Destructor */
	~op_token() {}
	
	void accept(Player &p) {
	  p.play_token(m_id, m_token);
	}
	
      private:
	/** @brief logged id
	 *
	 * This string indicates the id value extracted from the log.
	 * This value is used in order to associate the logged id to the real
	 * goal_id used durign this run. It is important to kee p track on that
	 * in order to recall the proper goal if needed
	 * 
	 * @sa Player::op_recall
	 */
	std::string m_id;
	/** @brief The goal
	 *
	 * This is the goal that will be effectively posted durign this execution.
	 */
	goal_id m_token;
      }; // TREX::transaction::Player::op_request
      
      /** @brief Cancel operation
       *
       * Describe the operation to cancle a former plan token on an
       * Internal timeline
       *
       * @relates Player
       */
      class op_cancel :public transaction {
      public:
	/** @brief Constructor
	 *
	 * @param[in] node A xml description for a recall
	 *
	 * Create a new instance describing the action on cancelling the
	 * plan token described by @p node
	 *
	 * The usual format for a request is
	 * @code
	 * <cancel id="<an id>"/>
	 * @endcode 
	 * 
	 * @pre @p node is a valid cancel description
	 *
	 * @throw XmlError unable to parse @p node as a cancel
	 */
	op_cancel(boost::property_tree::ptree::value_type &node);
	/** @brief Destructor */
	~op_cancel() {}
	
	void accept(Player &p) {
	  p.play_cancel(m_id);
	}
	
      private:
	/** @brief Logged id
	 *
	 * The id extracted from the log. This attribute will be used
	 * during execution in order to identify the real id of the request
	 * to recall
	 *
	 * @sa Player::play_recall(std::string const &)
	 */
	std::string m_id;
      }; // TREX::transaction::Player::op_recall


      /** @brief Destroy a list of pointer
       * @param[in,out] l A list
       *
       * This method delete all the elements of @p l before clearing it.
       * It is used in order to destroy the transactions lists maintained
       * by a Player instance wihtout creating a memory leak.
       *
       * @pre all the previous elment of @p l are deleted 
       * @post @p l is empty 
       */
      static void clear(std::list<transaction *> &l);

      /** @brief Execution map
       *
       * This attribute map to tick values all the operations that need to
       * be done for this tick.
       *
       * It is where the loaded ,log content is stored and the Player will
       * gather information on what transactions need to be done at every tick.
       */
      std::map< TICK, std::list<transaction *> > m_exec;
      /** @brief Reactor life flag
       *
       * This attribute indicates to the player hether it sohould succeed or fail
       * at the next synchronization. If its value becomes false, this instance will
       * "fail" the next synchronization
       */
      bool m_continue;

      /** @brief List of "active" goals
       *
       * This map associetes the logged id of a goal with its real goal id.
       * It is used by both Player::op_request and Player::op_recall in order
       * to identify the assciation between a logged id and its real counterpart.
       * By doing so we are sure to recall the proper goal when replaying a mission.
       *
       * @sa play_request(std::string const &, goal_id const &)
       * @sa play_recall(std::string const &)
       */
      std::map<std::string, goal_id> m_goals;

      /** @brief Post a new goal
       *
       * @param[in] id The id of the goal from the log
       * @param[in] g The goal to be posted
       *
       * This method post the goal @p g. If the operation succeddd it then
       * associates @p g to its @p id in m_goals.
       *
       * Otherwise it display a warning message in TREX.log
       *
       * @sa play_recall(std::string const &);
       */
      void play_request(std::string const &id, goal_id const &g);
      /** @brief Recall a goal
       *
       * @param[in] id The id of the goal from the log
       *
       * This method attempts to recall the goal asscoaited to the log id @p id.
       * It attempts first to find this goal in m_goals, then if it finds it it
       * recall it. otherwise it display a warning message in TREX.log
       * 
       * @sa  play_request(std::string const &, goal_id const &);
       */
      void play_recall(std::string const &id);
      
      void play_token(std::string const &id, goal_id const &g);
      void play_cancel(std::string const &id);
 
      virtual bool failed_external(TREX::utils::Symbol const &timeline, graph::timeline_failure const &err) {
        throw ReactorException(*this, "Failed to subscribe to external timeline "+timeline.str()+": "+err.what());
        return false;
      }
      virtual bool failed_internal(TREX::utils::Symbol const &timeline, graph::timeline_failure const &err) {
        throw ReactorException(*this, "Failed to create internal timeline "+timeline.str()+": "+err.what());
        return false;
      }

      
      friend class op_provide;
      friend class op_use;
      friend class op_unprovide;
      friend class op_unuse;
      friend class op_assert;
      friend class op_request;
      friend class op_recall;
      friend class op_token;
      friend class op_cancel;
    }; // TREX::transaction::Player 

  } // TREX::transaction
} // TREX

#endif // H_TransactionPlayer
