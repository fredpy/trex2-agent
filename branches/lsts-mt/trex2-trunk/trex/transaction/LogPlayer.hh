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

      /** @brief Transaction event
       * 
       * And abstarct class that decibe a transaction event to be 
       * replayed from a log.
       *
       * @relates LogPlayer
       * @ingroup transaction
       * @aurthor Frederic Py
       */
      class tr_event {
      public:
	/** @brief transaction events factory
	 *
	 * The factory used to generate transactions vents from a log 
	 * file.
	 */
	typedef utils::XmlFactory<tr_event, SHARED_PTR<tr_event>,
				  LogPlayer *> factory;

	/** @brief Constructor 
	 * @param[in] arg A XML descriptor
	 * 
	 * The constructor called byt the factory to produce new 
	 * transaction events. @p arg embeds the XML descriptor along 
	 * with a reference to the Logplayer that initiated this call.
	 * This constructor just extract this LogpPlayer in order to 
	 * associate the new instance to this reactor.
	 */
	tr_event(factory::argument_type const &arg);
	/** @brief Destructor */
	virtual ~tr_event() {}

	/** @btrief play event
	 * 
	 * This method is called to replay the event this instance 
	 * describe.
	 */
	virtual void play() =0;

      protected:
	/** @brief Get a goal
	 * 
	 * @param[in] key A key identifiying the goal in the log
	 * 
	 * Request to the Logplayer to identify the goal_id 
	 * associated to @p key
	 * 
	 * @return the goal_id referred by @p key or a NULL 
	 * goal_id if no such goal exists
	 * 
	 * @sa set_goal(std::string const &, goal_id const &)
	 * @sa LogPLayer::m_goal_map
	 */
	goal_id get_goal(std::string const &key);
	/** @brief Associate goal
	 * 
	 * @param[in] key A key 
	 * @param[in] g A goal_id
	 *
	 * Associates the goal @p g to the key @p key. If a goal 
	 * was already associated to @p key it will be replaced 
	 * by @p g
	 *
	 * @post the key @p key is associated to @p g
	 * @sa get_goal(std::string const &)
	 * @sa LogPLayer::m_goal_map
	 */
	void set_goal(std::string const &key, goal_id const &g);

	/** @briefAssociated LogPlayer
	 * 
	 * The LogPlayer this instance is attached to.
	 */
	LogPlayer &m_reactor;
      }; // TREX::transaction::details::tr_event

    } // TREX::transaction::details

    /** @brief Transaction log player
     * 
     * This reactor allow easily to replay a mission by replacing a
     * reactor by this instance whicvh will then replay its transaction 
     * log file (the @c .tr.log). This very usefull to reproduce a 
     * mission as it happened in order to debug an isdue into the 
     * deterministic reactors (such as EuropaReactor). 
     *
     * @note This LogPLayer is meant to replay logs from trex version 
     * 0.4.0. If you need to replay a mission ran using an anterior 
     * version please refer to TREX::backward::Player
     *
     * @ingroup transaction
     * @author Frederic Py
     */
    class LogPlayer :public TeleoReactor {
    public:
      /** @brief Constructor 
       * 
       * @param[in] arg A XML descriptor
       *
       * Create a new LogPlayer instance using the XML descriptor 
       * @p arg. The XML format is:
       * @code 
       * <LogPlayer name="<name>" latency="<latency>" lookahead="<lookahead>"
       *            file="<logfile>" />
       * @endcode 
       * Where:
       * @li @c <name> is the reactor name
       * @li @c <latency> is the reactor latency
       * @li @c <lookahead> is the reactor lookahead
       * @li @c <logfile> is an optional attribute pointing 
       *     to the transaction log file to replay. If this 
       *     is not specified then the reactor will olook for 
       *     @c <name>.tr.log
       * 
       * @pre the log file loaded is a valid transaction log file.
       *
       * @note if the attribute @p log from TeleoReactor is set to 1
       * this constructor will automatically force it to 0 in order 
       * to not relog the transaction that are being played.
       */
      LogPlayer(TeleoReactor::xml_arg_type arg);
      /** @brief Destructor */
      ~LogPlayer(); 

      /** @brief set work need
       *
       * @param[in] val A boolean
       * 
       * Sets the next value to be returned by hasWork() to @p val. 
       * This allow to emulate the LogPLayer requesting for work 
       */
      void set_work(bool val) {
	m_work = val;
      }
      /** @brief External timeline declaration
       * 
       * @param[in] tl    A timeline name
       * @param[in] goals Control flag
       * @param[in] plan Plan subscription flag
       * 
       * subscribe this reactor to the timeline @p tl with the transaction 
       * flags @p goals and @p plan
       */
      void play_use(utils::Symbol const &tl, bool goals, bool plan);
      /** @brief External timeline undeclaration
       * 
       * @param[in] tl    A timeline name
       * 
       * Unsubscribe this reactor to the timeline @p tl.
       */
      void play_unuse(utils::Symbol const &tl);
      /** @brief Internal timeline declaration
       * 
       * @param[in] tl    A timeline name
       * @param[in] goals Control flag
       * @param[in] plan Plan plublish flag
       * 
       * Make this reactor delcare the timeline @p tl as Internal 
       * with the transaction flags @p goals and @p plan
       */
      void play_provide(utils::Symbol const &tl, bool goals, bool plan);
      /** @brief Internal timeline undeclaration
       * 
       * @param[in] tl    A timeline name
       * 
       * Make this reactor give away its ownership of the timeline @p tl.
       */
      void play_unprovide(utils::Symbol const &tl);
      /** @brief post observation
       *
       * @param[in] obs An observation
       *
       * Make this reactor publish the observation @p obs.
       *
       * @pre obs object should  be Internal to this reactor
       */
      void play_obs(Observation const &obs);
      /** @brief post request
       *
       * @param[in] g A goal
       *
       * Make this reactor send a request for @p g.
       *
       * @pre g object should  be External to this reactor
       */
      void play_request(goal_id const &g);
      /** @brief post recall
       *
       * @param[in] g A goal
       *
       * Make this reactor cancel the request for @p g.
       *
       * @pre g should have been formerly requested by this reactor
       */
      void play_recall(goal_id const &g);
      /** @brief publish plan token
       *
       * @param[in] g A goal
       *
       * Make this reactor publish the token @p g as part of its plan
       *
       * @pre g object should  be Internal to this reactor
       */
      void play_add(goal_id const &g);
      /** @brief cancel plan token
       *
       * @param[in] g A goal
       *
       * Make this reactor notify that the token @p g is no longer part of 
       * its plan.
       *
       * @pre g should have be publish as a plan before
       */
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
     
      /** @brief Execution phase 
       * 
       * A reactor execution phase. This calss associates to a specific 
       * phase of the reactor execution (such as init, new_tick, 
       * synchronize, has_work or step) A set of transaction events to 
       * be replayed. This aloow the logpleyer to reporduce accurately 
       * all these vents as they did occur within the tick.
       *
       * @relates LogPlayer
       * @ingroup transaction
       * @author Frederic Py
       */
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
	std::list< SHARED_PTR<details::tr_event> > m_events;
      };

      typedef std::pair< TICK, SHARED_PTR<phase> > tick_event;
      std::list<tick_event>    m_log;
      std::map<std::string, goal_id> m_goal_map;

      bool next_phase(TICK tck, utils::Symbol const &kind); 
      bool in_tick(TICK tck) const;
      
      static TeleoReactor::xml_arg_type &alter_cfg(TeleoReactor::xml_arg_type &arg);

      bool m_work, m_inited;
      
      friend class details::tr_event;
    }; // TREX::transaction::LogPlayer

  } // TREX::transaction
} // TREX

#endif // H_trex_transaction_LogPlayer
