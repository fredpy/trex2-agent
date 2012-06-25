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
      static TeleoReactor::xml_arg_type &alter_cfg(TeleoReactor::xml_arg_type &arg);

      bool m_work, m_inited;
      
      friend class details::tr_event;
    }; // TREX::transaction::LogPlayer

  } // TREX::transaction
} // TREX

#endif // H_trex_transaction_LogPlayer
