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
#include "LogPlayer.hh"
#include <set>


namespace TREX {
  namespace transaction {
    namespace details {

      /** @brief timeline related event
       *
       * The base class for all the log events related to timeline 
       * subscription/creation
       *
       * @relates LogPlayer
       * @ingroup transaction
       * @author Frederic Py
       */
      class tl_event :public tr_event {
      protected:
	/** @brief Constructor
	 *
	 * @param[in] arg A xml descriptor
	 *
	 *  Create a new instance based on the xml information 
	 * embedded in @p arg. The XML format expected is:
	 * @code 
	 * < <op_type> name="<timeline>" goals="<goals>" plan="<plan>" />
	 * @endcode
	 * Where:
	 * @li @c <op_type> is the type of operation
	 * @li @c <name> is the tinmeline it applies to
	 * @li @c <goals> indicates whether the timelien will accept/post goals
	 * @li @c <plan> indicates whether the timellin will publish/subscribe 
	 *     to its internal plan
	 * Both @c <goals> and @c <plan> are optional and will be set to 1 if 
	 * absent.
	 */
	tl_event(factory::argument_type const &arg);
	/** @brief Destructor */
	virtual ~tl_event() {}
	
	/** @brief timeline
	 *
	 * @return the name of the timeline this operation applies to
	 */
	utils::symbol const &timeline() const {
	  return m_timeline;
	}
	/** @brief goal flag
	 *
	 * @return a flag indicating if this timeline accept/post goals
	 */
	bool goals() const {
	  return m_goals;
	}
	/** @brief plan
	 *
	 * @return A flag that indicate if this timleine publish/listen to 
	 * its @e Internal plan
	 */
	bool plan() const {
	  return m_plan;
	}
      private:
	utils::symbol m_timeline;
	bool m_goals, m_plan;
      }; // TREX::transaction::details::tl_event
      
      /** @brief timeline use operation
       *
       * This class describe an External timeline declaration operation 
       * extracted from the log 
       *
       * @relates LogPlayer
       * @ingroup transaction
       * @author Frederic Py
       * @sa class tr_unuse
       */
      class tr_use:public tl_event {
      public:
	tr_use(factory::argument_type const &arg)
	  :tl_event(arg) {}
	~tr_use() {}
      private:
	void play() {
	  m_reactor.play_use(timeline(), goals(), plan());
	}
      }; // TREX::transaction::details::tr_use

      /** @brief timeline unuse operation
       *
       * This class describe an External timeline undeclaration operation 
       * extracted from the log 
       *
       * @relates LogPlayer
       * @ingroup transaction
       * @author Frederic Py
       * @sa class tr_use
       */
      class tr_unuse:public tl_event {
      public:
	tr_unuse(factory::argument_type const &arg)
	  :tl_event(arg) {}
	~tr_unuse() {}
      private:
	void play() {
	  m_reactor.play_unuse(timeline());
	}
      }; // TREX::transaction::details::tr_unuse
      
      /** @brief timeline provide operation
       *
       * This class describe an Internal timeline declaration operation 
       * extracted from the log 
       *
       * @relates LogPlayer
       * @ingroup transaction
       * @author Frederic Py
       * @sa class tr_unprovide
       */
      class tr_provide:public tl_event {
      public:
	tr_provide(factory::argument_type const &arg)
	  :tl_event(arg) {}
	~tr_provide() {}
      private:
	void play() {
	  m_reactor.play_provide(timeline(), goals(), plan());
	}
      }; // TREX::transaction::details::tr_provide

      /** @brief timeline unprovide operation
       *
       * This class describe an Internal timeline undeclaration operation 
       * extracted from the log 
       *
       * @relates LogPlayer
       * @ingroup transaction
       * @author Frederic Py
       * @sa class tr_provide
       */
      class tr_unprovide:public tl_event {
      public:
	tr_unprovide(factory::argument_type const &arg)
	  :tl_event(arg) {}
	~tr_unprovide() {}
      private:
	void play() {
	  m_reactor.play_unprovide(timeline());
	}
      }; // TREX::transaction::details::tr_unprovide

      /** @brief Reactor failure
       *
       * This class describe a reactor failure from the log. The 
       * execution of it wil lresult on the LogPLayer throwing an 
       * exception which will then destroy this reactor within the 
       * agent.
       *
       * @relates LogPlayer
       * @ingroup transaction
       * @author Frederic Py
       */
      class tr_fail :public tr_event {
      public:
	/** @brief Constructor 
	 *
	 * @param[in] arg A xml descriptor
	 *
	 * Create a new instance. The XML is:
	 * @code
	 * <failed/>
	 * @endcode
	 */
	tr_fail(factory::argument_type const &arg) 
	  :tr_event(arg) {}
	/** @brief destructor */
	~tr_fail() {}
      private:
	void play() {
	  throw ReactorException(m_reactor, "Played failed log event.");
	}
      }; // TREX::transaction::details::tr_fail

      /** @brief New observation event 
       *
       * Describe an observation posting event from the log
       *
       * @relates LogPlayer
       * @ingroup transaction
       * @author Frederic Py
       */
      class tr_notify :public tr_event {
      public:
	/** @brief Constructor 
	 * @param[in] arg A XML descriptor
	 *
	 * Create anew instance based on the observation described in @p arg. 
	 * The XML is an XML desciptor for an observation as accepted by 
	 * the XML constructor of the Observation class.
	 *
	 * @sa Observation::Observation(boost::property_tree::ptree::value_type &)
	 */
	tr_notify(factory::argument_type const &arg)
	  :tr_event(arg), m_obs(factory::node(arg)) {}
	/** @brief Destructor */
	~tr_notify() {}
      private:
	void play() {
	  m_reactor.play_obs(m_obs);
	}
	token m_obs;
      }; // TREX::transaction::details::tr_notify

      /** @brief Goal related event base class
       *
       * This class is the base for all the events ambedding a goal_id
       * as an argaument. This includes request/recall events but also
       * plan publication/cancelation
       * 
       * It extract the goal from the log and associate goals created in this 
       * agent to their former id as described by the log allowing to replay 
       * the same goal excahnges despite the goal_id changing at every run.
       *
       * @relates LogPlayer
       * @ingroup transaction
       * @author Frederic Py
       */
      class tr_goal_event :public tr_event {
      protected:
	/** @brief Constructor 
	 * @param[in] arg A xml descriptor
	 * @param[in] build goal parsing flag
	 * 
	 * Create the new goal event using the XML tag in @p arg. Depending 
	 * on @p build the XML structure will differ.
	 * @li if @p build is @c true the structure is:
	 * @code 
	 * < <op_type> id="<log_id>" >
	 * <!-- goal_descriptor -->
	 * </ <op_type> >
	 * @endcode 
	 * And the constructor wil create a new goal using the 
	 * @c goal_descriptor as defined for  the XML constrauctor of the 
	 * class Goal and will then associate to the pseudo id @c <log_id>
	 * @li if @p build is false the structure is just 
	 * @code 
	 * < <op_type> id="<log_id>" />
	 * @endcode
	 * And the constructor will look for the formerly createdted goal
	 * associated to the pseudo id @c <log_id>
	 *
	 * In both cases the @c <op_type> defines the type of operation
	 * and will be assciated to one concrete descendant of this class.
	 *
	 * @sa Goal::Goal(boost::property_tree::ptree::value_type &)
	 */
	tr_goal_event(factory::argument_type const &arg, bool build);
	/** @brief Destructor */
	virtual ~tr_goal_event() {}

	/** @brief goal
	 *
	 * @return the goal associated to this operation
	 */
	token_id const &goal() const {
	  return m_goal;
	}
      private:
	token_id m_goal;
      }; // TREX::transaction::details::tr_goal_event

      /** @brief Request event
       *
       * The event of producing a request
       * 
       * @relates LogPlayer
       * @ingroup transaction
       * @author Frederic Py 
       * @sa tr_recall
       */
      class tr_request :public tr_goal_event {
      public:
	tr_request(factory::argument_type const &arg) 
	  :tr_goal_event(arg, true) {}
	~tr_request() {}
      private:
	void play() { 
	  m_reactor.play_request(goal());
	}
      }; // TREX::transaction::details::tr_request

      /** @brief Recall event
       *
       * The event of recalling a request
       * 
       * @relates LogPlayer
       * @ingroup transaction
       * @author Frederic Py 
       * @sa tr_request
       */
      class tr_recall :public tr_goal_event {
      public:
	tr_recall(factory::argument_type const &arg) 
	  :tr_goal_event(arg, false) {}
	~tr_recall() {}
      private:
	void play() { 
	  m_reactor.play_recall(goal());
	}
      }; // TREX::transaction::details::tr_recall

      /** @brief Plan notification event
       *
       * The event of publishing a plan token
       * 
       * @relates LogPlayer
       * @ingroup transaction
       * @author Frederic Py 
       * @sa tr_plan_cancel
       */
      class tr_plan_add :public tr_goal_event {
      public:
	tr_plan_add(factory::argument_type const &arg) 
	  :tr_goal_event(arg, true) {}
	~tr_plan_add() {}
      private:
	void play() { 
	  m_reactor.play_add(goal());
	}
      }; // TREX::transaction::details::tr_plan_add

      /** @brief Plan cancelation event
       *
       * The event of cancelling a formerly published plan token
       * 
       * @relates LogPlayer
       * @ingroup transaction
       * @author Frederic Py 
       * @sa tr_plan_add
       */
      class tr_plan_cancel :public tr_goal_event {
      public:
	tr_plan_cancel(factory::argument_type const &arg) 
	  :tr_goal_event(arg, false) {}
	~tr_plan_cancel() {}
      private:
	void play() { 
	  m_reactor.play_cancel(goal());
	}
      }; // TREX::transaction::details::tr_plan_cancel

      /** @brief Work status update event
       *
       * This event is used to emulate the work requests from 
       * the reactor. It will set the work flag of the reactor 
       * to the corresponding value allowing it to "consume" 
       * the same deliberation steps as the reactor it replaying.
       * 
       * @ingroup transaction
       * @relates LogPlayer
       * @uthor Frederic Py
       */
      class tr_work :public tr_event {
      public:
	/** @brief Constructor 
	 * 
	 * @param[in] arg A XML descriptor
	 * 
	 * Create a new instance using the XML emebedded in @p arg. 
	 * The XML format is:
	 * @code 
	 *  <work value="<bool>" />
	 * @endcode 
	 * Where @c <bool> indiacates the value to which the reactor log 
	 * flag should be set to (@c 1 if it needs to request for work, 
	 * @c 0 otherwise 
	 */
	tr_work(factory::argument_type const &arg)
	  :tr_event(arg), 
	   m_work(utils::parse_attr<bool>(factory::node(arg), "value")) {}
	/** @brief Destructor */
	~tr_work() {}
      private:
	void play() {
	  m_reactor.set_work(m_work);
	}
	bool m_work;
      };

    } // TREX::transaction::details
  } // TREX::transaction
} // TREX

using namespace TREX::transaction;
using TREX::utils::symbol;

namespace util=TREX::utils;
namespace xml = boost::property_tree::xml_parser;

namespace {
  
  reactor::declare<LogPlayer> decl("LogPlayer");
  
  details::tr_event::factory::declare<details::tr_use> 
    d_use("use");
  details::tr_event::factory::declare<details::tr_unuse> 
    d_unuse("unuse");
  details::tr_event::factory::declare<details::tr_provide> 
    d_provide("provide");
  details::tr_event::factory::declare<details::tr_unprovide> 
    d_unprovide("unprovide");
  details::tr_event::factory::declare<details::tr_fail> 
    d_failed("failed");
  details::tr_event::factory::declare<details::tr_notify> 
    d_notify("Observation");
  details::tr_event::factory::declare<details::tr_request> 
    d_request("request");
  details::tr_event::factory::declare<details::tr_recall> 
    d_recall("recall");
  details::tr_event::factory::declare<details::tr_plan_add> 
    d_token("token");
  details::tr_event::factory::declare<details::tr_plan_cancel> 
    d_cancel("cancel");
  details::tr_event::factory::declare<details::tr_work> 
    d_work("work");

} // ::

/*
 * class TREX::transaction::LogPlayer::phase
 */

LogPlayer::phase::phase(LogPlayer *owner, 
			boost::property_tree::ptree::value_type &node)
  :m_type(node.first) {
  typedef details::tr_event::factory                  tr_fact;
  typedef boost::property_tree::ptree::iterator iter;

  utils::singleton::use<tr_fact> events_f;
  iter pos = node.second.begin();
  tr_fact::iter_traits<iter>::type 
    it = tr_fact::iter_traits<iter>::build(pos, owner);
  SHARED_PTR<details::tr_event> event;
  while( events_f->iter_produce(it, node.second.end(), event) )
    m_events.push_back(event);
}

/*
 * class TREX::transaction::LogPlayer
 */
// statics

symbol const LogPlayer::s_init("init");
symbol const LogPlayer::s_new_tick("start");
symbol const LogPlayer::s_synchronize("synchronize");
symbol const LogPlayer::s_has_work("has_work");
symbol const LogPlayer::s_step("step");

reactor::xml_arg_type &LogPlayer::alter_cfg(reactor::xml_arg_type &arg) {
  // force logging to false
  utils::set_attr(reactor::xml(arg), "log", false);
  return arg;
}

// structors 

LogPlayer::LogPlayer(reactor::xml_arg_type arg)
  :reactor(arg, false, false) {
  std::string 
    file_name = utils::parse_attr<std::string>(name().str()+".tr.log",
                                               reactor::xml(arg),
					"file");
  bool found;
  file_name = manager().use(file_name, found).string();
  if( !found ) {
    syslog(null, error)<<"Unable to locate transaction log \""
		       <<file_name<<"\".";
    throw ReactorException(*this, 
			   "Unable to locate specified transaction log file.");
  }
  boost::property_tree::ptree pt;
  read_xml(file_name, pt, xml::no_comments|xml::trim_whitespace);
  
  if( pt.empty() ) {
    syslog(null, error)<<"Transaction log \""<<file_name<<"\" is empty.";
    throw ReactorException(*this, "Empty transaction log file.");
  }
  if( pt.size()!=1 ) {
    syslog(null, error)<<"Transaction log \""<<file_name
		       <<"\" has multiple xml trees.";
    throw ReactorException(*this, "Invalid transaction log file.");
  }
  if( pt.front().first!="Log" )
    syslog(null, warn)<<"root tag \""<<pt.front().first<<"\" is not Log.";
  pt = pt.front().second;

  // Play the header
  boost::property_tree::ptree::assoc_iterator i, last;
  boost::tie(i, last) = pt.equal_range("header");
  if( last!=i ) {
    typedef details::tr_event::factory                  tr_fact;
    typedef boost::property_tree::ptree::iterator iter;
    utils::singleton::use<tr_fact> event_f;
    iter pos = i->second.begin();
    LogPlayer *me = this;
    tr_fact::iter_traits<iter>::type
      it = tr_fact::iter_traits<iter>::build(pos, me);
    SHARED_PTR<details::tr_event> event;
    while( event_f->iter_produce(it, i->second.end(), event) )
      event->play();
  }
  
  // Load the ticks
  boost::tie(i, last) = pt.equal_range("tick");
  bool first = true;
  for( ; last!=i; ++i) {
    TICK cur = utils::parse_attr<TICK>(*i, "value");
    for(boost::property_tree::ptree::iterator j=i->second.begin();
	i->second.end()!=j; ++j) {
      if( s_init==j->first ) {
	if( !first )
          throw boost::property_tree::ptree_bad_data(s_init.str()+" tag can only be the first phase.", *j);
      } else if( "<xmlattr>"==j->first )
	continue;
      else if( s_new_tick!=j->first &&
	       s_synchronize!=j->first &&
	       s_has_work!=j->first &&
	       s_step!=j->first ) {
	syslog(null, warn)<<"Skipping unknown phase \""<<j->first<<"\".";
	continue;
      }	
      first = false;
      SHARED_PTR<phase> p(new phase(this, *j));
      m_log.push_back(std::make_pair(cur, p));
    }
  }
  if( m_log.empty() )
    syslog(null, warn)<<" this reactor has no event to play.";
  else 
    syslog(null, info)<<"Loaded "<<m_log.size()<<" phases from tick "
		      <<m_log.front().first<<" to tick "
		      <<m_log.back().first;
  m_goal_map.clear();
}

LogPlayer::~LogPlayer() {
} 

// manipulators

bool LogPlayer::next_phase(TICK tck, utils::symbol const &kind) {
  if( !m_log.empty() ) {
    if( m_log.front().first==tck  ) {
      SHARED_PTR<phase> nxt = m_log.front().second;
      if( nxt->type()==kind ) {
	m_log.pop_front();
	nxt->execute();
	return true;
      } 
    }
  } else
    syslog(warn)<<"No more phase to replay (tick="<<tck<<')';
  return false;
}

bool LogPlayer::in_tick(TICK tck) const {
  return !m_log.empty() && m_log.front().first==tck;
}

// callbacks

void LogPlayer::handle_init() {
  m_inited = next_phase(current_tick(), s_init);
}

void LogPlayer::handle_tick_start() {
  if( !m_inited ) 
    handle_init(); // try to emulate the reactor staring at a later tick
  if( !next_phase(current_tick(), s_new_tick) ) {
    size_t skipped =0;
    std::ostringstream oss;
    while( !m_log.empty() && m_log.front().first<current_tick() ) {
      oss<<"\n\t- ["<<m_log.front().first<<"]: "
	 <<m_log.front().second->type();
      size_t n = m_log.front().second->execute();
      if( n>0 ) 
	oss<<"( "<<n<<" events late)";	
      m_log.pop_front();
      ++skipped;
    }
    if( skipped>0 ) {
      syslog(null, warn)<<"Replayed "<<skipped
			<<" past events after the tick started !!!:"<<oss.str();
      m_inited = true;
      next_phase(current_tick(), s_new_tick);
    } 
  } else 
    m_inited = true;
}

bool LogPlayer::synchronize() {
  if( next_phase(current_tick(), s_synchronize) )
    m_inited = true;
  return true;
}

bool LogPlayer::has_work() {
  TICK cur = current_tick();
  
  if( !next_phase(cur, s_has_work) ) {
    if( m_inited && in_tick(cur) )
      syslog(null, warn)<<"Next logged phase is not has_work !!!";
    m_work = false;
  } else 
    m_inited = true;
  return m_work;
}

void LogPlayer::resume() {
  if( next_phase(current_tick(), s_step) )
    m_inited = true;
}


// events 
void LogPlayer::play_use(utils::symbol const &tl, bool goals, bool plan) {
  use(tl, goals, plan);
}
 
void LogPlayer::play_unuse(utils::symbol const &tl) {
  unuse(tl);
}

void LogPlayer::play_provide(utils::symbol const &tl, bool goals, bool plan) {
  provide(tl, goals, plan);
}

void LogPlayer::play_unprovide(utils::symbol const &tl) {
  unprovide(tl);
}

void LogPlayer::play_obs(token const &obs) {
  post_observation(obs);
}

void LogPlayer::play_request(token_id const &g) {
  post_goal(g);
}

void LogPlayer::play_recall(token_id const &g) {
  post_recall(g);
}

void LogPlayer::play_add(token_id const &g) {
  post_plan_token(g);
}

void LogPlayer::play_cancel(token_id const &g) {
  cancel_plan_token(g);
}

using namespace TREX::transaction::details;

/*
 * class TREX::transaction::details::tr_event
 */

// structors 

tr_event::tr_event(tr_event::factory::argument_type const &arg)
  :m_reactor(*(arg.second)) {}

// manipulators

token_id tr_event::get_goal(std::string const &key) {
  std::map<std::string, token_id>::const_iterator
    i =  m_reactor.m_goal_map.find(key);
  if( m_reactor.m_goal_map.end()!=i )
    return i->second;
  return token_id();
}


void tr_event::set_goal(std::string const &key, token_id const &g) {
  std::map<std::string, token_id>::iterator i;
  bool inserted;
  boost::tie(i, inserted) = m_reactor.m_goal_map.insert(std::make_pair(key, g));
  if( !inserted )
    i->second = g;
}


/*
 * class TREX::transaction::details::tl_event
 */

tl_event::tl_event(tl_event::factory::argument_type const &arg) 
  :tr_event(arg), 
   m_timeline(utils::parse_attr<std::string>(factory::node(arg), "name")),
   m_goals(utils::parse_attr<bool>(true, factory::node(arg), "goals")),
   m_plan(utils::parse_attr<bool>(true, factory::node(arg), "plan")) {}

/*
 * class TREX::transaction::details::tr_goal_event
 */

tr_goal_event::tr_goal_event(tr_goal_event::factory::argument_type const &arg, 
			     bool build)
  :tr_event(arg) {
  std::string id = utils::parse_attr<std::string>(factory::node(arg), "id");
  
  if( id.empty() )
    throw boost::property_tree::ptree_bad_data("id attribute is empty.",
                                               factory::node(arg));
  
  if( build ) {
    boost::property_tree::ptree::assoc_iterator
      desc = factory::node(arg).second.find("Goal");
    if( factory::node(arg).second.not_found()==desc )
      throw boost::property_tree::ptree_bad_data("Unable to find token description.", factory::node(arg));
    m_goal.reset(new token(*desc));
    set_goal(id, m_goal);
  } else {
    m_goal = get_goal(id);
    if( !m_goal )
      throw boost::property_tree::ptree_bad_data(
		     "Unable to find goal for id \""+id+"\".",
                                                 factory::node(arg));
  }
}
