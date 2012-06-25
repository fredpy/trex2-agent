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

      class tl_event :public tr_event {
      protected:
	tl_event(factory::argument_type const &arg);
	virtual ~tl_event() {}
	
	utils::Symbol const &timeline() const {
	  return m_timeline;
	}
      private:
	utils::Symbol m_timeline;
      }; // TREX::transaction::details::tl_event
      
      class tr_use:public tl_event {
      public:
	tr_use(factory::argument_type const &arg)
	  :tl_event(arg) {}
	~tr_use() {}
      private:
	void play() {
	  m_reactor.play_use(timeline());
	}
      }; // TREX::transaction::details::tr_use

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
      
      class tr_provide:public tl_event {
      public:
	tr_provide(factory::argument_type const &arg)
	  :tl_event(arg) {}
	~tr_provide() {}
      private:
	void play() {
	  m_reactor.play_provide(timeline());
	}
      }; // TREX::transaction::details::tr_provide

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

      class tr_fail :public tr_event {
      public:
	tr_fail(factory::argument_type const &arg) 
	  :tr_event(arg) {}
	~tr_fail() {}
      private:
	void play() {
	  throw ReactorException(m_reactor, "Played failed log event.");
	}
      }; // TREX::transaction::details::tr_fail

      class tr_notify :public tr_event {
      public:
	tr_notify(factory::argument_type const &arg)
	  :tr_event(arg), m_obs(factory::node(arg)) {}
	~tr_notify() {}
      private:
	void play() {
	  m_reactor.play_obs(m_obs);
	}
	Observation m_obs;
      }; // TREX::transaction::details::tr_notify

      class tr_goal_event :public tr_event {
      protected:
	tr_goal_event(factory::argument_type const &arg, bool build);
	virtual ~tr_goal_event() {}

	goal_id const &goal() const {
	  return m_goal;
	}
      private:
	goal_id m_goal;
      }; // TREX::transaction::details::tr_goal_event

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

      class tr_work :public tr_event {
      public:
	tr_work(factory::argument_type const &arg)
	  :tr_event(arg), 
	   m_work(utils::parse_attr<bool>(factory::node(arg), "value")) {}
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
using namespace TREX::utils;
namespace xml = boost::property_tree::xml_parser;

namespace {
  
  TeleoReactor::xml_factory::declare<LogPlayer> decl("LogPlayer");
  
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

  SingletonUse<tr_fact> events_f;
  iter pos = node.second.begin();
  tr_fact::iter_traits<iter>::type 
    it = tr_fact::iter_traits<iter>::build(pos, owner);
  boost::shared_ptr<details::tr_event> event;
  while( events_f->iter_produce(it, node.second.end(), event) )
    m_events.push_back(event);
}

/*
 * class TREX::transaction::LogPlayer
 */
// statics

Symbol const LogPlayer::s_init("init");
Symbol const LogPlayer::s_new_tick("start");
Symbol const LogPlayer::s_synchronize("synchronize");
Symbol const LogPlayer::s_has_work("has_work");
Symbol const LogPlayer::s_step("step");

TeleoReactor::xml_arg_type &LogPlayer::alter_cfg(TeleoReactor::xml_arg_type &arg) {
  // force logging to false
  set_attr(xml_factory::node(arg), "log", false);
  return arg;
}

// structors 

LogPlayer::LogPlayer(TeleoReactor::xml_arg_type arg)
  :TeleoReactor(arg, false, false) {
  std::string 
    file_name = parse_attr<std::string>(getName().str()+".tr.log",
					xml_factory::node(arg),
					"file");
  bool found;
  file_name = manager().use(file_name, found);
  if( !found ) {
    syslog("ERROR")<<"Unable to locate transaction log \""<<file_name<<"\".";
    throw ReactorException(*this, 
			   "Unable to locate specified transaction log file.");
  }
  boost::property_tree::ptree pt;
  read_xml(file_name, pt, xml::no_comments|xml::trim_whitespace);
  
  if( pt.empty() ) {
    syslog("ERROR")<<"Transaction log \""<<file_name<<"\" is empty.";
    throw ReactorException(*this, "Empty transaction log file.");
  }
  if( pt.size()!=1 ) {
    syslog("ERROR")<<"Transaction log \""<<file_name
		   <<"\" has multiple xml trees.";
    throw ReactorException(*this, "Invalid transaction log file.");
  }
  // Play the header
  boost::property_tree::ptree::assoc_iterator i, last;
  boost::tie(i, last) = xml_factory::node(arg).second.equal_range("header");
  if( last!=i ) {
    typedef details::tr_event::factory                  tr_fact;
    typedef boost::property_tree::ptree::iterator iter;
    SingletonUse<tr_fact> event_f;
    iter pos = i->second.begin();
    LogPlayer *me = this;
    tr_fact::iter_traits<iter>::type
      it = tr_fact::iter_traits<iter>::build(pos, me);
    boost::shared_ptr<details::tr_event> event;
    while( event_f->iter_produce(it, i->second.end(), event) )
      event->play();
  }
  
  // Load the ticks
  boost::tie(i, last) = xml_factory::node(arg).second.equal_range("tick");
  bool first = true;
  for( ; last!=i; ++i) {
    TICK cur = parse_attr<TICK>(*i, "value");    
    for(boost::property_tree::ptree::iterator j=i->second.begin();
	i->second.end()!=j; ++j) {
      if( s_init==j->first ) {
	if( !first ) 
	  throw XmlError(*j, s_init.str()+" tag can only be the first phase.");
      } else if( s_new_tick!=j->first &&
		 s_synchronize!=j->first &&
		 s_has_work!=j->first &&
		 s_step!=j->first ) {
	syslog("WARN")<<"Skiping unknown phase \""<<j->first<<"\".";
	continue;
      }	
      first = false;
      boost::shared_ptr<phase> p(new phase(this, *j));
      m_log.push_back(std::make_pair(cur, p));
    }
  }
}

LogPlayer::~LogPlayer() {
} 

// manipulators

bool LogPlayer::next_phase(TICK tck, utils::Symbol const &kind) {
  if( !m_log.empty() && m_log.front().first==tck  ) {
    boost::shared_ptr<phase> nxt = m_log.front().second;
    if( nxt->type()==kind ) {
      m_log.pop_front();
      nxt->execute();
      return true;
    }
  }
  return false;
}

// callbacks

void LogPlayer::handleInit() {
  m_inited = next_phase(getCurrentTick(), s_init);
}

void LogPlayer::handleTickStart() {
  if( !m_inited ) 
    handleInit(); // try to emulate the reactor staring at a later tick
  if( !next_phase(getCurrentTick(), s_new_tick) ) {
    size_t skipped =0;
    while( !m_log.empty() && m_log.front().first<getCurrentTick() ) {
      m_log.pop_front();
      ++skipped;
    }
    if( skipped>0 ) {
      syslog("WARN")<<"Skipped "<<skipped<<" past events !!!";
      m_inited = true;
      next_phase(getCurrentTick(), s_new_tick);
    } 
  } else 
    m_inited = true;
}

bool LogPlayer::synchronize() {
  if( next_phase(getCurrentTick(), s_synchronize) )
    m_inited = true;
  return true;
}

bool LogPlayer::hasWork() {
  if( !next_phase(getCurrentTick(), s_has_work) ) {
    if( m_inited )
      syslog("WARN")<<"next logged phase is not has_work !!!";
    m_work = false;
  } else 
    m_inited = true;
  return m_work;
}

void LogPlayer::resume() {
  if( next_phase(getCurrentTick(), s_step) )
    m_inited = true;
}


// events 
void LogPlayer::play_use(utils::Symbol const &tl) {
  use(tl);
}
 
void LogPlayer::play_unuse(utils::Symbol const &tl) {
  unuse(tl);
}

void LogPlayer::play_provide(utils::Symbol const &tl) {
  provide(tl);
}

void LogPlayer::play_unprovide(utils::Symbol const &tl) {
  unprovide(tl);
}

void LogPlayer::play_obs(Observation const &obs) {
  postObservation(obs);
}

void LogPlayer::play_request(goal_id const &g) {
  postGoal(g);
}

void LogPlayer::play_recall(goal_id const &g) {
  postRecall(g);
}

void LogPlayer::play_add(goal_id const &g) {
  postPlanToken(g);
}

void LogPlayer::play_cancel(goal_id const &g) {
  cancelPlanToken(g);
}

using namespace TREX::transaction::details;

/*
 * class TREX::transaction::details::tr_event
 */

// structors 

tr_event::tr_event(tr_event::factory::argument_type const &arg)
  :m_reactor(*(arg.second)) {}

// manipulators

goal_id tr_event::get_goal(std::string const &key) {
  std::map<std::string, goal_id>::const_iterator 
    i =  m_reactor.m_goal_map.find(key);
  if( m_reactor.m_goal_map.end()!=i )
    return i->second;
  return goal_id();
}


void tr_event::set_goal(std::string const &key, goal_id const &g) {
  std::map<std::string, goal_id>::iterator i;
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
   m_timeline(parse_attr<std::string>(factory::node(arg), "name")) {}

/*
 * class TREX::transaction::details::tr_goal_event
 */

tr_goal_event::tr_goal_event(tr_goal_event::factory::argument_type const &arg, 
			     bool build)
  :tr_event(arg) {
  std::string id = parse_attr<std::string>(factory::node(arg), "id");
  
  if( id.empty() )
    throw XmlError(factory::node(arg), "id attribute is empty.");
  
  if( build ) {
    boost::property_tree::ptree::assoc_iterator
      desc = factory::node(arg).second.find("Goal");
    if( factory::node(arg).second.not_found()==desc )
      throw XmlError(factory::node(arg), 
		     "Unable to find token description.");
    m_goal.reset(new Goal(*desc));
    set_goal(id, m_goal);
  } else {
    m_goal = get_goal(id);
    if( !m_goal )
      throw XmlError(factory::node(arg),
		     "Unable to find goal for id \""+id+"\".");
  }
}
