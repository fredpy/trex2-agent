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
#include "WitreApp.hh"

#include <trex/agent/RealTimeClock.hh>

using namespace TREX::witre;

namespace {
  
  typedef long long tick_type;
  
  class Msg;
  
  class MsgType {
  public:
    std::string identifier;
    Wt::Dbo::collection< Wt::Dbo::ptr<Msg> > messages;
    
    template<class Action>
    void persist(Action &a) {
      Wt::Dbo::field(a, identifier, "name");
      Wt::Dbo::hasMany(a, messages, Wt::Dbo::ManyToOne, "type");
    }
    
  };
  
  class Source {
  public:
    std::string name;
    Wt::Dbo::collection< Wt::Dbo::ptr<Msg> > messages;
    
    template<class Action>
    void persist(Action &a) {
      Wt::Dbo::field(a, name, "name");
      Wt::Dbo::hasMany(a, messages, Wt::Dbo::ManyToOne, "source");
    }
  };
  
  class Msg {
  public:
    // Wyt::Dbo does not like unsigneds ...
    boost::optional<tick_type> date;
    Wt::Dbo::ptr<Source> source;
    Wt::Dbo::ptr<MsgType> type;
    std::string content; 
    
    template<class Action>
    void persist(Action &a) {
      Wt::Dbo::field(a, date, "tick");
      Wt::Dbo::belongsTo(a, source, "source");
      Wt::Dbo::belongsTo(a, type, "type");
      Wt::Dbo::field(a, content, "content");
    }
  };
  
  
}

/*
 * class TREX::witre::Server
 */

// structors 

Server::Server(Wt::WServer &server)
:m_server(server), m_log_db(":memory:"), m_reserved(false), m_stop(false) {
  // for debugging purpose
  m_log_db.setProperty("show-queries", "true");

  // create our session
  init_session();
  // ensure that log is initialized
  m_log->add_handler(log_proxy(*this));
  m_log->logPath();
  
  m_log->syslog("witre", TREX::utils::info)<<"witre server loaded.\n\tWelcome to T-REX";
}

Server::~Server() {
}

// observers

std::string Server::name() const {
  boost::recursive_mutex::scoped_lock lck(m_mutex);
  if( NULL!=m_agent.get() ) {
    return m_agent->getName().str();
  } 
  return "";
}

Wt::Dbo::QueryModel<Server::entry_fields> *Server::log_model() {
  Wt::Dbo::Query<entry_fields>
    rq = m_log_session.query<entry_fields>("select L.tick, S.name, T.name, L.content from log_msg L, log_src S, log_type T").where("L.source_id = S.id").where("L.type_id = T.id").orderBy("L.id DESC");
  Wt::Dbo::QueryModel<entry_fields> *query = new  Wt::Dbo::QueryModel<entry_fields>;
      
  query->setQuery(rq);
  query->addAllFieldsAsColumns();
  
  return query;
}

bool Server::reserved() const {
  boost::recursive_mutex::scoped_lock lck(m_mutex);
  return m_reserved || NULL!=m_agent.get();
}

bool Server::completed() const {
  boost::recursive_mutex::scoped_lock lck(m_mutex);
  return NULL==m_agent.get();
}

// modifiers

void Server::init_session() {
  m_log_session.setConnection(m_log_db);
  m_log_session.mapClass<Source>("log_src");
  m_log_session.mapClass<MsgType>("log_type");
  m_log_session.mapClass<Msg>("log_msg");
  m_log_session.createTables();
}

void Server::log_entry(boost::optional<long long> const &tick,
                       std::string const &src, std::string const &type,
                       std::string const &msg) {
  /*
   * Note: as this is a log message handler DO NOT put syslog calls in here 
   * otherwise it would result on this handler producing an infinite amount 
   * of messages.
   */
  try {
    Wt::Dbo::ptr<Source>  src_db;
    Wt::Dbo::ptr<MsgType> type_db;
    {
      Wt::Dbo::Transaction transaction(m_log_session);
    
      // Check or create the source 
      src_db = m_log_session.find<Source>().where("name = ?").bind(src);
      if( !src_db ) {
        Source *src_v = new Source;
        src_v->name = src;
        src_db = m_log_session.add(src_v);
      }
      transaction.commit();
    }
  
    {
      Wt::Dbo::Transaction transaction(m_log_session);

      // Check or create type
      type_db = m_log_session.find<MsgType>().where("name = ?").bind(type);
      if( !type_db ) {
        MsgType *type_v = new MsgType;
        type_v->identifier = type;
        type_db = m_log_session.add(type_v);
      }
      transaction.commit();
    }
    Wt::Dbo::ptr<Msg> msg_db;
    {
      Wt::Dbo::Transaction transaction(m_log_session);
      // Now I can add the new message
      Msg *entry = new Msg;
      entry->date = tick;
      entry->source = src_db;
      entry->type = type_db;
      entry->content = msg;
      msg_db = m_log_session.add(entry);
    
      transaction.commit();
    }
    emit(&WitreApp::log_updated);
  } catch(Wt::Dbo::Exception const &e) {
    std::cerr<<" >>> Dbo exception: "<<e.what()<<std::endl;
  }
}

bool Server::stop() const {
  boost::recursive_mutex::scoped_lock lck(m_mutex);
  return m_stop;
}


bool Server::reserve(bool flag) {
  {
    boost::recursive_mutex::scoped_lock lck(m_mutex);
    if( m_reserved==flag ) 
      return false;
    
    m_reserved = flag;
  }
  emit(boost::bind(&WitreApp::agent_updated, _1));
  return true;
}

void Server::create_agent(std::string const &cfg) {
  if( cfg.empty() )
    throw TREX::utils::Exception("Empty config file name.");
  else {
    if( NULL!=m_agent_thread.get() ) {
      m_agent_thread->join();
    }
    
    m_agent.reset(new agent::Agent(cfg));
    m_agent->setClock(new agent::RealTimeClock(boost::chrono::seconds(1)));
    
    // Last thing to do is to spawn a new thread to run this agent
    m_agent_thread.reset(new boost::thread(agent_runner(this)));
  }
}

bool Server::connect(WitreApp *cli) {
  connect_map::scoped_lock lock(m_connections);
  
  Connection tmp(Wt::WApplication::instance()->sessionId());
  return m_connections->insert(std::make_pair(cli, tmp)).second;
}

void Server::disconnect(WitreApp *cli) {
  connect_map::scoped_lock lock(m_connections);
  m_connections->erase(cli);
}

