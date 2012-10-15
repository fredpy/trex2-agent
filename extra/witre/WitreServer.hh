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
#ifndef TREX_witre_server
# define TREX_witre_server

# include "LogProxy.hh"
# include <Wt/Wserver>
# include <Wt/Dbo/backend/Sqlite3>

namespace TREX {
  namespace witre {
    
    
    class WitreServer :boost::noncopyable {
    public:      
      WitreServer(Wt::WServer &server);
      ~WitreServer();
      
      LogProxy::log_signal &new_log();
      /** @brief Collect log types
       * @param[out] types A set of message types
       *
       * This method extract from the trex log database all the exisiting
       * messages types and insert them in @p types
       *
       * @return The number of new elements added to @p types
       */
      size_t log_types(std::set<std::string> &types);
      
      typedef boost::tuple<std::string, boost::optional<long long>, std::string, std::string> msg;
      typedef std::list<msg> msg_set;
      
      msg_set last_messages(size_t count);
      
      Wt::WServer &server() {
        return m_server;
      }
      
      boost::filesystem::path locales(std::string const &default_file);
       
    private:
      boost::filesystem::path locale_path(std::string const &file);
      
      
      void init_session();
      
      Wt::WServer &m_server;
      utils::SingletonUse<utils::LogManager> m_log;
      
      void reset_log(LogProxy *ref=NULL);
      LogProxy *m_log_proxy;
      boost::filesystem::path m_locales;
      
      Wt::Dbo::backend::Sqlite3 m_log_db;
      Wt::Dbo::Session          m_log_session;

      mutable boost::recursive_mutex      m_mutex;      
      friend class LogProxy;
    }; // WitreServer
    
    
    
//    class WitreApp;
//    
//    class Server :boost::noncopyable {
//    public:
//      typedef long long tick_type;
//      typedef boost::tuple< boost::optional<tick_type>, std::string, 
//                            std::string, std::string > entry_fields;
//
//      
//      Server(Wt::WServer &server);
//      ~Server();
//
//      std::string name() const; 
//      void create_agent(std::string const &cfg);
//      
//      Wt::Dbo::QueryModel<entry_fields> *log_model();
//      
//      bool reserve(bool flag=true);
//      bool reserved() const;
//
//      bool completed() const;
//      
//      utils::internals::LogEntry info() const {
//        return m_log->syslog("", utils::info);
//      }
//      
//      bool connect(WitreApp *cli);
//      void disconnect(WitreApp *cli);
//      
//      void new_timeline(utils::Symbol const &agent, utils::Symbol const &tl);
//      void graph_updated(utils::Symbol const &agent);
//      
//    private:
//      struct Connection {
//        explicit Connection(std::string const &id)
//          :session_id(id) {}
//        
//        template<class Fn> 
//        void post(Wt::WServer *serv, Fn const &fn) const {
//          serv->post(session_id, fn);
//        }
//        std::string session_id;
//      };
//            
//      
//      typedef std::map<WitreApp *, Connection> conn_map;
//      typedef utils::SharedVar<conn_map> connect_map; 
//      
//      connect_map m_connections;
//      
//      
//      template<class Fn>
//      void emit(Fn const &fn) {
//        connect_map::scoped_lock lock(m_connections);
//        
//        for(conn_map::const_iterator i=m_connections->begin();
//            m_connections->end()!=i; ++i) {
//          i->second.post(Wt::WServer::instance(), boost::bind(fn, i->first));
//        }
//      }
//      
//      
//      Wt::WServer               &m_server;
//      Wt::Dbo::backend::Sqlite3  m_log_db;
//      Wt::Dbo::Session           m_log_session;
//      
//      class log_proxy:public TREX::utils::TextLog::handler {
//      public:
//        explicit log_proxy(Server &creator):me(&creator) {}
//        log_proxy(log_proxy const &other):me(other.me) {}
//        ~log_proxy() {}
//        
//      private:
//        void message(boost::optional<date_type> const &date, 
//                     id_type const &who, id_type const &kind,
//                     msg_type const &what) {
//          if( NULL!=me ) {
//            boost::optional<long long> tick;
//            std::string str_who, str_what, str_msg;
//            if( date )
//              tick = *date;
//            str_who = who.str();
//            str_what = kind.str();
//            str_msg = what;
//            me->log_entry(tick, str_who, str_what, str_msg);
//          }
//        }
//        
//        Server *me;
//      };
//      
//      
//      friend class log_proxy;
//      
//      void init_session();
//      void log_entry(boost::optional<long long> const &tick,
//                     std::string const &src, std::string const &type,
//                     std::string const &msg);
//      
//      mutable boost::recursive_mutex      m_mutex;
//      bool m_reserved, m_stop;
//      
//      bool stop() const;
//
//            
//      struct agent_runner {
//      public:
//        agent_runner(Server *creator) :me(creator) {}
//        agent_runner(agent_runner const &other):me(NULL) {
//          std::swap(me, other.me);
//        }
//        ~agent_runner() {
//          if( NULL!=me ) {
//            boost::recursive_mutex::scoped_lock lck(me->m_mutex);
//            me->m_agent.reset();
//          }
//        }
//        
//        bool completed() const {
//          if( NULL==me || me->stop() )
//            return true;
//          else {
//            boost::recursive_mutex::scoped_lock lck(me->m_mutex);
//            return me->m_agent->missionCompleted();
//          }
//        }
//        
//        void operator()() {
//          try {
//            if( NULL!=me ) {
//              me->m_agent->initComplete();
//              while( !completed() )
//                me->m_agent->doNext();
//            }
//          } catch(utils::Exception const &e) {
//            std::cerr<<"TREX exception : "<<e<<std::endl;
//          } catch(std::exception const &se) {
//            std::cerr<<"C++ exception : "<<se.what()<<std::endl;
//          } catch(...) {
//            std::cerr<<"Unknwon exception."<<std::endl;
//          }
//        }
//      private:
//        mutable Server *me;
//      };
//      friend struct agent_runner;
//      
//      std::auto_ptr<agent::Agent> m_agent;
//      std::auto_ptr<boost::thread> m_agent_thread;
//      TREX::utils::SingletonUse<TREX::utils::LogManager> m_log;
//    }; // TREX::witre::Server

  } // TREX::witre
} // TREX

#endif // TREX_witre_server
