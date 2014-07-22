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
#include "Witre.hh"

#include <trex/utils/xml_utils.hh>
#include <boost/algorithm/string.hpp>
#include <trex/transaction/reactor_graph.hh>
#include <boost/graph/breadth_first_search.hpp>
#include <boost/graph/adjacency_list.hpp>

#include <cstring>

using namespace TREX::witre;
using namespace TREX;

using TREX::utils::symbol;

namespace tr = TREX::transaction;
namespace xml = boost::property_tree::xml_parser;

/*
 * class TREX::witre::WitreServer
 */


// structors

WitreServer::WitreServer(tr::reactor::xml_arg_type arg)
:tr::reactor(arg, false),
graph::timelines_listener(std::make_pair(arg.get<0>(), arg.get<1>())),
 m_server(NULL), m_entry(NULL) {
   // Attempt to locate wt_config.xml first
   bool found;
   boost::filesystem::path config = m_log->use("wt_config.xml", found), 
     path = file_name("Wt.log");
   
   if( found ) {
     boost::property_tree::ptree cfg;
     read_xml(config.string(), cfg);
     if( cfg.empty() )
       throw Error("XML document "+config.string()+" is empty.");
     cfg.put("server.application-settings.log-file", path.string());
     config = m_log->log_file("cfg/wt_trex.xml");
     write_xml(config.string(), cfg);
     
     m_server = new Wt::WServer("", config.string());
   } else {
     m_server = new Wt::WServer("");
   }
   // set server configuration arguments
   config = m_log->use("witre.xml", found);
   
  if( !found )
    throw Error("Unable to locate witre.xml server configuration");
  try {
    boost::property_tree::ptree doc;
    read_xml(config.string(), doc, xml::no_comments|xml::trim_whitespace);

    if( doc.empty() )
      throw Error("XML document "+config.string()+" is empty");
    std::string arg = doc.get<std::string>("config.server");

    std::vector<std::string> args;
    boost::algorithm::split(args, arg, boost::algorithm::is_space());
    args.push_back("--accesslog="+m_log->log_file("wt_access.log").string());

    size_t argc = args.size();
    char **argv = NULL;
    argv = new char*[argc+1];
    argv[0] = strdup(name().str().c_str());
    
    for(size_t i=0; i<argc; ++i) {
      argv[i+1] = strdup(args[i].c_str());
      m_log->syslog("witre.server", null)<<"argv["<<(i+1)<<"] "<<argv[i+1];
    }
    m_server->setServerConfiguration(argc, argv);

    if( NULL!=argv )
      delete[] argv;

    m_server->addEntryPoint(Wt::Application, boost::bind(createWitre, _1, this));
    if( !m_server->start() )
      throw Error("Unable to start the server");
    
    // m_log_conn = manager().on_new_log(log_proxy(*this));
    
  } catch(Wt::WServer::Exception const &e) {
    m_log->syslog("witre.server", error)<<"Server initialization ERROR : "
					<<e.what();
    throw Error(e.what());
  }
}

WitreServer::~WitreServer() {
  // Disconnect from the log events 
  // m_log_conn.disconnect();
  try {
    // destroy the server
    if( NULL!=m_server )
      delete m_server;
  } catch(Wt::WServer::Exception const &e) {
    m_log->syslog("witre.server", error)<<"Error during shutdown : "<<e.what();
    throw Error(e.what());
  }
}

// accessors

// WitreApplication const &WitreServer::app() const {
//   return *m_app;
// }

// WitreApplication &WitreServer::app() {
//   return *m_app;
// }

Wt::WServer &WitreServer::wt() {
  return *m_server;
}

Wt::WServer const &WitreServer::wt() const {
  return *m_server;
}

// wt functions

void WitreServer::connect(WitreApplication *client, const boost::function<void()>& function)
{
    boost::mutex::scoped_lock lock(mutex_);
    connections.push_back(Connection(Wt::WApplication::instance()->sessionId(), client, function));
}

void WitreServer::disconnect(WitreApplication *client)
{
    boost::mutex::scoped_lock lock(mutex_);
    for (unsigned i = 0; i < connections.size(); ++i) {
      if (connections[i].client == client) {
        connections.erase(connections.begin() + i);
        return;
          }
        }

        assert(false);
}

std::string WitreServer::getDependencies(std::string name)
{
    WitrePaintSearch::GraphMap::iterator it;
    boost::mutex::scoped_lock lock(mutex_);
    for(it=timelineGraph.begin(); it!=timelineGraph.end(); it++)
    {
        if((*it).first==name)
        {
            std::list<utils::symbol>::iterator connection;
            std::stringstream msg;
            for(connection = (*it).second.connections.begin(); connection!=(*it).second.connections.end(); connection++)
            {
                msg<<((connection!=(*it).second.connections.begin())?"&&":"")<<*connection;
            }
            return msg.str();
        }
    }
    return "";
}

/*
 * by deriving from graph::timelines_listener I can now be informed
 * from new timeline created in the agent at any time
 */
void WitreServer::declared(tr::details::timeline const &timeline) {
  if( !is_external(timeline.name()) ) {
    pendingTimelines.insert(timeline.name());

 //    // If I did not connect to it yet just create the connection
//     use(timeline.name(), true, true); // as we do not have internal timlines this operation will always succeed
//                                        //  the first boolean means that we may want to post goals
//                                        //  the second boolean indicate if we want to be notified on plan updates
//                                        //      (false for now)
    {
      // then add it to externalTimelines
      boost::mutex::scoped_lock lock(mutex_);
      externalTimelines.push_back(timeline.name());
    }

    searchGraph();
    std::string name = timeline.name().str();
    //Updates each client with the new timeline name
    for (unsigned i = 0; i < connections.size(); ++i)
    {
        boost::mutex::scoped_lock lock(mutex_);
        Connection& c = connections[i];
        const boost::function<void(std::string)>& function(boost::bind(&WitreApplication::addTimeline, c.client, _1));
        Wt::WServer::instance()->post(c.sessionId, boost::bind(function, name) );
    }
  }
}

/*
 * conversely I am notified from timelines not being owned by any reactor
 * anymore ... but we do not have that much to do here for now
 */
void WitreServer::undeclared(tr::details::timeline const &timeline)
{
  pendingTimelines.erase(timeline.name());
}

void WitreServer::handle_init()
{
    graph::timelines_listener::initialize();
    searchGraph();
}

void WitreServer::handle_tick_start() {
  std::set<utils::symbol> subscribe;
  std::swap(subscribe, pendingTimelines);

  for(std::set<utils::symbol>::const_iterator i=subscribe.begin();
      subscribe.end()!=i; ++i) 
    use(*i, true, true);

}

//Getting the timeline Graph dependencies
void WitreServer::searchGraph()
{
    timelineGraph.clear();
    WitrePaintSearch vis2(timelineGraph);
    boost::mutex::scoped_lock lock(mutex_);
    boost::depth_first_search(get_graph(), boost::visitor(vis2));
}

void WitreServer::notify(token const &obs)
{
    boost::mutex::scoped_lock lock(mutex_);
    // time_t now = (tickToTime(getCurrentTick())-boost::posix_time::from_time_t(0)).total_seconds();
    std::ostringstream oss;
    oss <<"<Token tick=\""<<current_tick()<<"\" on=\""
        <<obs.object()<<"\" pred=\""<<obs.predicate()<<"\""
        <<" level=\""<<timelineGraph.find(obs.object())->second.level<<"\" "
        <<">"<<obs<<"</Token>";
    //Storing the observation
    observations.push(oss.str());
    /* This is where we notify all connected clients. */
    for (unsigned i = 0; i < connections.size(); ++i) {
        Connection& c = connections[i];
        c.client->addObs(oss.str());
        Wt::WServer::instance()->post(c.sessionId, c.function);
    }

}

void WitreServer::log_proxy::operator()(TREX::utils::log::entry::pointer msg) {
  if( msg->kind()!=TREX::utils::log::null ) {
    boost::mutex::scoped_lock lock(me.mutex_);
    boost::optional<unsigned long long> date;
    if( msg->is_dated() ) 
      date = msg->date();
    for(unsigned i=0; i<me.connections.size(); ++i) {
      Connection &c = me.connections[i];
      c.client->addLog(date, msg->source().str(), msg->kind().str(), 
                       msg->content());
      // Wt::WServer::instance()->post(c.sessionId, c.function);
    }
  }
  
}

bool WitreServer::synchronize()
{
    boost::mutex::scoped_lock lock(mutex_);
  TREX::transaction::TICK date = current_tick();
    std::string now = date_str(date+1);
  // time_t now = std::floor(tickToTime(getCurrentTick()+1));
    std::ostringstream oss;
    oss<<"<Token tick=\""<<date<<"\" on=\"Tick\">"<<now<<" ("<<(date+1)<<")</Token>";
    //Storing ticks
    observations.push(oss.str());
    //Notify all connected clients
    for (unsigned i = 0; i < connections.size(); ++i) {
            Connection& c = connections[i];
            c.client->addObs(oss.str());
            Wt::WServer::instance()->post(c.sessionId, c.function );
    }
    //Notify all connected clients of plan
    dispatchPlanTokens();
    return true;
}

void WitreServer::new_plan_token(token_id const &t)
{
    planTokens.remove(t);
    planTokens.push_back(t);
    dispatchPlanTokens();
}

void WitreServer::cancelled_plan_token(token_id const &t)
{

}

void WitreServer::dispatchPlanTokens()
{
    std::list<timed_goal::iterator> eraselist;
    timed_goal::iterator plan;
    for(plan = planTokens.begin(); plan!=planTokens.end(); ++plan)
    {
        token_id const& t = (*plan);
        if(t->end().upper_bound()<current_tick())
        {
            pastTokens.push_front((*plan));
            eraselist.push_back(plan);
        }
    }
    while(!eraselist.empty())
    {
        planTokens.erase(eraselist.front());
        eraselist.pop_front();
    }
    for (unsigned i = 0; i < connections.size(); ++i) {
        Connection& c = connections[i];
        const boost::function<void(const timed_goal&)>& function(boost::bind(&WitreApplication::newPlanToken, c.client, _1));
        Wt::WServer::instance()->post(c.sessionId, boost::bind(function, planTokens) );
    }
}

token_id WitreServer::clientGoalPost(boost::property_tree::ptree::value_type const &xml) {
  boost::mutex::scoped_lock lock(mutex_);
  token_id g = parse_goal(xml);
  if( post_goal(g) )
    return g;
  return token_id();
}


token_id WitreServer::clientGoalPost(token const &g)
{
    boost::mutex::scoped_lock lock(mutex_);
    if(is_external(g.object()))
    {
        token_id id = post_goal(g);
        return id;
    }
    return token_id();
}

TREX::transaction::token WitreServer::getGoal(std::string obs, std::string prd)
{
    boost::mutex::scoped_lock lock(mutex_);
    TREX::transaction::token clientGoal(obs,prd);
    return clientGoal;
}


// Modifiers

bool WitreServer::attach(WitreReactor &r) {
  if( NULL!=m_entry )
    m_entry = &r;
  return &r==m_entry;
}

void WitreServer::detach(WitreReactor &r) {
  if( &r==m_entry )
    m_entry = NULL;
}
