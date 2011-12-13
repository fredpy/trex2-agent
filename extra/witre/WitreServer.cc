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

#include <trex/utils/XmlUtils.hh>
#include <boost/algorithm/string.hpp>

#include <cstring>

using namespace TREX::witre;
using namespace TREX::transaction;
using namespace TREX::utils;

namespace xml = boost::property_tree::xml_parser;

/*
 * class TREX::witre::WitreServer
 */


// structors

WitreServer::WitreServer(TeleoReactor::xml_arg_type arg):TeleoReactor(arg), m_server(NULL),
			   m_entry(NULL) {
  bool found;
  std::string config = m_log->use("witre.xml", found);
  if( !found )
    throw Error("Unable to locate witre.xml server configuration");
  try {
    /* old code :
     * rapidxml::file<> cfg(config.c_str());
     * rapidxml::xml_document<> doc;
     * doc.parse<0>(cfg.data());
     * rapidxml::xml_node<> *root = doc.first_node(), *child;
     *
     * if( NULL==root )
     * throw Error("Unable to parse xml content of "+config);
     *
     * child = root->first_node("server");
     * if( NULL==child )
     * throw TREX::utils::XmlError(*root, "Unable to find \"server\" configuration tag");
     *
     * std::string arg(child->value(), child->value_size());
     */
    // new code
    boost::property_tree::ptree doc;
    read_xml(config, doc, xml::no_comments|xml::trim_whitespace);

    if( doc.empty() )
      throw Error("XML document "+config+" is empty");
    std::string arg = doc.get<std::string>("config.server");
    // end new code

    std::vector<std::string> args;
    boost::algorithm::split(args, arg, boost::algorithm::is_space());

    size_t argc = args.size();
    char **argv = NULL;
    argv = new char*[argc+1];
    argv[0] = strdup(config.c_str());

    for(size_t i=0; i<argc; ++i) {
      m_log->syslog("witre.server")<<"args["<<i<<"] "<<args[i];
      argv[i+1] = strdup(args[i].c_str());
      m_log->syslog("witre.server")<<"argv["<<i<<"] "<<argv[i+1];
    }

    m_server = new Wt::WServer(config);

    m_server->setServerConfiguration(argc, argv);
    if( NULL!=argv )
      delete[] argv;

    m_server->addEntryPoint(Wt::Application, boost::bind(createWitre, _1, this));
    if( !m_server->start() )
      throw Error("Unable to start the server");

  } catch(Wt::WServer::Exception const &e) {
    m_log->syslog("witre.server")<<"Server initialization ERROR : "<<e.what();
    throw Error(e.what());
  }
}

WitreServer::~WitreServer() {
  std::cerr<<"Deleting the server ???"<<std::endl;
  try {
    // destroy the server
    if( NULL!=m_server )
      delete m_server;
  } catch(Wt::WServer::Exception const &e) {
    m_log->syslog("WitreServer")<<"Error during shutdown : "<<e.what();
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

void WitreServer::handleInit()
{
    boost::mutex::scoped_lock lock(mutex_);
    external_iterator first(ext_begin());
    for(;first!=ext_end(); ++first)
    {
        externalTimelines.push_back(first->name());
    }

    return;
}

void WitreServer::notify(Observation const &obs)
{
    boost::mutex::scoped_lock lock(mutex_);
    time_t now = std::floor(tickToTime(getCurrentTick()));
    std::ostringstream oss;
    oss <<"<Token tick=\""<<getCurrentTick()<<"\" on=\""
        <<obs.object()<<"\" pred=\""<<obs.predicate()<<"\">"
        <<ctime(&now)<<": "<<obs<<"</Token>";
    std::ostringstream time;
    time << getCurrentTick();
    //Storing the observation
    Observations* temp = new Observations(oss.str(), obs.object().str(), time.str());
    observations.push(*temp);
    /* This is where we notify all connected clients. */
    for (unsigned i = 0; i < connections.size(); ++i) {
        Connection& c = connections[i];
        c.client->addObs(temp);
        Wt::WServer::instance()->post(c.sessionId, c.function);
    }

}

bool WitreServer::synchronize()
{
    boost::mutex::scoped_lock lock(mutex_);
    time_t now = std::floor(tickToTime(getCurrentTick()+1));
    //time(&now);
    //struct tm* tnow = localtime(&now);
    //std::cout<<timeToTick(now)<<std::endl;
    std::ostringstream oss;
    oss<<"<Tick value=\""<<(getCurrentTick()+1)<<"\">"<<"Tick Value: "<<now<<"</Tick>";
    //Storing ticks
    Observations* temp = new Observations(oss.str(), "Tick");
    observations.push(*temp);
    //Notify all connected clients
    for (unsigned i = 0; i < connections.size(); ++i) {
            Connection& c = connections[i];
            c.client->addObs(temp);
            Wt::WServer::instance()->post(c.sessionId, c.function );
    }

    return true;
}

goal_id WitreServer::clientGoalPost(Goal const &g)
{
    boost::mutex::scoped_lock lock(mutex_);
    if(isExternal(g.object()))
    {
        goal_id id = postGoal(g);
        return id;
    }
    return goal_id();
}

TREX::transaction::Goal WitreServer::getGoal(std::string obs, std::string prd)
{
    boost::mutex::scoped_lock lock(mutex_);
    TREX::transaction::Goal clientGoal(obs,prd);
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
