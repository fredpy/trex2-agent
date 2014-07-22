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
#include "REST_reactor.hh"
#include "TimelineHistory.hh"
#include "timeline_services.hh"
#include "tick_manager.hh"

#include <trex/utils/trex_version.hh>

#include <boost/algorithm/string.hpp>

using namespace TREX;
using namespace TREX::REST;
using namespace TREX::transaction;

namespace bp=boost::property_tree;
namespace xml=bp::xml_parser;


namespace {
  
  bp::ptree trex_version(rest_request const &req) {
    bp::ptree ret;
    ret.put("version_major", TREX::version::major());
    ret.put("version_minor", TREX::version::minor());
    ret.put("version_release", TREX::version::release());
    if( TREX::version::is_release_candidate() ) {
      ret.put("version_rc", TREX::version::rc_number());
    }
    if( TREX::version::svn_info() ) {
      ret.put("svn_root", TREX::version::svn_root());
      ret.put("svn_rev", TREX::version::svn_revision());
    }
    ret.put("version_str", TREX::version::full_str());
    
    return ret;
  }
  
}

/*
 * class TREX::REST::REST_reactor
 */

// structors 

REST_reactor::REST_reactor(reactor::xml_arg_type arg)
:reactor(arg, false) {
  // Initialize web server
  bool found;
  
  boost::filesystem::path wt_cfg = manager().use("wt_config.xml", found),
    log_dest = file_name("Wt.log");
  boost::optional<std::string> wt_cfg_arg;
  
  
  if( found ) {
    bp::ptree cfg;
    
    xml::read_xml(wt_cfg.string(), cfg);
    if( cfg.empty() )
      throw SYSTEM_ERROR(reactor_error_code(reactor_error::configuration_error),
                         "Wt XML config "+wt_cfg.string()+" is empty");
    // Add log redirection to the file and then save it
    syslog(utils::log::info)<<"redirecting Wt server logs to "<<log_dest.string();
    cfg.put("server.application-settings.log-file", log_dest.string());
    wt_cfg = manager().log_file("cfg/wt_trex.xml");
    xml::write_xml(wt_cfg.string(), cfg);
  
    wt_cfg_arg = wt_cfg.string(); 
    // m_server.reset(new Wt::WServer("", wt_cfg.string()));
  } else {
    syslog(utils::log::warn)<<"Did not find "<<wt_cfg.string()<<": using default Wt server config instead.";
  }
  
  // Now look for the program options
  wt_cfg = manager().use("rest.xml", found);
  
  if( !found )
    throw SYSTEM_ERROR(reactor_error_code(reactor_error::configuration_error),
                       "Failed to locate "+wt_cfg.string()+" required for Wt server arguments");
  // Extarct argument list to be given to the server
  bp::ptree doc;
  xml::read_xml(wt_cfg.string(), doc);
  if( doc.empty() )
    throw SYSTEM_ERROR(reactor_error_code(reactor_error::configuration_error),
                       "Server arguments XML file "+wt_cfg.string()+" is empty");
  
  
  size_t argc;
  char **argv;
  {
    std::string arg_str = doc.get<std::string>("config.server");
    // Now split the command line as a shell would do ... well not really but it will do
    std::vector<std::string> args;
    boost::algorithm::split(args, arg_str, boost::algorithm::is_space());
    // Add extra attribute to make Wt log access into one our log directory
    args.push_back("--accesslog="+file_name("wt_access.log").string());
    
    // Now I can populate argc and argv
    argc = args.size();
    argv = new char*[argc+1];
    
    // Make a fake app name
    argv[0] = strdup(name().str().c_str());
    for(size_t i=0; i<argc; ++i)
      argv[i+1] = strdup(args[i].c_str());
  }
  if( !m_server->is_inited() ) {
    syslog(utils::log::info)<<"Starting web server";
    m_server->init(argc, argv, wt_cfg_arg);
  }
     
  // clean up this bloody malloc mess
  for(size_t i=0; i<=argc; ++i)
    free(argv[i]);
  delete[] argv;
  
  // My server is setup and ready I will start it on handleInit
}

REST_reactor::~REST_reactor() {
  syslog(utils::log::info)<<"Closing Wt REST server";
  try {
    m_timelines.reset();
  } catch(Wt::WServer::Exception const &e) {
    syslog(utils::log::error)<<"Eror during REST server shutdown: "<<e.what();
  }
}

// TREX callbacks

void REST_reactor::handle_init() {
  // First create my timelien observer
  m_timelines.reset(new TimelineHistory(*this));
  m_tick.reset(new tick_manager(get_graph(), manager().service()));
  
  // Build service tree
  m_services.reset(new service_tree(m_server->impl(), "/rest"));
  m_services->add_handler("version",
                          new json_direct(&trex_version,
                                          "Give current trex version"));
  m_tick->populate(*m_services);
  m_services->add_handler("timelines", new timeline_list_service(m_timelines));
  m_services->add_handler("timeline", new timeline_service(m_timelines));
  m_services->add_handler("goals", new goals_service(m_timelines));
  
  boost::filesystem::path recvd = file_name("rest_goal");
  m_services->add_handler("goal", new goal_service(m_timelines, recvd.string()));

  
  
  m_tick->signal().connect(boost::bind(&TimelineHistory::update_tick, m_timelines, _1));
  
 
  if( !m_server->is_running() ) {
    try {
      // the final thing to do is start my server
      if( !m_server->start() )
        throw SYSTEM_ERROR(reactor_error_code(reactor_error::unexpected_exception),
                           "Unable to start Wt REST server");
      syslog(utils::log::info)<<"Wt REST server started on port "<<m_server->impl().httpPort();
    } catch(Wt::WServer::Exception const &e) {
      syslog(utils::log::error)<<"Server initialization error: "<<e.what();
      throw e;
    }
  }
}

void REST_reactor::handle_tick_start() {
}

void REST_reactor::notify(token const &obs) {
  m_timelines->new_obs(obs, current_tick());
}

bool REST_reactor::synchronize() {
  m_tick->new_tick(current_tick());
  return true;
}

void REST_reactor::new_plan_token(token_id const &t) {
  
}

void REST_reactor::cancelled_plan_token(token_id const &t) {
  
}
