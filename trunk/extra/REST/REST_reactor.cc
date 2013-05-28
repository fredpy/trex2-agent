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

#include <trex/utils/TREXversion.hh>

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

REST_reactor::REST_reactor(TeleoReactor::xml_arg_type arg)
:TeleoReactor(arg, false) {
  // Initialize web server
  bool found;
  
  boost::filesystem::path wt_cfg = manager().use("wt_config.xml", found),
    log_dest = file_name("Wt.log");
  if( found ) {
    bp::ptree cfg;
    
    xml::read_xml(wt_cfg.string(), cfg);
    if( cfg.empty() )
      throw ReactorException(*this, "Wt XML config "+wt_cfg.string()+" is empty");
    // Add log redirection to the file and then save it
    syslog(utils::log::info)<<"redirecting Wt server logs to "<<log_dest.string();
    cfg.put("server.application-settings.log-file", log_dest.string());
    wt_cfg = manager().file_name("cfg/wt_trex.xml");
    xml::write_xml(wt_cfg.string(), cfg);
  
    m_server.reset(new Wt::WServer("", wt_cfg.string()));
  } else {
    syslog(utils::log::warn)<<"Did not find "<<wt_cfg.string()<<": using default Wt server config instead.";
    m_server.reset(new Wt::WServer(""));
  }
  
  // Now look for the program options
  wt_cfg = manager().use("rest.xml", found);
  
  if( !found )
    throw ReactorException(*this, "Failed to locate "+wt_cfg.string()+" required for Wt server arguments");
  // Extarct argument list to be given to the server
  bp::ptree doc;
  xml::read_xml(wt_cfg.string(), doc);
  if( doc.empty() )
    throw ReactorException(*this, "Server arguments XML file "+wt_cfg.string()+" is empty");
  
  
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
    argv[0] = strdup(getName().str().c_str());
    for(size_t i=0; i<argc; ++i)
      argv[i+1] = strdup(args[i].c_str());
  }
  // No I can set my configuration as Wt expect it
  m_server->setServerConfiguration(argc, argv);
  
  // clean up this blooy malloc mess
  for(size_t i=0; i<=argc; ++i)
    free(argv[i]);
  delete[] argv;
  
  // My server is setup and ready I will start it on handleInit
}

REST_reactor::~REST_reactor() {
  syslog(utils::log::info)<<"Closing Wt REST server";
  try {
    m_server.reset();
    m_timelines.reset();
  } catch(Wt::WServer::Exception const &e) {
    syslog(utils::log::error)<<"Eror during REST server shutdown: "<<e.what();
  }
}

// TREX callbacks

void REST_reactor::handleInit() {
  // First create my timelien observer
  m_timelines.reset(new TimelineHistory(*this));
  m_tick.reset(new tick_manager(get_graph(), manager().service()));
  
  // Build service tree
  m_services.reset(new service_tree(*m_server, "/rest"));
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
  
 
  try {
    // the final thing to do is start my server
    if( !m_server->start() )
      throw ReactorException(*this, "Unable to start Wt REST server");
    syslog(utils::log::info)<<"Wt REST server started on port "<<m_server->httpPort();
  } catch(Wt::WServer::Exception const &e) {
    syslog(utils::log::error)<<"Server initialization error: "<<e.what();
    throw ReactorException(*this, e.what());
  }
}

void REST_reactor::handleTickStart() {
}

void REST_reactor::notify(Observation const &obs) {
  m_timelines->new_obs(obs, getCurrentTick());
}

bool REST_reactor::synchronize() {
  m_tick->new_tick(getCurrentTick());
  return true;
}

void REST_reactor::newPlanToken(goal_id const &t) {
  
}

void REST_reactor::cancelledPlanToken(goal_id const &t) {
  
}

//REST_reactor::REST_reactor(TeleoReactor::xml_arg_type arg)
//:TeleoReactor(arg, false), graph::timelines_listener(arg) {
//  bool found;
//  
//  m_strand.reset(new boost::asio::strand(manager().service()));
//  set_verbose(true);
//  
//  boost::filesystem::path wt_cfg = manager().use("wt_config.xml", found),
//  log_dest = file_name("Wt.log");
//  
//  if( found ) {
//    boost::property_tree::ptree cfg;
//    xml::read_xml(wt_cfg.string(), cfg);
//    
//    if( cfg.empty() )
//      throw ReactorException(*this, "XML document "+wt_cfg.string()+" is empty");
//    cfg.put("server.application-settings.log-file", log_dest.string());
//    wt_cfg = manager().file_name("cfg/wt_trex.xml");
//    xml::write_xml(wt_cfg.string(), cfg);
//    m_server.reset(new Wt::WServer("", wt_cfg.string()));
//  } else
//    m_server.reset(new Wt::WServer(""));
//  wt_cfg = manager().use("rest.xml", found);
//  
//  if( !found )
//    throw ReactorException(*this, "unable to locate rest.xml config file");
//
//  try {
//    boost::property_tree::ptree doc;
//    
//    xml::read_xml(wt_cfg.string(), doc, xml::no_comments|xml::trim_whitespace);
//    if( doc.empty() )
//      throw ReactorException(*this, "XML document "+wt_cfg.string()+" is empty");
//    std::string arg = doc.get<std::string>("config.server");
//    
//    std::vector<std::string> args;
//    boost::algorithm::split(args, arg, boost::algorithm::is_space());
//    args.push_back("--accesslog="+file_name("wt_access.log").string());
//    
//    size_t argc = args.size();
//    char **argv = new char*[argc+1];
//    
//    argv[0] = strdup(getName().str().c_str());
//    for(size_t i=0; i<argc; ++i) {
//      argv[i+1] = strdup(args[i].c_str());
//      syslog()<<"argv["<<(i+1)<<"] = "<<argv[i+1];
//    }
//    m_server->setServerConfiguration(argc, argv);
//    for(size_t i=0; i<=argc; ++i)
//      free(argv[i]);
//    delete [] argv;
//    
//    // Create the timline database
//    std::string fname = this->file_name("timelines_history.db3").string();
//    m_obs_db.reset(new dbo::backend::Sqlite3(fname));
//    syslog(utils::log::info)<<"Observations will be logged in database: "<<fname;
//    //m_obs_db->setProperty("show-queries", "true");
//    
//    m_obs_session.setConnection(*m_obs_db);
//    m_obs_session.mapClass<db_timeline>("timeline");
//    m_obs_session.mapClass<db_token>("token");
//    m_obs_session.createTables();
//
//    // Now that we have the database for timelines we can start to listen to timeline events
//    graph::timelines_listener::initialize();
//    
//    /*
//     * Populate the rest/ URI with services 
//     */
//    m_services.reset(new service_tree(*m_server, "/rest"));
//    // TREX general information
//    m_services->add_handler("version",
//                            new json_direct(boost::bind(&REST_reactor::trex_version, this, _1),
//                                            "Give current trex version"));
//    
//    // Tick related info
//    m_services->add_handler("tick",
//                            new json_direct(boost::bind(&REST_reactor::get_tick, this, _1),
//                                            "Give tick information.\n"
//                                            "If no argument, gives the current tick.\n"
//                                            "Example: /rest/tick/1"));
//    m_services->add_handler("tick/next",
//                            new json_direct(boost::bind(&REST_reactor::next_tick, this, _1),
//                                            "Give next tick information"));
//    m_services->add_handler("tick/initial",
//                            new json_direct(boost::bind(&REST_reactor::initial_tick, this, _1),
//                                            "Give initial tick information (ie the tick when trex got started)"));
//    m_services->add_handler("tick/final",
//                            new json_direct(boost::bind(&REST_reactor::final_tick, this, _1),
//                                            "Give final tick information (ie the tick when trex will exit)"));
//    m_services->add_handler("tick/at",
//                            new json_direct(boost::bind(&REST_reactor::tick_at, this, _1),
//                                            "Give the largest tick before the given date.\n"
//                                            "Example: /tick/at/2013-May-03%2021:17:21"));
//    
//    m_services->add_handler("tick/rate",
//                            new json_direct(boost::bind(&REST_reactor::tick_period, this, _1),
//                                            "Give the duration between two ticks"));
//    m_services->add_handler("tick/wait", new bits::tick_wait(*this, "Wait for next tick."));
//    
//    // Timeline related stuff
//    m_services->add_handler("timelines",
//                            new json_direct(boost::bind(&REST_reactor::timelines, this, _1),
//                                            "List all the timelines"));
//    m_services->add_handler("timeline",
//                            new json_direct(boost::bind(&REST_reactor::timeline, this, _1),
//                                            "Access information ot a specific timeline.\n"
//                                            "Example: /rest/timeline/foo"));
//    
//    // Goal management
//    m_services->add_handler("goals",
//                            new json_direct(boost::bind(&REST_reactor::goals, this, _1),
//                                            "List all the goals currently received"));
//    m_services->add_handler("goal",
//                            new json_direct(boost::bind(&REST_reactor::manage_goal, this, _1),
//                                            "POST: post the attached goal to trex.\n"
//                                            "DELETE: request the cancelation of the given goal.\n"
//                                            "GET: get a description of an existing goal."));
//    
// 
//    if( !m_server->start() )
//      throw ReactorException(*this, "Unable to start the server");
//
//  } catch(Wt::WServer::Exception const &e) {
//    syslog(utils::log::error)<<"Server initialization error: "<<e.what();
//    throw ReactorException(*this, e.what());
//  }
//}
//
//REST_reactor::~REST_reactor() {
//  try {
//    m_server.reset();
//  } catch(Wt::WServer::Exception const &e) {
//    syslog(utils::log::error)<<"Error during server shutdown : "<<e.what();
//  }
//}
//
//void REST_reactor::handleInit() {
//}
//
//void REST_reactor::handleTickStart() {
//  m_tick_signal(getCurrentTick());
//}
//
//void REST_reactor::notify(Observation const &obs) {
//  m_strand->post(boost::bind(&REST_reactor::add_obs, this, obs, getCurrentTick()));
//}
//
//
//bool REST_reactor::synchronize() {
//  m_strand->post(boost::bind(&REST_reactor::extend_obs, this, getCurrentTick()));
//  return true;
//}
//
//void REST_reactor::newPlanToken(goal_id const &t) {
//  
//}
//
//void REST_reactor::cancelledPlanToken(goal_id const &t) {
//  
//}
//
//size_t REST_reactor::get_id() {
//  utils::SharedVar<size_t>::scoped_lock lock(m_file_count);
//  *m_file_count += 1;
//  return *m_file_count;
//}
//
//
//// timelines events
//void REST_reactor::declared(details::timeline const &tl) {
//  bits::timeline_info t(tl, *this);
//  m_strand->post(boost::bind(&REST_reactor::add_tl, this, t));
//}
//
//void REST_reactor::undeclared(details::timeline const &tl) {
//  m_strand->post(boost::bind(&REST_reactor::remove_tl, this, tl.name()));
//}
//
//bp::ptree REST_reactor::timelines(rest_request const &req) {
//  return strand_run<bp::ptree>(boost::bind(&list_timelines<tl_set>,
//                                           boost::ref(m_timelines),
//                                           m_services->base()));
//}
//
//#include <boost/lexical_cast.hpp>
//
//bp::ptree REST_reactor::timeline(rest_request const &req) {
//  if( req.arg_path().empty() )
//    throw ReactorException(*this, "Missing timeline argument to "+req.request().path());
//  IntegerDomain::bound lo=IntegerDomain::minus_inf, hi=IntegerDomain::plus_inf;
//  bool as_date = true;
//  
//  std::string const *param;
//  
//  param = req.request().getParameter("format");
//  if( param ) {
//    if( "tick"==*param )
//      as_date = false;
//    else if( "date"!=*param )
//      throw ReactorException(*this, "Invalid format: allowed values are date and tick");
//  }
//  
//  param = req.request().getParameter("from");
//  if( as_date ) {
//    if( NULL!=param ) {
//      try {
//        date_type date = utils::string_cast<date_type>(Wt::Utils::urlDecode(*param));
//        lo = timeToTick(date);
//      } catch(utils::bad_string_cast const &e) {
//        throw ReactorException(*this, "Failed to parse from="+Wt::Utils::urlDecode(*param)+" as a date");
//      }
//    }
//    param = req.request().getParameter("to");
//    if( NULL!=param ) {
//      try {
//        date_type date = utils::string_cast<date_type>(Wt::Utils::urlDecode(*param));
//        hi = timeToTick(date)+1;
//      } catch(utils::bad_string_cast const &e) {
//        throw ReactorException(*this, "Failed to parse to="+Wt::Utils::urlDecode(*param)+" as a date");
//      }
//    }
//  } else {
//    if( NULL!=param ) {
//      try {
//        lo = boost::lexical_cast<TICK>(Wt::Utils::urlDecode(*param));
//      } catch(boost::bad_lexical_cast const &e) {
//        throw ReactorException(*this, "Failed to parse from="+Wt::Utils::urlDecode(*param)+" as a tick");
//      }
//    }
//    param = req.request().getParameter("to");
//    if( NULL!=param ) {
//      try {
//        hi = boost::lexical_cast<TICK>(Wt::Utils::urlDecode(*param));
//      } catch(boost::bad_lexical_cast const &e) {
//        throw ReactorException(*this, "Failed to parse to="+Wt::Utils::urlDecode(*param)+" as a tick");
//      }
//    }
//  }
//  
//  return strand_run<bp::ptree>(boost::bind(&REST_reactor::get_timeline,
//                                           this, req.arg_path().dump(),
//                                           IntegerDomain(lo, hi)));
//}
//
//bp::ptree REST_reactor::export_goal(goal_id g) const {
//  bp::ptree ret;
//  rest_request::path_type uri = m_services->base();
//  uri /= "goal";
//  ret.put("id", g);
//  uri /= ret.get<std::string>("id");
//  
//   ret.put("href", uri.dump());
//  ret.push_back(getGraph().export_goal(g).front());
//  return ret;
//}
//
//void REST_reactor::add_goal(transaction::goal_id g) {
//  std::ostringstream id;
//  postGoal(g);
//  id<<g;
//  m_goals[id.str()] = g;
//}
//
//goal_id REST_reactor::get_goal(std::string const &id) const {
//  goal_map::const_iterator pos = m_goals.find(id);
//  
//  if( m_goals.end()==pos )
//    return goal_id();
//  else
//    return pos->second;
//}
//
//bool REST_reactor::remove_goal(std::string const &id) {
//  goal_map::iterator pos = m_goals.find(id);
//
//  if( m_goals.end()==pos )
//    return false;
//  else {
//    goal_id g = pos->second;
//    m_goals.erase(pos);
//    return postRecall(g);
//  }
//}
//
//
//
//bp::ptree REST_reactor::list_goals(rest_request const &req) const {
//  bp::ptree ret;
//  for(goal_map::const_iterator i=m_goals.begin();
//      m_goals.end()!=i; ++i) {
//    ret.push_back(bp::ptree::value_type("", export_goal(i->second)));
//  }
//  return ret;
//}
//
//bp::ptree REST_reactor::goals(rest_request const &req) {
//  bp::ptree ret,
//    tmp = strand_run<bp::ptree>(boost::bind(&REST_reactor::list_goals,
//                                            this, boost::ref(req)));
//  // Do not add elements that are empty
//  if( !tmp.empty() )
//    ret.add_child("goals", tmp);
//  return ret;
//}
//
//
//bp::ptree REST_reactor::manage_goal(rest_request const &req) {
//  Wt::Http::Request const &rq = req.request();
//  std::string kind = rq.method();
//  bp::ptree ret;
//  
//  if( kind=="POST" ) {
//    if( "application/json"!=rq.contentType() )
//      throw ReactorException(*this, "Data content type should be application/json instead of "+rq.contentType());
//    if( rq.contentLength()<=0 )
//      throw ReactorException(*this, "POST content is empty.");
//    std::ostringstream oss;
//    oss<<"upload."<<get_id()<<".dat";
//    path_type f_name = file_name(oss.str());
//    
//    syslog(utils::log::info)<<"Caching POST goal data to "<<f_name.string();
//    {
//      std::ofstream tmp(f_name.c_str());
//      tmp<<rq.in().rdbuf();
//    }
//    bp::ptree data;
//    try {
//      std::ifstream in(f_name.c_str());
//      utils::read_json(in, data);
//    } catch(std::exception const &e) {
//      syslog(utils::log::warn)<<"Failed to parse "<<f_name<<" as json:\n\t"
//      <<e.what();
//      throw ReactorException(*this, std::string("Failed to parse data as json: ")+e.what());
//    } catch(...) {
//      syslog(utils::log::warn)<<"Failed to parse "<<f_name<<" as json: unknown exception";
//      throw ReactorException(*this, "Failed to parse data as json");
//    }
//    
//    if( data.empty() )
//      throw ReactorException(*this, "empty json data");
//    
//    bp::ptree::value_type g_desc("goal", data);
//    
//    // For now I only parse the first element
//    goal_id g(parse_goal(g_desc));
//    
//    if( !isExternal(g->object()) )
//      throw ReactorException(*this, "Timeline "+g->object().str()+" does not exist");
//    
//    strand_run<void>(boost::bind(&REST_reactor::add_goal, this, g));
//    
//    return export_goal(g);
//  } else {
//    if( req.arg_path().empty() )
//      throw ReactorException(*this, "Missing goal id on "+kind+" "+req.request().path()+" request");
//    std::string id = req.arg_path().dump();
//    
//    if( "DELETE"==kind ) {
//      bool recalled = strand_run<bool>(boost::bind(&REST_reactor::remove_goal, this, id));
//      ret.put("id", id);
//      ret.put("deleted", recalled);
//      return ret;
//    } else if( "GET"==kind ) {
//      goal_id g = strand_run<goal_id>(boost::bind(&REST_reactor::get_goal, this, id));
//      if( !g )
//        throw ReactorException(*this, "No goal associated to id \""+id+"\"");
//      return export_goal(g);
//    } else
//      throw ReactorException(*this, "Service "+req.request().path()+" do not implement "+kind);
//  }
//}
//
//
//bp::ptree REST_reactor::trex_version(rest_request const &req) const {
//  bp::ptree ret;
//  ret.put("version_major", TREX::version::major());
//  ret.put("version_minor", TREX::version::minor());
//  ret.put("version_release", TREX::version::release());
//  if( TREX::version::is_release_candidate() ) {
//    ret.put("version_rc", TREX::version::rc_number());
//  }
//  if( TREX::version::svn_info() ) {
//    ret.put("svn_root", TREX::version::svn_root());
//    ret.put("svn_rev", TREX::version::svn_revision());
//  }
//  ret.put("version_str", TREX::version::full_str());
//  
//  return ret;
//}
//
//
//bp::ptree REST_reactor::get_timeline(std::string name,
//                                     IntegerDomain range) {
//  bp::ptree ret;
//
//  tl_set::iterator i=m_timelines.begin();
// 
//  for(; m_timelines.end()!=i && *i<name; ++i);
//    
//  if( i!=m_timelines.end() && i->name()==name ) {    
//    ret = i->basic_tree(false);
//    ret.put_child("requested_tick_range", range.as_tree());
//    ret.put_child("token", i->list_tokens(range));
//  }
//  return ret;
//}
//
//bp::ptree REST_reactor::tick_info(TICK date) const {
//  bp::ptree ret;
//  
//  ret.put("value", date);
//  ret.put("date", this->date_str(date));
//  return ret;
//}
//
//bp::ptree REST_reactor::get_tick(rest_request const &req) const {
//  TICK date;
//  
//  if( req.arg_path().empty() )
//    date = getCurrentTick();
//  else {
//    std::string arg = req.arg_path().dump();
//    try {
//      date = TREX::utils::string_cast<TICK>(arg);
//    } catch(...) {
//      throw ReactorException(*this, "REST argument \""+arg+"\" is not a valid tick.");
//    }
//  }
//  return tick_info(date);
//}
//
//bp::ptree REST_reactor::tick_at(rest_request const &req) const {
//  if( req.arg_path().empty() )
//    throw ReactorException(*this, "Missing date argument to "+req.request().path());
//  std::string date_str = Wt::Utils::urlDecode(req.arg_path().dump());
//  
//  try {
//    date_type date = utils::string_cast<date_type>(date_str);
//    return tick_info(this->timeToTick(date));
//  } catch(...) {
//    throw ReactorException(*this, "Failed to parse date: "+date_str);
//  }
//}
//
//bp::ptree REST_reactor::tick_period(rest_request const &req) const {
//  duration_type rate = this->tickDuration();
//  CHRONO::nanoseconds
//    ns = CHRONO::duration_cast<CHRONO::nanoseconds>(rate);
//  bp::ptree ret;
//  
//  ret.put("nanoseconds", ns.count());
//  ret.put("duration", duration_str(1));
//  return ret;
//}
//
//
//void REST_reactor::add_tl(bits::timeline_info const &tl) {
//  m_timelines.insert(tl);
//  use(tl.name());
//  
//  dbo::ptr<db_timeline> db_entry;
//
//  {
//    dbo::Transaction tr(m_obs_session);
//    db_timeline *tmp = new db_timeline;
//    
//    tmp->name = tl.name().str();
//    db_entry = m_obs_session.add(tmp);
//  
//    tr.commit();
//  }
//  syslog(utils::log::info)<<"Added "<<db_entry->name<<" timeline to database.";
//}
//
//void REST_reactor::add_obs(Observation obs, TICK cur) {
//  tl_set::const_iterator i = m_timelines.begin();
//  
//  for( ; m_timelines.end()!=i && i->name()<obs.object();++i);
//
//  if( m_timelines.end()!=i && i->name()==obs.object() )
//    i->notify(obs, cur);
//  else
//    syslog(utils::log::warn)<<"Observaytion on unknown timeline "<<obs.object();
//}
//
//void REST_reactor::extend_obs(TICK cur) {
//  IntegerDomain fut(cur+1, IntegerDomain::plus_inf);
//  
//  for(tl_set::const_iterator i=m_timelines.begin(); m_timelines.end()!=i; ++i)
//    i->future(fut);
//}
//
//
//
//
//void REST_reactor::remove_tl(utils::Symbol const &tl) {
//  unuse(tl);
//}
//
///*
// * class TREX::REST::REST_reactor::timeline_info
// */
//
//TREX::utils::Symbol const &bits::timeline_info::name() const {
//  return m_timeline.name();
//}
//
//bool bits::timeline_info::operator< (TREX::utils::Symbol const &n) const {
//  return name()<n;
//}
//
//bool bits::timeline_info::operator< (bits::timeline_info const &other) const {
//  return operator< (other.name());
//}
//
//bool bits::timeline_info::alive() const {
//  return m_timeline.owned();
//}
//
//bool bits::timeline_info::accept_goals() const {
//  return m_timeline.look_ahead()>0;
//}
//
//bp::ptree bits::timeline_info::duration_tree(TICK d) const {
//  bp::ptree ret;
//  ret.put("ticks", d);
//  ret.put("duration", m_reactor->duration_str(d));
//  return ret;
//}
//
//bp::ptree bits::timeline_info::basic_tree(bool complete) const {
//  bp::ptree ret, tick_info;
//  ret.put("name", name());
//  ret.put("alive", alive());
//  if( complete ) {
//    ret.put("accept_goals", accept_goals());
//    // Extra info
//    ret.put_child("latency", duration_tree(m_timeline.latency()));
//    ret.put_child("look_ahead", duration_tree(m_timeline.look_ahead()));
//    ret.put("publish_plan", m_timeline.publish_plan());
//  }
//  return ret;
//}
//
//bp::ptree bits::timeline_info::token_tree(goal_id g) const {
//  graph const &gr = m_reactor->getGraph();
//  return gr.export_goal(g).get_child("Goal");
//}
//
//
//void  bits::timeline_info::notify(Observation const &obs, TICK cur) const {
//  
//  if( m_last ) {
//    std::ostringstream oss;
//  
//    m_last->restrictEnd(IntegerDomain(cur));
//    helpers::json_stream json(oss);
//    utils::write_json(json, token_tree(m_last));
//        
//    {
//      dbo::Transaction tr(m_reactor->m_obs_session);
//      dbo::ptr<db_timeline> tl = m_reactor->m_obs_session.find<db_timeline>().where("name = ?").bind(name().str());
//      if( !tl ) {
//        m_reactor->syslog(utils::log::error)<<"Received an observation for timeline "
//        <<name().str()<<" which is not on the database.";
//        return;
//      }
//      db_token *prev_t  = new db_token;
//      prev_t->start = m_last_tick;
//      prev_t->end = cur;
//      prev_t->timeline = tl;
//      prev_t->value = oss.str();
//      dbo::ptr<db_token> tok = m_reactor->m_obs_session.add(prev_t);
//      tl.modify()->last_obs = tok;
//      tl.flush();
//      tr.commit();
//      // m_reactor->syslog(utils::log::info)<<"Added "<<tok->value<<" on ["<<m_last_tick<<", "<<cur<<"].";
//    }
//  }
//  m_last_tick = cur;
//  
//  m_last.reset(new Goal(obs, cur));
//}
//
//void bits::timeline_info::future(IntegerDomain const &f) const {
//  if( m_last )
//    m_last->restrictEnd(f);
//}
//
//
//
//bp::ptree bits::timeline_info::list_tokens(IntegerDomain const &range) const {
//  typedef dbo::collection< dbo::ptr<db_token> > token_set;
//  IntegerDomain::bound lo, hi;
//  
//  range.getBounds(lo, hi);
//  
//  bool no_db = true, get_current = false;
//  dbo::Query< dbo::ptr<db_token> >
//    query = m_reactor->m_obs_session.find<db_token>().where("timeline_name = ?").bind(name().str());
//  
//  m_reactor->syslog(utils::log::info)<<"Building the set of tokens over "<<range;
//  
//  if( m_last ) {
//    if( range.hasLower() ) {
//      if( m_last_tick>=lo.value() ) {
//        no_db = false;
//        query.where("end >= ?").bind(lo.value());
//      }
//    } else
//      no_db = false;
//    get_current = (hi>=m_last_tick);
//  }
//  // do it with an innefficient reparse for now
//  bp::ptree result;
//  
//  if( !no_db ) {
//    token_set past;
//    dbo::Transaction tr(m_reactor->m_obs_session);
//    if( range.hasUpper() && m_last_tick>=hi.value() )
//      query.where("start <= ?").bind(hi.value());
//    
//    past = query;
//    
//    m_reactor->syslog(utils::log::info)<<past.size()<<" past observations in "<<range;
//    for(token_set::const_iterator i=past.begin(); i!=past.end(); ++i) {
//      if( lo>(*i)->start )
//        lo = (*i)->start;
//      if( hi< (*i)->end )
//        hi = (*i)->end;
//      
//      std::istringstream iss((*i)->value);
//      bp::ptree tok;
//      utils::read_json(iss, tok);
//      result.push_back(bp::ptree::value_type("", tok));
//    }
//    tr.commit();
//  }
//  
//  if( get_current ) {
//    if( lo>m_last_tick )
//      lo = m_last_tick;
//    if( hi<m_last->getEnd().lowerBound() )
//      hi = m_last->getEnd().lowerBound();
//    
//    result.push_back(bp::ptree::value_type("", token_tree(m_last)));
//  }  
//  return result;
//}
//
//
//
//
//
//
//
//
//
