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
#include "REST_service.hh"

#include <trex/utils/XmlUtils.hh>
#include <trex/utils/ptree_io.hh>
#include <trex/utils/chrono_helper.hh>

#include <boost/date_time/posix_time/posix_time_io.hpp>
#include <boost/algorithm/string.hpp>

#include <Wt/Utils>


using namespace TREX::REST;
using namespace TREX::transaction;

namespace xml=boost::property_tree::xml_parser;
namespace bp=boost::property_tree;
namespace alg=boost::algorithm;


namespace {
  
  template<class Set>
  bp::ptree list_timelines(Set const &l, req_info const &req) {
    bp::ptree ret, tls;
    
    for(typename Set::const_iterator i=l.begin(); l.end()!=i; ++i) {
      bp::ptree tl;
      tl.put("name", *i);
      tl.put("href", req.request().path()+"/timeline/"+i->str());
      tls.push_back(bp::ptree::value_type("", tl));
    }
    ret.add_child("timelines", tls);
    return ret;
  }

}

REST_reactor::REST_reactor(TeleoReactor::xml_arg_type arg)
:TeleoReactor(arg, false), graph::timelines_listener(arg) {
  bool found;
  
  m_strand.reset(new boost::asio::strand(manager().service()));
  set_verbose(true);
  
  boost::filesystem::path wt_cfg = manager().use("wt_config.xml", found),
  log_dest = file_name("Wt.log");
  
  if( found ) {
    boost::property_tree::ptree cfg;
    xml::read_xml(wt_cfg.string(), cfg);
    
    if( cfg.empty() )
      throw ReactorException(*this, "XML document "+wt_cfg.string()+" is empty");
    cfg.put("server.application-settings.log-file", log_dest.string());
    wt_cfg = manager().file_name("cfg/wt_trex.xml");
    xml::write_xml(wt_cfg.string(), cfg);
    m_server.reset(new Wt::WServer("", wt_cfg.string()));
  } else
    m_server.reset(new Wt::WServer(""));
  wt_cfg = manager().use("rest.xml", found);
  
  if( !found )
    throw ReactorException(*this, "unable to locate rest.xml config file");

  try {
    boost::property_tree::ptree doc;
    
    xml::read_xml(wt_cfg.string(), doc, xml::no_comments|xml::trim_whitespace);
    if( doc.empty() )
      throw ReactorException(*this, "XML document "+wt_cfg.string()+" is empty");
    std::string arg = doc.get<std::string>("config.server");
    
    std::vector<std::string> args;
    boost::algorithm::split(args, arg, boost::algorithm::is_space());
    args.push_back("--accesslog="+file_name("wt_access.log").string());
    
    size_t argc = args.size();
    char **argv = new char*[argc+1];
    
    argv[0] = strdup(getName().str().c_str());
    for(size_t i=0; i<argc; ++i) {
      argv[i+1] = strdup(args[i].c_str());
      syslog()<<"argv["<<(i+1)<<"] = "<<argv[i+1];
    }
    m_server->setServerConfiguration(argc, argv);
    for(size_t i=0; i<=argc; ++i)
      free(argv[i]);
    delete [] argv;
    
    graph::timelines_listener::initialize();
    m_server->addResource(&m_services, "/rest");
    m_services.add_handler("tick",
                           boost::bind(&REST_reactor::get_tick, this, _1),
                           "Give tick information.\n"
                           "If no argument, gives the current tick.\n"
                           "Example: /rest/tick/1");
    m_services.add_handler("tick/next",
                           boost::bind(&REST_reactor::next_tick, this, _1),
                           "Give next tick information");
    m_services.add_handler("tick/initial",
                           boost::bind(&REST_reactor::initial_tick, this, _1),
                           "Give initial tick when trex got started.");
    m_services.add_handler("tick/final",
                           boost::bind(&REST_reactor::final_tick, this, _1),
                           "Give final tick when trex will exit.");
    m_services.add_handler("tick/at",
                           boost::bind(&REST_reactor::tick_at, this, _1),
                           "Give the largest tick before the given date.\n"
                           "Example: /tick/at/2013-May-03%2021:17:21");
    m_services.add_handler("tick/rate",
                           boost::bind(&REST_reactor::tick_period, this, _1),
                           "Give the duration between two ticks");
    m_services.add_handler("timelines",
                           boost::bind(&REST_reactor::timelines, this, _1),
                           "List all the timelines");
    m_services.add_handler("timeline",
                           boost::bind(&REST_reactor::timeline, this, _1),
                           "Access information ot a specific timeline.\n"
                           "Example: /rest/timeline/foo");
    m_services.add_handler("goal",
                           boost::bind(&REST_reactor::manage_goal, this, _1),
                           "POST: post the attached goal to trex.\n"
                           "DELETE: request the concelation of the given goal.\n"
                           "GET: get a description of an exisiting goal.\n");
    m_services.add_handler("goals",
                           boost::bind(&REST_reactor::goals, this, _1),
                           "List all the goals");
    
    if( !m_server->start() )
      throw ReactorException(*this, "Unable to start the server");

  } catch(Wt::WServer::Exception const &e) {
    syslog(utils::log::error)<<"Server initialization error: "<<e.what();
    throw ReactorException(*this, e.what());
  }
}

REST_reactor::~REST_reactor() {
  try {
    m_server.reset();
  } catch(Wt::WServer::Exception const &e) {
    syslog(utils::log::error)<<"Error during server shutdown : "<<e.what();
  }
}

void REST_reactor::handleInit() {
}

void REST_reactor::handleTickStart() {
}

void REST_reactor::notify(Observation const &obs) {
  
}

bool REST_reactor::synchronize() {
  return true;
}

void REST_reactor::newPlanToken(goal_id const &t) {
  
}

void REST_reactor::cancelledPlanToken(goal_id const &t) {
  
}

size_t REST_reactor::get_id() {
  utils::SharedVar<size_t>::scoped_lock lock(m_file_count);
  *m_file_count += 1;
  return *m_file_count;
}


// timelines events
void REST_reactor::declared(details::timeline const &tl) {
  syslog()<<"New timeline "<<tl.name();
  m_strand->post(boost::bind(&REST_reactor::add_tl, this, tl.name()));
}

void REST_reactor::undeclared(details::timeline const &tl) {
  m_strand->post(boost::bind(&REST_reactor::remove_tl, this, tl.name()));
}

bp::ptree REST_reactor::timelines(req_info const &req) {
  return strand_run<bp::ptree>(boost::bind(&list_timelines<tl_set>,
                                           boost::ref(m_timelines),
                                           req));
}

bp::ptree REST_reactor::timeline(req_info const &req) {
  if( req.arg_list().empty() )
    throw ReactorException(*this, "Missing timeline argument to "
                           +req.request().path()+req.request().pathInfo());
  return strand_run<bp::ptree>(boost::bind(&REST_reactor::get_timeline,
                                           this, req.arg_list().front()));
}

bp::ptree REST_reactor::export_goal(goal_id g, req_info const &req) const {
  bp::ptree ret;
  ret.put("id", g);
  ret.put("href", req.request().path()+"/goal/"+ret.get<std::string>("id"));
  ret.push_back(getGraph().export_goal(g).front());
  return ret;
}

void REST_reactor::add_goal(transaction::goal_id g) {
  std::ostringstream id;
  postGoal(g);
  id<<g;
  m_goals[id.str()] = g;
}

goal_id REST_reactor::get_goal(std::string const &id) const {
  goal_map::const_iterator pos = m_goals.find(id);
  
  if( m_goals.end()==pos )
    return goal_id();
  else
    return pos->second;
}

bool REST_reactor::remove_goal(std::string const &id) {
  goal_map::iterator pos = m_goals.find(id);

  if( m_goals.end()==pos )
    return false;
  else {
    goal_id g = pos->second;
    m_goals.erase(pos);
    return postRecall(g);
  }
}



bp::ptree REST_reactor::list_goals(req_info const &req) const {
  bp::ptree ret;
  for(goal_map::const_iterator i=m_goals.begin();
      m_goals.end()!=i; ++i) {
    ret.push_back(bp::ptree::value_type("", export_goal(i->second, req)));
  }
  return ret;
}

bp::ptree REST_reactor::goals(req_info const &req) {
  bp::ptree ret,
    tmp = strand_run<bp::ptree>(boost::bind(&REST_reactor::list_goals,
                                            this, boost::ref(req)));
  
  ret.add_child("goals", tmp);
  return ret;
}


bp::ptree REST_reactor::manage_goal(req_info const &req) {
  Wt::Http::Request const &rq = req.request();
  std::string kind = rq.method();
  bp::ptree ret;
  
  if( kind=="POST" ) {
    if( "application/json"!=rq.contentType() )
      throw ReactorException(*this, "Data content type should be application/json instead of "+rq.contentType());
    if( rq.contentLength()<=0 )
      throw ReactorException(*this, "POST content is empty.");
    std::ostringstream oss;
    oss<<"upload."<<get_id()<<".dat";
    path_type f_name = file_name(oss.str());
    
    syslog(utils::log::info)<<"Caching POST goal data to "<<f_name.string();
    {
      std::ofstream tmp(f_name.c_str());
      tmp<<rq.in().rdbuf();
    }
    bp::ptree data;
    try {
      std::ifstream in(f_name.c_str());
      utils::read_json(in, data);
    } catch(std::exception const &e) {
      syslog(utils::log::warn)<<"Failed to parse "<<f_name<<" as json:\n\t"
        <<e.what();
      throw ReactorException(*this, std::string("Failed to parse data as json: ")+e.what());
    } catch(...) {
      syslog(utils::log::warn)<<"Failed to parse "<<f_name<<" as json: unknown exception";
      throw ReactorException(*this, "Failed to parse data as json");
    }
    
    if( data.empty() )
      throw ReactorException(*this, "empty json data");
    
    bp::ptree::value_type g_desc("goal", data);
    
    // For now I only parse the first element
    goal_id g(parse_goal(g_desc));
    
    if( !isExternal(g->object()) )
      throw ReactorException(*this, "Timeline "+g->object().str()+" does not exist");
    
    strand_run<void>(boost::bind(&REST_reactor::add_goal, this, g));
    
    return export_goal(g, req);
  } else if( kind=="DELETE" ) {
    if( req.arg_list().empty() )
      throw ReactorException(*this, "Missing goal id argument to "
                             +req.request().path()+req.request().pathInfo());

    bool recalled = strand_run<bool>(boost::bind(&REST_reactor::remove_goal,
                                                 this, req.arg_list().front()));
    ret.put("id", req.arg_list().front());
    ret.put("deleted", recalled);
                                                 
    return ret;
  } else if( kind=="GET" ) {
    if( req.arg_list().empty() )
      throw ReactorException(*this, "Missing goal id argument to "
                             +req.request().path()+req.request().pathInfo());
    goal_id g = strand_run<goal_id>(boost::bind(&REST_reactor::get_goal, this,
                                                req.arg_list().front()));
    if( !g )
      throw ReactorException(*this,
                             "No goal associated to "+req.arg_list().front());
    
    return export_goal(g, req);
  } else 
    throw ReactorException(*this, "http method \""+kind+"\" is not supported by "
                           +req.request().path()+req.request().pathInfo());

}


bp::ptree REST_reactor::get_timeline(std::string name) {
  bp::ptree ret;
  
  tl_set::iterator i=m_timelines.find(name);
  
  ret.put("warning", "unimplemented");
  if( m_timelines.end()!=i )
    ret.put("name", *i);
  return ret;
}

bp::ptree REST_reactor::tick_info(TICK date) const {
  bp::ptree ret;
  
  ret.put("value", date);
  ret.put("date", this->date_str(date));
  return ret;
}

bp::ptree REST_reactor::get_tick(req_info const &req) const {
  TICK date;
  
  if( req.arg_list().empty() )
    date = getCurrentTick();
  else {
    try {
      date = TREX::utils::string_cast<TICK>(req.arg_list().front());
    } catch(...) {
      throw ReactorException(*this, "Invalid tick request "+req.request().path()+
                             req.request().pathInfo());
    }
  }
  return tick_info(date);
}

bp::ptree REST_reactor::tick_at(req_info const &req) const {
  if( req.arg_list().empty() )
    throw ReactorException(*this, "Missing date argument to "+req.request().path()+req.request().pathInfo());
  std::string date_str = Wt::Utils::urlDecode(req.arg_list().front());
  syslog()<<"Requested to convert date \""<<date_str<<"\" into a tick";
  try {
    date_type date = utils::string_cast<date_type>(date_str);
    syslog()<<"Result date is "<<date<<" with zone "<<date.zone_abbrev();
    return tick_info(this->timeToTick(date));
  } catch(...) {
    throw ReactorException(*this, "Failed to parse date: "+date_str);
  }
}

bp::ptree REST_reactor::tick_period(req_info const &req) const {
  duration_type rate = this->tickDuration();
  CHRONO::nanoseconds
    ns = CHRONO::duration_cast<CHRONO::nanoseconds>(rate);
  bp::ptree ret;
  
  ret.put("nanoseconds", ns.count());
  ret.put("duration", duration_str(1));
  return ret;
}


void REST_reactor::add_tl(utils::Symbol const &tl) {
  m_timelines.insert(tl);
  use(tl);
}

void REST_reactor::remove_tl(utils::Symbol const &tl) {
  unuse(tl);
  m_timelines.erase(tl);
}





