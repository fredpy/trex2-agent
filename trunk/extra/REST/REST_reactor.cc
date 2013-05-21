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

#include <trex/utils/TREXversion.hh>
#include <trex/utils/XmlUtils.hh>
#include <trex/utils/ptree_io.hh>
#include <trex/utils/chrono_helper.hh>

#include <boost/date_time/posix_time/posix_time_io.hpp>
#include <boost/algorithm/string.hpp>

#include <boost/enable_shared_from_this.hpp>

#include <Wt/Utils>


using namespace TREX::REST;
using namespace TREX::transaction;

namespace xml=boost::property_tree::xml_parser;
namespace bp=boost::property_tree;
namespace alg=boost::algorithm;


namespace {
  
  template<class Set>
  bp::ptree list_timelines(Set const &l, rest_request::path_type const &pfx) {
    bp::ptree ret, tls;
    rest_request::path_type uri(pfx);
    uri /= "timeline";
    
    for(typename Set::const_iterator i=l.begin(); l.end()!=i; ++i) {
      bp::ptree tl = i->basic_tree();
      tl.put("href", (uri / rest_request::path_type(i->name().str())).dump());
      tls.push_back(bp::ptree::value_type("", tl));
    }
    if( !tls.empty() )
      ret.add_child("timelines", tls);
    return ret;
  }
  

}

#include <trex/utils/ptree_io.hh>

namespace TREX {
  namespace REST {
    namespace bits {
      
      class tick_wait :public rest_service {
      public:
        tick_wait(REST_reactor &r, std::string const &info)
        :rest_service(info), m_reactor(&r) {
          m_conn = m_reactor->m_tick_signal.connect(boost::bind(&tick_wait::new_tick, this, _1));
        }
        ~tick_wait() {
          m_conn.disconnect();
          beingDeleted();
        }

      private:
        void new_tick(TICK cur) {
          m_current = cur;
          haveMoreData();
        }
        TICK get_tick() {
          TREX::utils::SharedVar<TICK>::scoped_lock lck(m_current);
          return *m_current;
        }
        
        void handleRequest(rest_request const &req,
                           std::ostream &data,
                           Wt::Http::Response &ans) {
          Wt::Http::ResponseContinuation *cont = req.request().continuation();
          if( cont ) {
            TICK date = boost::any_cast<TICK>(cont->data()), cur = get_tick();
           if( date<=cur ) {
              helpers::json_stream json(data);
              ans.setMimeType("application/json");
              TREX::utils::write_json(json, m_reactor->tick_info(cur));
            } else {
              cont = ans.createContinuation();
              cont->setData(date);
              cont->waitForMoreData();
           }
          } else {
            // Initial implementation only wait for next tick
            TICK next = get_tick()+1;
            
            cont = ans.createContinuation();
            cont->setData(next);
            cont->waitForMoreData();
          }
        }
        
        TREX::utils::SharedVar<TICK> m_current;
        
        REST_reactor *m_reactor;
        boost::signals2::connection m_conn;
      }; //TREX::REST::bits::tick_wait

    } // TREX::REST::bits
  } // TREX::REST
} // TREX

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
    m_services.reset(new service_tree(*m_server, "/rest"));
    // TREX general information
    m_services->add_handler("version",
                            new json_direct(boost::bind(&REST_reactor::trex_version, this, _1),
                                            "Give current trex version"));
    
    // Tick related info
    m_services->add_handler("tick",
                            new json_direct(boost::bind(&REST_reactor::get_tick, this, _1),
                                            "Give tick information.\n"
                                            "If no argument, gives the current tick.\n"
                                            "Example: /rest/tick/1"));
    m_services->add_handler("tick/next",
                            new json_direct(boost::bind(&REST_reactor::next_tick, this, _1),
                                            "Give next tick information"));
    m_services->add_handler("tick/initial",
                            new json_direct(boost::bind(&REST_reactor::initial_tick, this, _1),
                                            "Give initial tick information (ie the tick when trex got started)"));
    m_services->add_handler("tick/final",
                            new json_direct(boost::bind(&REST_reactor::final_tick, this, _1),
                                            "Give final tick information (ie the tick when trex will exit)"));
    m_services->add_handler("tick/at",
                            new json_direct(boost::bind(&REST_reactor::tick_at, this, _1),
                                            "Give the largest tick before the given date.\n"
                                            "Example: /tick/at/2013-May-03%2021:17:21"));
    
    m_services->add_handler("tick/rate",
                            new json_direct(boost::bind(&REST_reactor::tick_period, this, _1),
                                            "Give the duration between two ticks"));
    m_services->add_handler("tick/wait", new bits::tick_wait(*this, "Wait for next tick."));
    
    // Timeline related stuff
    m_services->add_handler("timelines",
                            new json_direct(boost::bind(&REST_reactor::timelines, this, _1),
                                            "List all the timelines"));
    m_services->add_handler("timeline",
                            new json_direct(boost::bind(&REST_reactor::timeline, this, _1),
                                            "Access information ot a specific timeline.\n"
                                            "Example: /rest/timeline/foo"));
    
    // Goal management
    m_services->add_handler("goals",
                            new json_direct(boost::bind(&REST_reactor::goals, this, _1),
                                            "List all the goals currently received"));
    m_services->add_handler("goal",
                            new json_direct(boost::bind(&REST_reactor::manage_goal, this, _1),
                                            "POST: post the attached goal to trex.\n"
                                            "DELETE: request the concelation of the given goal.\n"
                                            "GET: get a description of an existing goal."));
 
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
  m_tick_signal(getCurrentTick());
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
  bits::timeline_info t(tl, *this);
  m_strand->post(boost::bind(&REST_reactor::add_tl, this, t));
}

void REST_reactor::undeclared(details::timeline const &tl) {
  m_strand->post(boost::bind(&REST_reactor::remove_tl, this, tl.name()));
}

bp::ptree REST_reactor::timelines(rest_request const &req) {
  return strand_run<bp::ptree>(boost::bind(&list_timelines<tl_set>,
                                           boost::ref(m_timelines),
                                           m_services->base()));
}

bp::ptree REST_reactor::timeline(rest_request const &req) {
  if( req.arg_path().empty() )
    throw ReactorException(*this, "Missing timeline argument to "+req.request().path());
  
  return strand_run<bp::ptree>(boost::bind(&REST_reactor::get_timeline,
                                           this, req.arg_path().dump()));
}

bp::ptree REST_reactor::export_goal(goal_id g) const {
  bp::ptree ret;
  rest_request::path_type uri = m_services->base();
  uri /= "goal";
  ret.put("id", g);
  uri /= ret.get<std::string>("id");
  
   ret.put("href", uri.dump());
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



bp::ptree REST_reactor::list_goals(rest_request const &req) const {
  bp::ptree ret;
  for(goal_map::const_iterator i=m_goals.begin();
      m_goals.end()!=i; ++i) {
    ret.push_back(bp::ptree::value_type("", export_goal(i->second)));
  }
  return ret;
}

bp::ptree REST_reactor::goals(rest_request const &req) {
  bp::ptree ret,
    tmp = strand_run<bp::ptree>(boost::bind(&REST_reactor::list_goals,
                                            this, boost::ref(req)));
  // Do not add elements that are empty
  if( !tmp.empty() )
    ret.add_child("goals", tmp);
  return ret;
}


bp::ptree REST_reactor::manage_goal(rest_request const &req) {
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
    
    return export_goal(g);
  } else {
    if( req.arg_path().empty() )
      throw ReactorException(*this, "Missing goal id on "+kind+" "+req.request().path()+" request");
    std::string id = req.arg_path().dump();
    
    if( "DELETE"==kind ) {
      bool recalled = strand_run<bool>(boost::bind(&REST_reactor::remove_goal, this, id));
      ret.put("id", id);
      ret.put("deleted", recalled);
      return ret;
    } else if( "GET"==kind ) {
      goal_id g = strand_run<goal_id>(boost::bind(&REST_reactor::get_goal, this, id));
      if( !g )
        throw ReactorException(*this, "No goal associated to id \""+id+"\"");
      return export_goal(g);
    } else
      throw ReactorException(*this, "Service "+req.request().path()+" do not implement "+kind);
  }
}


bp::ptree REST_reactor::trex_version(rest_request const &req) const {
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


bp::ptree REST_reactor::get_timeline(std::string name) {
  bp::ptree ret;

  tl_set::iterator i=m_timelines.begin();
 
  for(; m_timelines.end()!=i && *i<name; ++i);
    
  if( i!=m_timelines.end() && i->name()==name ) {
    ret = i->basic_tree();
    ret.put("warning", "unimplemented");
  }
  return ret;
}

bp::ptree REST_reactor::tick_info(TICK date) const {
  bp::ptree ret;
  
  ret.put("value", date);
  ret.put("date", this->date_str(date));
  return ret;
}

bp::ptree REST_reactor::get_tick(rest_request const &req) const {
  TICK date;
  
  if( req.arg_path().empty() )
    date = getCurrentTick();
  else {
    std::string arg = req.arg_path().dump();
    try {
      date = TREX::utils::string_cast<TICK>(arg);
    } catch(...) {
      throw ReactorException(*this, "REST argument \""+arg+"\" is not a valid tick.");
    }
  }
  return tick_info(date);
}

bp::ptree REST_reactor::tick_at(rest_request const &req) const {
  if( req.arg_path().empty() )
    throw ReactorException(*this, "Missing date argument to "+req.request().path());
  std::string date_str = Wt::Utils::urlDecode(req.arg_path().dump());
  
  try {
    date_type date = utils::string_cast<date_type>(date_str);
    return tick_info(this->timeToTick(date));
  } catch(...) {
    throw ReactorException(*this, "Failed to parse date: "+date_str);
  }
}

bp::ptree REST_reactor::tick_period(rest_request const &req) const {
  duration_type rate = this->tickDuration();
  CHRONO::nanoseconds
    ns = CHRONO::duration_cast<CHRONO::nanoseconds>(rate);
  bp::ptree ret;
  
  ret.put("nanoseconds", ns.count());
  ret.put("duration", duration_str(1));
  return ret;
}


void REST_reactor::add_tl(bits::timeline_info const &tl) {
  m_timelines.insert(tl);
  use(tl.name());
}

void REST_reactor::remove_tl(utils::Symbol const &tl) {
  unuse(tl);
}

/*
 * class TREX::REST::REST_reactor::timeline_info
 */

TREX::utils::Symbol const &bits::timeline_info::name() const {
  return m_timeline.name();
}

bool bits::timeline_info::operator< (TREX::utils::Symbol const &n) const {
  return name()<n;
}

bool bits::timeline_info::operator< (bits::timeline_info const &other) const {
  return operator< (other.name());
}

bool bits::timeline_info::alive() const {
  return m_timeline.owned();
}

bool bits::timeline_info::accept_goals() const {
  return m_timeline.look_ahead()>0;
}

bp::ptree bits::timeline_info::duration_tree(TICK d) const {
  bp::ptree ret;
  ret.put("ticks", d);
  ret.put("duration", m_reactor->duration_str(d));
  return ret;
}


bp::ptree bits::timeline_info::basic_tree() const {
  bp::ptree ret, tick_info;
  ret.put("name", name());
  ret.put("alive", alive());
  ret.put("accept_goals", accept_goals());
  // Extra info
  ret.put_child("latency", duration_tree(m_timeline.latency()));
  ret.put_child("look_ahead", duration_tree(m_timeline.look_ahead()));
  ret.put("publish_plan", m_timeline.publish_plan());
  return ret;
}






