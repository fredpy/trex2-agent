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
  bp::ptree list_timelines(Set const &l, Wt::Http::Request const &req) {
    bp::ptree ret, tls;
    
    for(typename Set::const_iterator i=l.begin(); l.end()!=i; ++i) {
      bp::ptree tl;
      tl.put("name", *i);
      tl.put("href", "/timeline/"+i->str());
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
  wt_cfg = manager().use("witre.xml", found);
  
  if( !found )
    throw ReactorException(*this, "unable to locate witre.xml config file");

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
    m_server->addResource(new REST_service(boost::bind(&REST_reactor::timelines,
                                                       this, _1)),
                          "/timelines");
    m_server->addResource(new REST_service(boost::bind(&REST_reactor::tick_info,
                                                       this, _1)),
                          "/tick");
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

// timelines events
void REST_reactor::declared(details::timeline const &tl) {
  syslog()<<"New timeline "<<tl.name();
  m_strand->post(boost::bind(&REST_reactor::add_tl, this, tl.name()));
}

void REST_reactor::undeclared(details::timeline const &tl) {
  m_strand->post(boost::bind(&REST_reactor::remove_tl, this, tl.name()));
}

bp::ptree REST_reactor::timelines(Wt::Http::Request const &req) {
  return strand_run<bp::ptree>(boost::bind(&list_timelines<tl_set>,
                                           boost::ref(m_timelines),
                                           req));
}

bp::ptree REST_reactor::tick_info(Wt::Http::Request const &req) {
  bp::ptree ret;
  std::string sub_path(req.pathInfo());
  
  std::list<std::string> rest_params;
  alg::split(rest_params, sub_path, alg::is_any_of("/"),
             alg::token_compress_on);
  std::list<std::string>::iterator i=rest_params.begin();
  
  // Remove all the empty ones
  while( i!=rest_params.end() ) {
    if( i->empty() )
      i = rest_params.erase(i);
    else
      ++i;
  }
  
  TICK cur;
  
  if( rest_params.empty() )
    cur = getCurrentTick();
  else if( rest_params.front()=="rate" ) {
    duration_type rate = this->tickDuration();
    // Commented out as the format id not really standard right now
    //std::ostringstream oss;
    //TREX::utils::display(oss, rate);
    //ret.put("duration", oss.str());
    
    CHRONO::nanoseconds
      ns = CHRONO::duration_cast<CHRONO::nanoseconds>(rate);
    ret.put("nanoseconds", ns.count());
    ret.put("duration", duration_str(1));
    
    return ret;
  } else if( rest_params.front()=="initial" )
    cur = getInitialTick();
  else if( rest_params.front()=="next" )
    cur = getCurrentTick()+1;
  else if( rest_params.front()=="final" )
    cur = getFinalTick();
  else if( rest_params.front()=="at" ) {
    if( rest_params.size()!=2 )
      throw ReactorException(*this, "tick/at require a UTC date.\n"
                             "Example tick/at/2013-Apr-23%2021:03:00");
    // TODO need to parse the date
    std::string date_str = Wt::Utils::urlDecode(rest_params.back());
    try {
      syslog(info)<<"Parsing \""<<date_str<<"\" as a date";
      date_type date = utils::string_cast<date_type>(date_str);
      syslog(info)<<"Result of parsing is "<<date;
      
      cur = this->timeToTick(date);
      syslog(info)<<"conversion in tick is "<<cur
      <<"\n\twhich is "<<this->date_str(cur);
    } catch(...) {
      throw ReactorException(*this, "Invalid date format: "+date_str);
    }
  } else {
    try {
      cur = TREX::utils::string_cast<TICK>(rest_params.front());
    } catch(...) {
      throw ReactorException(*this, "Invalid tick request "+req.path()+req.pathInfo());
    }
  }
  ret.put("value", cur);
  ret.put("date", this->date_str(cur));
  return ret;
}


void REST_reactor::add_tl(utils::Symbol const &tl) {
  m_timelines.insert(tl);
}

void REST_reactor::remove_tl(utils::Symbol const &tl) {
  m_timelines.erase(tl);
}





