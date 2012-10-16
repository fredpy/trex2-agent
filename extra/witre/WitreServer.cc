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
#include "WitreServer.hh"

using namespace TREX::witre;
using namespace TREX::utils;

/*
 * class TREX::witre::WitreServer
 */

// structors

WitreServer::WitreServer(Wt::WServer &server)
:m_server(server), m_log_proxy(NULL), m_log_db(":memory:") {
  init_session();
  m_log->add_handler(LogProxy(*this)); // produce signals on new log entry
  m_log->logPath(); // initialize TREX log directory
  log(info)<<"Witre server created.";

  // Get the cfg directory
  m_locales = m_log->file_name("cfg");
}

WitreServer::~WitreServer() {
  log(warn)<<"Witre server destroyed.";
}

// modifiers

void WitreServer::init_session() {
  m_log_db.setProperty("show-queries", "true");  
  
  m_log_session.setConnection(m_log_db);
  m_log_session.mapClass<dbo::Msg>("log_msg");
  m_log_session.mapClass<dbo::MsgType>("log_type");
  m_log_session.createTables();
  
  
}

void WitreServer::reset_log(LogProxy *ref) {
  m_log_proxy = ref; 
}

// manipulators

LogProxy::log_signal &WitreServer::new_log() {
  return m_log_proxy->new_entry();
}

boost::filesystem::path WitreServer::locales(std::string const &default_file) {
  bool found;
  boost::filesystem::path file = m_log->locate(default_file, found);
  
  if( !found ) {
    log(error)<<"Unable to locate \""<<default_file<<"\"";
    // TODO throw an exception 
    return boost::filesystem::path();
  } else {
    boost::filesystem::path log_dir = locale_path(default_file);
    if( !exists(log_dir/file.filename()) ) {
      log(info)<<"Copying locales based on "<<default_file
      <<" to "<<log_dir;
      copy_file(file, log_dir/file.filename());
      std::string loc_base = file.stem().string()+"_";
      
      for(boost::filesystem::directory_iterator it(file.parent_path()), endit;
          endit!=it; ++it) {
        boost::filesystem::path tmp = it->path();
        if( 0==tmp.stem().string().compare(0, loc_base.length(), loc_base) &&
           tmp.extension()==file.extension() ) {
          log(info)<<"\tAdding locale ["
                    <<tmp.stem().string().substr(loc_base.length())<<"]";
          copy_file(tmp, log_dir/tmp.filename());
        }
      }
    }
    return log_dir/file.stem();
  }
}

boost::filesystem::path WitreServer::locale_path(std::string const &file) {
  boost::filesystem::path p(file);
  
  if( p.has_parent_path() ) {
    p = m_locales/p.parent_path();
    p.make_preferred();
    create_directories(p);
    return p;
  } else
    return m_locales;
}



// observers

size_t WitreServer::log_types(std::set<std::string> &types) {
  typedef Wt::Dbo::collection< Wt::Dbo::ptr<dbo::MsgType> > Types;
  Types msg_types;
  size_t ret = 0;
  {
    Wt::Dbo::Transaction tr(m_log_session);
    msg_types = m_log_session.find<dbo::MsgType>();
  
    
    for(Types::const_iterator i=msg_types.begin(); 
        i!=msg_types.end(); ++i) {
      if( types.insert((*i)->identifier).second )
        ++ret;
    }
    tr.commit();
  }
  
  return ret;
}

WitreServer::msg_set WitreServer::last_messages(size_t count) {
  msg_set ret;
  {
    Wt::Dbo::Transaction tr(m_log_session);
    Wt::Dbo::collection< Wt::Dbo::ptr<dbo::Msg> > 
      req = m_log_session.find<dbo::Msg>().orderBy("id DESC").limit(count);
    for(Wt::Dbo::collection< Wt::Dbo::ptr<dbo::Msg> >::const_iterator i=req.begin();
        i!=req.end();++i) {
      ret.push_back(msg((*i)->type->identifier, (*i)->date, (*i)->source, (*i)->content));
    }
    tr.commit();
  }
  return ret;
}

