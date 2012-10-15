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

/*
 * class TREX::witre::LogProxy
 */

// structors 

LogProxy::LogProxy(WitreServer &creator):me(&creator), m_log(this) {
  me->reset_log(this);
}

LogProxy::LogProxy(LogProxy const &other):me(NULL), m_log(this) {
  std::swap(me, other.me);
  if( NULL!=me ) {
    me->reset_log(this);
  }
}

LogProxy::~LogProxy() {
  if( NULL!=me ) {
    me->reset_log();
  }
}

// observers

Wt::Dbo::Session &LogProxy::db() const {
  return me->m_log_session;
}

// T-REX callbacks

void LogProxy::message(boost::optional<LogProxy::date_type> const &date, 
                       LogProxy::id_type const &who, LogProxy::id_type const &kind,
                       LogProxy::msg_type const &what) {
  // I filter out the messages with no type
  if( utils::null!=kind && NULL!=me ) {
    try {
      // Send message to the database
      boost::optional<long long> db_date;
      if( date )
        db_date = *date;
      
      Wt::Dbo::ptr<dbo::MsgType> type;
      { // Check or create new type 
        Wt::Dbo::Transaction tr(db());
        
        type = db().find<dbo::MsgType>().where("type_id = ?").bind(kind.str());
        if( !type ) {
          dbo::MsgType *db_type = new dbo::MsgType;
          db_type->identifier = kind.str();
          type = db().add(db_type);
        }
        tr.commit();
      }
      boost::optional<long long> w_date;
      if( date )
        w_date = *date;
      
      { // create the new message
        Wt::Dbo::Transaction tr(db());
        dbo::Msg *entry = new dbo::Msg;
        entry->date = w_date;
        entry->type = type;
        entry->source = who.str();
        entry->content = what;
        db().add(entry);
        
        tr.commit();
      }
      // Notify listeners
      m_log.emit(kind.str(), w_date, who.str(), what);
    } catch(Wt::Dbo::Exception const &e) {
      std::cerr<<" >>> DB exception: "<<e.what()<<std::endl;
    }
  }
}
