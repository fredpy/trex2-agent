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
#include "TransactionPlayer.hh"
#include <set>

using namespace TREX::transaction;
using namespace TREX::utils;
namespace xml = boost::property_tree::xml_parser;

namespace {
  
  TeleoReactor::xml_factory::declare<Player> decl("LogPlayer");

} // ::

/*
 * class TREX::transaction::Player::timeline_transaction
 */ 

Player::timeline_transaction::timeline_transaction(boost::property_tree::ptree::value_type &node)
  :m_name(parse_attr<Symbol>(node, "name")) {}

/*
 * class TREX::transaction::Player::op_assert
 */ 

Player::op_assert::op_assert(boost::property_tree::ptree::value_type &node) 
  :m_obs(node) {}

/*
 * class TREX::transaction::Player::op_request
 */ 
Player::op_request::op_request(boost::property_tree::ptree::value_type &node)
  :m_id(parse_attr<std::string>(node, "id")) {
  boost::property_tree::ptree::assoc_iterator i = node.second.find("Goal");
  if( node.second.not_found()==i ) 
    throw XmlError(node, "Unable to find Goal description");
  m_request.reset(new Goal(*i));  
}

/*
 * class TREX::transaction::Player::op_recall
 */ 

Player::op_recall::op_recall(boost::property_tree::ptree::value_type &node)
  :m_id(parse_attr<std::string>(node, "id")) {
}


/*
 * class TREX::transaction::Player::op_token
 */ 
Player::op_token::op_token(boost::property_tree::ptree::value_type &node)
:m_id(parse_attr<std::string>(node, "id")) {
  boost::property_tree::ptree::assoc_iterator i = node.second.find("Goal");
  if( node.second.not_found()==i ) 
    throw XmlError(node, "Unable to find Goal description");
  m_token.reset(new Goal(*i));  
}

/*
 * class TREX::transaction::Player::op_recall
 */ 

Player::op_cancel::op_cancel(boost::property_tree::ptree::value_type &node)
:m_id(parse_attr<std::string>(node, "id")) {
}


/*
 * class TREX::transaction::Player
 */

// statics

void Player::clear(std::list<Player::transaction *> &l) {
  for(std::list<Player::transaction *>::iterator i=l.begin();
      l.end()!=i; ++i) 
    delete *i;
  l.clear();
}

TeleoReactor::xml_arg_type &Player::transform(TeleoReactor::xml_arg_type &arg) {
  set_attr(TeleoReactor::xml_factory::node(arg), "log", 0);
  return arg;
}


// structors

Player::Player(TeleoReactor::xml_arg_type arg) 
  :TeleoReactor(transform(arg), false), m_continue(true) {
  std::string
    file_name = parse_attr<std::string>(getName().str()+".tr.log",
					TeleoReactor::xml_factory::node(arg),
					"file");
  bool found;

  file_name = manager().use(file_name, found);
  if( !found ) {
    syslog("ERROR")<<"Unable to locate transaction log \""<<file_name<<"\".";
    throw ReactorException(*this, "Unable to locate specified transaction log file.");
  }
  boost::property_tree::ptree pt;
  
  read_xml(file_name, pt, xml::no_comments|xml::trim_whitespace);

  if( pt.empty() ) {
    syslog("ERROR")<<"Transaction log \""<<file_name<<"\" is empty.";
    throw ReactorException(*this, "Empty transaction file.");    
  }
  if( pt.size()!=1 ) {
    syslog("ERROR")<<"Transaction log \""<<file_name<<"\" has more than 1 root.";
    throw ReactorException(*this, "Invalid transaction file.");    
  }
  loadTransactions(pt.front().second);
}

Player::~Player() {
  for( std::map<TICK, std::list<transaction *> >::iterator i=m_exec.begin();
       m_exec.end()!=i; ++i )
    clear(i->second);
  m_exec.clear();
}

// modifiers 


void Player::loadTransactions(boost::property_tree::ptree &properties) {
  boost::property_tree::ptree::assoc_iterator i, last;

  boost::tie(i, last) = properties.equal_range("header");
  if( last==i )
    syslog("WARN")<<"Transaction log file does not have an header.";
  else {
    Symbol tl;
    for(;last!=i; ++i) {
      for(boost::property_tree::ptree::iterator j=i->second.begin(); 
	  i->second.end()!=j; ++j) {
	if( is_tag(*j, "provide") ) {
	  op_provide(*j).accept(*this);
	}  else if( is_tag(*j, "use") ) {
	  op_use(*j).accept(*this);
	} else if( is_tag(*j, "unprovide") ) {
	  op_unprovide(*j).accept(*this);
	}  else if( is_tag(*j, "unuse") ) {
	  op_unuse(*j).accept(*this);
	} else if( !is_tag(*j, "<xmlattr>") ) //< <xmlattr> can be silently ignored 
	  syslog("WARN")<<"Unexpected tag \""<<j->first<<"\" in transaction header.";
      }
    }
  }
  size_t tr_count = 0;
  boost::tie(i, last) = properties.equal_range("tick");
  for( ; last!=i; ++i) {
    TICK cur = parse_attr<TICK>(*i, "value");
    std::list<transaction *> &ops = m_exec[cur];    
    for(boost::property_tree::ptree::iterator it=i->second.begin();
	i->second.end()!=it; ++it) {
      if( is_tag(*it, "provide") ) {
	ops.push_back(new op_provide(*it));
      }  else if( is_tag(*it, "use") ) {
	ops.push_back(new op_use(*it));
      } else if( is_tag(*it, "unprovide") ) {
	ops.push_back(new op_unprovide(*it));
      }  else if( is_tag(*it, "unuse") ) {
	ops.push_back(new op_unuse(*it));
      } else if( is_tag(*it, "Observation") ) {
	ops.push_back(new op_assert(*it));
      } else if( is_tag(*it, "request") ) {
	ops.push_back(new op_request(*it));
      } else if ( is_tag(*it, "recall") ) {
	ops.push_back(new op_recall(*it));
      } else if( is_tag(*it, "token") ) {
	ops.push_back(new op_token(*it));
      } else if ( is_tag(*it, "cancel") ) {
	ops.push_back(new op_cancel(*it));
      } else if( !is_tag(*it, "<xmlattr>") ) //< <xmlattr> can be silently ignored 
	syslog("WARN")<<"Ignoring tag \""<<it->first<<'\"';	
    }
    tr_count += ops.size();
    // if( !ops.empty() )
    //   syslog()<<"Loaded "<<ops.size()<<" operations for tick "<<cur;
  }
  if( m_exec.empty() ) 
    syslog("WARN")<<"The log from the reactor did not do anything !";
  else 
    syslog()<<tr_count<<" transactions extracted from the log";
}

void Player::play_request(std::string const &id, goal_id const &g) {  
  if( !postGoal(g) )
    syslog("WARN")<<"Failed to post goal "<<g;
  else
    m_goals[id] = g;
}

void Player::play_recall(std::string const &id) {
  std::map<std::string, goal_id>::iterator i = m_goals.find(id);
  if( m_goals.end()==i )
    syslog("WARN")<<"goal \""<<id<<"\" not found in my lookup table.";
  else {
    syslog("recall")<<"RECALL "<<i->second<<" (Log ID: "<<i->first<<").";    
    postRecall(i->second);
    m_goals.erase(i);
  }
}

void Player::play_token(std::string const &id, goal_id const &g) {
  if( !postPlanToken(g) )
    syslog("WARN")<<"Failed to post plan token "<<g;
  else
    m_goals[id] = g;  
}

void Player::play_cancel(std::string const &id) {
  std::map<std::string, goal_id>::iterator i = m_goals.find(id);
  if( m_goals.end()==i )
    syslog("WARN")<<"token \""<<id<<"\" not found in lookup table.";
  else {
    syslog("cancel")<<"CANCEL "<<i->second<<" (Log ID: "<<i->first<<").";
    cancelPlanToken(i->second);
    m_goals.erase(i);
  }
}


// calbacks

void Player::handleTickStart() {
  TICK cur = getCurrentTick();
  std::map<TICK, std::list<transaction *> >::iterator i = m_exec.find(cur);
  if( m_exec.end()!=i ) {
    while( !i->second.empty() ) {
      transaction *tmp = i->second.front();
      i->second.pop_front();
      tmp->accept(*this);
      delete tmp;
    }
    m_exec.erase(i);
  }
}

bool Player::synchronize() {
  return m_continue;
}
