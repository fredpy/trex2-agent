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

namespace {
  
  TeleoReactor::xml_factory::declare<Player> decl("LogPlayer");

} // ::

/*
 * class TREX::transaction::Player::timeline_transaction
 */ 

Player::timeline_transaction::timeline_transaction(rapidxml::xml_node<> &node)
  :m_name(parse_attr<Symbol>(node, "name")) {}

/*
 * class TREX::transaction::Player::op_assert
 */ 

Player::op_assert::op_assert(rapidxml::xml_node<> &node) 
  :m_obs(node) {}

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

// structors

Player::Player(TeleoReactor::xml_arg_type arg) 
  :TeleoReactor(arg, false), m_continue(true) {
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
  rapidxml::xml_document<> doc;
  rapidxml::file<> file(file_name.c_str());

  doc.parse<0>(file.data());
  rapidxml::xml_node<> *root = doc.first_node();
  if( NULL==root ) {
    syslog("ERROR")<<"Transaction log \""<<file_name<<"\" is empty.";
    throw ReactorException(*this, "Empty transaction file.");
  }
  loadTransactions(*root);
}

Player::~Player() {
  for( std::map<TICK, std::list<transaction *> >::iterator i=m_exec.begin();
       m_exec.end()!=i; ++i )
    clear(i->second);
  m_exec.clear();
}

// modifiers 


void Player::loadTransactions(rapidxml::xml_node<> &node) {
  ext_iterator xlog(node), it;

  if( !xlog.valid() ) {
    syslog("ERROR")<<"Transaction log has no sub-node.";
    throw XmlError(node, "Empty transaction log file");
  }
  it = xlog.find_tag("header");
  if( !it.valid() ) {
    syslog("WARN")<<"Transaction log file does not have an header.";
  } else {
    Symbol tl;
    for(it = ext_iterator(*it); it.valid(); ++it) {
      if( is_tag(*it, "provide") ) {
	op_provide(*it).accept(*this);
      }  else if( is_tag(*it, "use") ) {
	op_use(*it).accept(*this);
      } else if( is_tag(*it, "unprovide") ) {
	op_unprovide(*it).accept(*this);
      }  else if( is_tag(*it, "unuse") ) {
	op_unuse(*it).accept(*this);
      } else 
	syslog("WARN")<<"Unexpected tag "<<it->name()<<" in transaction header.";
    }
  }
  for( xlog = xlog.find_tag("tick");  xlog.valid(); xlog = xlog.next("tick") ) {
    TICK cur = parse_attr<TICK>(*xlog, "value");
    std::list<transaction *> &ops = m_exec[cur];

    for( it=ext_iterator(*xlog); it.valid(); ++it) {
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
      } else 
	syslog("WARN")<<"Ignoring tag "<<it->name();      
    }
  }
}

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
