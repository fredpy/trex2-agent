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
#include "WitreApp.hh"

#include <trex/utils/TREXversion.hh>

#include <Wt/WMessageBox>
#include <Wt/WText>
#include <Wt/WEnvironment>
#include <Wt/WLineEdit>
#include <Wt/WPushButton>
#include <Wt/WTimer>

#include <boost/scope_exit.hpp>

using namespace TREX::witre;

/*
 * class TREX::witre::WitreApp
 */
WitreApp::WitreApp(Wt::WEnvironment const &env, Server &server) 
:Wt::WApplication(env), m_server(server), m_agent_updated(true), 
m_log_count(0) {  
  // Menu
  Wt::WText *menu_t = new Wt::WText("Menu");
  menu_t->addStyleClass("menu");
  root()->addWidget(menu_t);
  menu_t->clicked().connect(&m_menu, &Wt::WPopupMenu::popup);
  m_menu.triggered().connect(this, &WitreApp::updateAgentState);
  
  m_start = m_menu.addItem("Start ...", this, &WitreApp::start_agent);
  m_stop = m_menu.addItem("Stop", this, &WitreApp::stop_agent);
  m_menu.addItem("Ping", this, &WitreApp::ping);
  updateAgentState();
  
  m_menu.addSeparator();
  m_menu.addItem("About ...", this, &WitreApp::about);

  m_log = new TREXLogWidget(m_server, root());
  m_server.connect(this);
  
  m_refresh = new Wt::WTimer(this);
  m_refresh->setInterval(1000); // refresh every 1s
  m_refresh->timeout().connect(this, &WitreApp::process_updates);
  m_refresh->start();
}

WitreApp::~WitreApp() {
  if( m_refresh->isActive() )
    m_refresh->stop();
  m_server.disconnect(this);
}

void WitreApp::process_updates() {
  if( m_agent_updated )
    updateAgentState();
  if( m_log_count>0 ) {
    m_log_count = 0;
    m_log->updated();
  }
}

void WitreApp::start_agent() {
  if( m_server.reserve() ) {
    WitreApp &me = *this;
    BOOST_SCOPE_EXIT((&me)) {
      me.m_server.reserve(false);
      me.updateAgentState();
    } BOOST_SCOPE_EXIT_END;
    Wt::WDialog ask_name("Create Agent");
    new Wt::WText("Enter agent name: ", ask_name.contents());
    Wt::WLineEdit name(ask_name.contents());
    new Wt::WBreak(ask_name.contents());
    
    Wt::WPushButton ok("OK", ask_name.contents());
    
    name.enterPressed().connect(&ask_name, &Wt::WDialog::accept);
    ok.clicked().connect(&ask_name, &Wt::WDialog::accept);
    
    if( ask_name.exec()==Wt::WDialog::Accepted ) {
      // Do something
      std::string agent_name = name.text().toUTF8();
      try {
        m_server.create_agent(agent_name);
      } catch(std::exception const &e) {
        Wt::WMessageBox::show("Error", e.what(), Wt::Cancel);
      }
    }
  } else 
    updateAgentState();
}

void WitreApp::stop_agent() {
  if( Wt::WMessageBox::show("Stop", "Do you really want to stop the agent ?",
                            Wt::Ok|Wt::Cancel)==Wt::Ok ) {
    // Need to do something
    m_server.info()<<"Agent termination requested by witre user.";
  }
}

void WitreApp::log_updated() {
  ++m_log_count;
}


void WitreApp::agent_updated() {
  m_agent_updated = true; 
}


void WitreApp::ping() {
  m_server.info()<<"Ping from "<<environment().userAgent();
}


void WitreApp::updateAgentState() {
  if( m_agent_updated ) {
    setTitle("T-REX - "+m_server.name());
    m_start->setDisabled(m_server.reserved());
    m_stop->setDisabled(m_server.completed());
    m_agent_updated = false;
  }
}


void WitreApp::about() {
  Wt::WMessageBox::show("About", "This is witre for T-REX "+TREX::version::str(),
                        Wt::Ok);
}

