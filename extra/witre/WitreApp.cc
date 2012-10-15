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
#include "Popup.hh"

#include <trex/utils/TREXversion.hh>

#include <Wt/WVBoxLayout>
#include <Wt/WFileResource>
#include <Wt/WEnvironment>
#include <Wt/WTemplate>
#include <Wt/WMenu>
#include <Wt/WSubMenuItem>
#include <Wt/WMessageBox>

using namespace TREX::witre;

/*
 * class TREX::witre::WitreApp
 */

// structors

WitreApp::WitreApp(Wt::WEnvironment const &env, WitreServer &server) 
:Wt::WApplication(env), m_home(NULL) {
  setTitle("T-REX");
  boost::filesystem::path file = server.locales("witre_loc.xml");

  if( file.empty() ) {
    m_log->syslog("witre", TREX::utils::error)<<"Unable to find locale files";
  } else {
    messageResourceBundle().use(file.string());
  }
                              
  m_log->syslog("witre")<<"New session from "<<env.userAgent();
  setup();
}

WitreApp::~WitreApp() {
}

// modifiers 


void WitreApp::setup() {
  if( NULL==m_home ) {
    createHome();
  }
}

void WitreApp::createHome() {
  Wt::WTemplate *view = new Wt::WTemplate(root()); 
  m_home = view;
  // setLocale("fr");
  view->addFunction("tr", &Wt::WTemplate::Functions::tr);
  view->setTemplateText(tr("agent_info"));
                        
  Wt::WMenu *menu = new Wt::WMenu(Wt::Vertical, root());
  menu->setRenderAsList(true);
  Wt::WSubMenuItem *admin = new Wt::WSubMenuItem(tr("admin"), NULL); 
  Wt::WMenu *trex_menu = new Wt::WMenu(Wt::Vertical, root());
  trex_menu->setRenderAsList(true);
  trex_menu->addItem(tr("new_agent"), NULL);
  trex_menu->addItem(tr("kill_agent"), NULL);
  
  admin->setSubMenu(trex_menu);
  menu->addItem(admin);
  
  view->bindString("count", 
                   Wt::WString::trn("agent", menu->items().size()-1).arg((int)menu->items().size()-1));
  view->bindWidget("agents", menu);
  Wt::WText *about = new Wt::WText(tr("about"), root());
  about->clicked().connect(this, &WitreApp::about);
  view->bindWidget("about", about);
}

void WitreApp::about() {
  Wt::WTemplate about;
  about.setTemplateText(tr("about_text"));
  about.bindString("trex_version", TREX::version::str());
  std::ostringstream oss;
  about.htmlText(oss);
  
  m_log->syslog("witre")<<oss.str();
  Wt::WMessageBox::show(tr("about"), oss.str(), Wt::Ok);
}






