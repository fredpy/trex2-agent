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

#include <Wt/WMessageBox>
#include <Wt/WHBoxLayout>
#include <Wt/WTemplate>

using namespace TREX::witre;

/*
 * class TREX::witre::WitreApp
 */

// structors

WitreApp::WitreApp(Wt::WEnvironment const &env, WitreServer &server) 
:Wt::WApplication(env), m_server(server) {
  boost::filesystem::path file = m_server.locales("witre_loc.xml");

  if( file.empty() ) {
    log(TREX::utils::error)<<"Unable to find locale files";
  } else {
    messageResourceBundle().use(file.string());
  }
                              
  m_server.log()<<"New session from ["<<sessionId()<<"]\n\tclient locale is "
  <<env.locale();
  setTitle(tr("trex").arg(TREX::version::str()));
  internalPathChanged().connect(this, &WitreApp::path_changed);
  
  Wt::WMessageBox::show(tr("about"), 
                        tr("about_dialog").arg(TREX::version::str()), 
                        Wt::Ok);
}

WitreApp::~WitreApp() {
}

void WitreApp::path_changed(std::string const &path) {
  log(TREX::utils::info)<<"Client ["<<sessionId()<<"] changed path to "<<path;
}







