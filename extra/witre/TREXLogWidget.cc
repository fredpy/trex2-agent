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
#include "TREXLogWidget.hh"

#include <Wt/WText>

using namespace TREX::witre;

/*
 * class TREX::witre::TREXLogWidget
 */

// structors 

TREXLogWidget::TREXLogWidget(WitreServer &server, 
                             Wt::WContainerWidget *parent)
:Wt::WContainerWidget(parent) {
  std::set<std::string> types;
  server.log_types(types);
  
  Wt::WCssStyleSheet &style = Wt::WApplication::instance()->styleSheet();
  
 
  // Populate the menu
  for(std::set<std::string>::const_iterator i=types.begin();
      types.end()!=i; ++i) {
    Wt::WPopupMenuItem *item = m_type_select.addItem(*i);
    item->setCheckable(true);
    item->setChecked(true);
    item->triggered().connect(this, &TREXLogWidget::show_hide);

    m_types[*i] = style.addRule("."+(*i), Wt::WCssDecorationStyle());
    m_types[*i]->templateWidget()->show();
  }
    
  // Populate the logs with last 100 messages
  WitreServer::msg_set log = server.last_messages(100);
  for(WitreServer::msg_set::const_iterator i=log.begin();
      i!=log.end(); ++i) 
    add_msg(i->get<0>(), i->get<1>(), i->get<2>(), i->get<3>());
  
  clicked().connect(&m_type_select, &Wt::WPopupMenu::popup);
  Wt::WApplication::instance()->enableUpdates(true);
}

TREXLogWidget::~TREXLogWidget() {
  
}

// signal callbacks


void TREXLogWidget::new_msg(std::string type, boost::optional<long long> date,
                            std::string src, std::string content) {
  add_msg(type, date, src, content);
  Wt::WApplication::instance()->triggerUpdate();
}


void TREXLogWidget::add_msg(std::string type, boost::optional<long long> date,
                            std::string src, std::string content) {
  std::map<std::string, Wt::WCssTemplateRule *>::iterator pos;
  bool inserted;
  Wt::WCssStyleSheet &style = Wt::WApplication::instance()->styleSheet();
  
  boost::tie(pos, inserted) = m_types.insert(std::map<std::string, Wt::WCssTemplateRule *>::value_type(type, NULL));
  if( inserted ) {
    Wt::WPopupMenuItem *item = m_type_select.addItem(type);
    item->setCheckable(true);
    item->setChecked(true);
    item->triggered().connect(this, &TREXLogWidget::show_hide);

    pos->second = style.addRule("."+type, Wt::WCssDecorationStyle());
    pos->second->templateWidget()->show();
  }
  Entry *entry = new Entry(date, src, content);
  insertWidget(0, entry);
  entry->setStyleClass(type);
  entry->setToolTip(type);
}

void TREXLogWidget::show_hide(Wt::WPopupMenuItem *item) {
  std::map<std::string, Wt::WCssTemplateRule *>::iterator 
    pos = m_types.find(item->text().toUTF8());
  if( pos!=m_types.end() ) {
    pos->second->templateWidget()->setHidden(item->isChecked());
  }
}


/*
 *
 */
TREXLogWidget::Entry::Entry(boost::optional<long long> date, 
                            std::string src, std::string content,
                            TREXLogWidget *parent)
:Wt::WText(parent) {
  std::ostringstream oss;
  oss<<"<span class=\"trex_tick\">";
  if( date )
    oss<<(*date);
  oss<<"</span><span class=\"trex_src\">["<<src
     <<"] </span><span class=\"trex_msg\">"<<content<<"</span>";
  setText(oss.str());
}

