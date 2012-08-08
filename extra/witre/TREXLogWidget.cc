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

using namespace TREX::witre;

/*
 * class TREX::witre::TREXLogWidget
 */

// structors 

TREXLogWidget::TREXLogWidget(Server &server, 
                             Wt::WContainerWidget *parent)
  :Wt::WPanel(parent), m_server(server), 
   m_req(server.log_model()) {
     
  m_table = new Wt::WTableView;
  m_table->setModel(m_req);
  m_table->setAlternatingRowColors(true);
  m_table->setColumnWidth(0, 50);
  m_table->setColumnWidth(3, 500);
  m_table->setHeight(100);
  m_table->setWidth(Wt::WLength::Auto);
  m_table->setSortingEnabled(false);
  
  setTitle("TREX log");
  setCollapsible(true);
  setCollapsed(true);
  setCentralWidget(m_table);
  addStyleClass("trex-log");
  setPositionScheme(Wt::Fixed);
  setOffsets(0, Wt::Bottom | Wt::Left | Wt::Right);
}

TREXLogWidget::~TREXLogWidget() {
  delete m_req;
}

void TREXLogWidget::updated() {
  m_req->reload();
}


