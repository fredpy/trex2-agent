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
#include "Witre.hh"

#include <trex/utils/TREXversion.hh>
#include <trex/utils/Plugin.hh>

#include <Wt/WContainerWidget>
#include <Wt/WText>

using namespace TREX::utils;
using namespace TREX::witre;

namespace {

  SingletonUse<LogManager> s_log;

}

TREX::witre::WitreApplication *TREX::witre::createWitre(Wt::WEnvironment const &e, WitreServer* Server) {
      WitreApplication *app = new WitreApplication(e,Server);
      // hard coded for now

      bool found;
      std::string file = s_log->use("witre_loc.xml", found);

      if( found ) {
            file.erase(file.end()-4, file.end());
            s_log->syslog("witre")<<"set locale to "<<file;
            app->messageResourceBundle().use(file);
      } else {
            s_log->syslog("witre")<<"Did not find locale "<<file;
      }

      return app;
}


WitreApplication::WitreApplication(Wt::WEnvironment const &env, WitreServer* Server)
  :Wt::WApplication(env) {
    wServer = Server;
    wServer->connect(this, boost::bind(&WitreApplication::post, this));
    enableUpdates(true);

    setTitle("Witre - trex "+TREX::version::str());
    //root()->addWidget(new Wt::WText(Wt::WString::tr("test")));

    //setCssTheme("Polished");

    clock = new Wt::WText("",root());
    new Wt::WBreak(root());

    Wt::WGroupBox *tLines = new Wt::WGroupBox("Available Timelines", root());
    for(int i = 0; i<wServer->extTimelinesSize(); i++)
    {
        std::string name = wServer->extTimelinesName(i);
        tLineMap[name]=true;
        Wt::WCheckBox* temp = new Wt::WCheckBox(name, tLines);
        temp->setChecked(true);//
        temp->setObjectName(name);
        temp->changed().connect(this, &WitreApplication::timeLineChange);
    }
    new Wt::WText("Current ",root());
    tickNum = new Wt::WText("Tick Value: 0", root());
    tickNum->setMargin(5,Wt::Right);
    new Wt::WBreak(root());
    menu = new Wt::WComboBox(root());
    for(int i = 0; i<wServer->extTimelinesSize(); i++)
    {
        std::string name = wServer->extTimelinesName(i);
        if(wServer->acceptsGoal(wServer->extTimelinesName(i)))
        {
            menu->addItem(name);
        }
    }
    input= new Wt::WLineEdit(root());

    enter = new Wt::WPushButton("Post Goal", root());
    enter->setMargin(5,Wt::Left);
    enter->clicked().connect(this, &WitreApplication::attributePopup);
    enter->clicked().connect(enter, &Wt::WWidget::disable);
    input->enterPressed().connect(boost::bind(&WitreApplication::attributePopup, this));
    input->enterPressed().connect(enter, &Wt::WWidget::disable );

    popup = new Goalpopup(this->root());
    popup->finished().connect(this, &WitreApplication::clientPostGoal);
    popup->finished().connect(enter, &Wt::WPushButton::enable);
    popup->cancelled().connect(enter, &Wt::WPushButton::enable);

    new Wt::WBreak(root());

    observations = wServer->receiveObs();
    Wt::WTimer::singleShot(100, this, &WitreApplication::syncObservations); //Calls function after 100mill

    messages = new Wt::WContainerWidget(root());

    //timer = new Wt::WTimer();
    //timer->setInterval(1000);
    //timer->timeout().connect(this, &WitreApplication::updateTime);
    //timer->start();
}

WitreApplication::~WitreApplication()
{
    wServer->disconnect(this);
    enableUpdates(false);
}

void WitreApplication::post()
{
    std::string name = observations.front().getObj();
    std::string observ = observations.front().getObs();

    if(name=="Tick")
    {
        updateTick(observ);
        observations.pop();
        Wt::WApplication::instance()->triggerUpdate();
        return;
    }
    Wt::WPanel* box = new Wt::WPanel();
    box->setCentralWidget(new Wt::WText(observ)); // Addes the most recent observation to webpage
    box->setObjectName(name); // Names the box by the timeline
    observations.pop(); //Pops the most recent observation
    //box->setCollapsible(true);
    //box->setTitleBar(true);
    //box->setTitle(tickNum->text());
    if(!tLineMap[name])
    {
        box->hide();
    }
    insert(box);
}

void WitreApplication::insert(Wt::WPanel* wid)
{
    const Wt::WAnimation animate(Wt::WAnimation::Pop, Wt::WAnimation::Ease, 10000);
    messages->insertWidget(0, wid);
    Wt::WApplication::instance()->triggerUpdate();
}

void WitreApplication::updateTime()
{
    Wt::WTime* time = new Wt::WTime;
    *time = time->currentServerTime();
    clock->setText(time->toString());
}

void WitreApplication::addObs(std::string obs, std::string obj)
{
    Observations* temp = new Observations(obs,obj);
    observations.push(*temp);
    return;
}

void WitreApplication::timeLineChange()
{
    Wt::WObject* timeLine = sender();
    std::string tName = timeLine->objectName();
    for(int i=0; i<messages->count(); i++)
    {
        Wt::WWidget* temp = messages->widget(i);
        if(temp->objectName()==tName)
        {
            if(tLineMap[tName])
            {
                temp->hide();
            } else {
                temp->show();
            }
        }
    }
    // Set the timeline as hidden or show after going through all of the messages
    if(tLineMap[tName])
    {
        tLineMap[tName]=false;
    } else {
        tLineMap[tName]=true;
    }

}

void WitreApplication::syncObservations()
{
    while(!observations.empty())
    {
        this->post();
    }
}

void WitreApplication::attributePopup()
{
    std::string object = menu->currentText().toUTF8();
    std::string predicate = input->text().toUTF8();
    if(!object.empty() && !predicate.empty())
    {
        //popup = new Goalpopup(this->root());
        popup->setPosition(input);
        popup->valRange(wServer->getCurrentTick(),wServer->getFinalTick());
        popup->setVisable();
    }
    else{

        enter->enable();
    }
}

void WitreApplication::clientPostGoal(transaction::IntegerDomain start, transaction::IntegerDomain duration, transaction::IntegerDomain end)
{
    std::string object = menu->currentText().toUTF8();
    std::string predicate = input->text().toUTF8();
    if(!object.empty() && !predicate.empty())
    {
        TREX::transaction::Goal goal = wServer->getGoal(object,predicate);
        goal.restrictStart(start);
        goal.restrictDuration(duration);
        goal.restrictEnd(end);
        TREX::transaction::goal_id goalid = wServer->clientGoalPost(goal);
        if(goalid!=NULL)
        {
            Wt::WPanel* Gpanel = new Wt::WPanel();
            Wt::WPushButton* recall = new Wt::WPushButton("Recall");
            recall->clicked().connect(goalid,&WitreServer::postRecall);
            Gpanel->setCentralWidget(recall);
            //Gpanel->setObjectName(goalid);
        }

    }
    popup->clearText();

}

