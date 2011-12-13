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

    //clock = new Wt::WText("",root());
    new Wt::WText("Current ",root());
    tickNum = new Wt::WText("Tick Value: 0", root());
    tickNum->setMargin(5,Wt::Right);
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
    input->setMargin(5, Wt::Left);
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

    timeLineSlider = new Wt::WSlider(Wt::Horizontal, root());
    timeLineSlider->setMargin(100, Wt::Left);
    //timeLineSlider->setTickPosition(Wt::WSlider::TicksBothSides);
    timeLineSlider->setTickInterval(wServer->tickDuration());
    timeLineSlider->setRange(0, wServer->getFinalTick());
    timeLineSlider->resize(400, 50);
    //timeLineSlider->valueChanged().connect(this, &WitreApplication::sliderChanged );
    //timeLineSlider->valueChanged().connect(this, &WitreApplication::sliderText);
    timeLineSlider->sliderMoved().connect(this, &WitreApplication::sliderText);

    sliderTime = new Wt::WText(root());
    new Wt::WBreak(root());
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
    std::string time = observations.front().getTime();

    if(name=="Tick")
    {
        updateTick(observ);
        observations.pop();
        timeLineSlider->setValue(timeLineSlider->value()+1);
        sliderText();
        Wt::WApplication::instance()->triggerUpdate();
        return;
    }
    Wt::WPanel* panel = new Wt::WPanel();
    panel->setCentralWidget(new Wt::WText(observ)); // Addes the most recent observation to webpage
    panel->setObjectName(name); // Names the box by the timeline
    observations.pop(); //Pops the most recent observation
    //box->setCollapsible(true);
    //box->setTitleBar(true);
    //box->setTitle(tickNum->text());
    if(!tLineMap[name])
    {
        panel->hide();
    }
    insert(time, panel);
}

void WitreApplication::insert(std::string time, Wt::WPanel* wid)
{
    //const Wt::WAnimation animate(Wt::WAnimation::Fade, Wt::WAnimation::Ease, 10000);
    //wid->setAnimation(animate);
    if(boxPanels[time]==NULL)
    {
        Wt::WContainerWidget* temp = new Wt::WContainerWidget();
        temp->setObjectName(time);
        boxPanels[time]= temp;
        messages->insertWidget(0, boxPanels[time]);
    }
    //messages->insertWidget(0, wid);
    boxPanels[time]->insertWidget(0, wid);
    Wt::WApplication::instance()->triggerUpdate();
}

void WitreApplication::updateTime()
{
    Wt::WTime* time = new Wt::WTime;
    *time = time->currentServerTime();
    clock->setText(time->toString());
}

void WitreApplication::addObs(Observations* temp)
{
    observations.push(*temp);
    return;
}

void WitreApplication::timeLineChange()
{
    Wt::WObject* timeLine = sender();
    std::string tName = timeLine->objectName();
    std::map<std::string, Wt::WContainerWidget*>::iterator it;
    for(it=boxPanels.begin(); it!=boxPanels.end(); it++)
    {
        Wt::WContainerWidget* container = (*it).second;
        for(int i=0; i<container->count(); i++)
        {
            Wt::WWidget* tempWidget = container->widget(i);
            if(tempWidget->objectName()==tName)
            {
                if(tLineMap[tName])
                {
                    tempWidget->hide();
                } else {
                    tempWidget->show();
                }
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
        Wt::StandardButton incorrect = Wt::WMessageBox::show("Incorrect", "Invaild input for goal"
                                                             , Wt::Ok);
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
            Gpanel->setObjectName(goalid->object().str());
            //Wt::WPushButton* recall = new Wt::WPushButton("Recall");
            std::stringstream oss;
            oss<<"On Timeline <b>"<<goalid->object()<<"</b>, placed goal <b>"<<goalid->predicate()<<"</b>"
               <<" at the time: "<<wServer->getTime_t()<<"<br />"
               <<"Starting: "<<start<<"<br />"
               <<"Duration: "<<duration<<"<br />"
               <<"End: "<<end<<"<br />";
            Wt::WText* text = new Wt::WText(oss.str());
            Wt::WContainerWidget* container = new Wt::WContainerWidget;
            container->addWidget(text);
            //container->addWidget(recall);
            //recall->clicked().connect(goalid,&WitreServer::postRecall);
            Gpanel->setCentralWidget(container);
            if(!tLineMap[goalid->object().str()])
            {
                Gpanel->hide();
            }
            std::stringstream time;
            time<<wServer->getCurrentTick();
            insert(time.str(), Gpanel);
        }

    }
    popup->clearText();

}

void WitreApplication::sliderChanged()
{
    std::stringstream value;
    value<<timeLineSlider->value();
    std::map<std::string, Wt::WContainerWidget*>::iterator it;
    for(it = boxPanels.begin(); it!= boxPanels.end(); it++ )
    {
        if(boost::lexical_cast<int>((*it).first) > boost::lexical_cast<int>(value.str()))
        {
            (*it).second->hide();
        }
        else
        {
            (*it).second->show();
        }
    }
}

void WitreApplication::sliderText()
{
    std::stringstream value;
    value<<timeLineSlider->value();
    sliderTime->setText(value.str());
    Wt::WApplication::instance()->triggerUpdate();
}
