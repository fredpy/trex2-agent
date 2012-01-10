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

namespace xml = boost::property_tree::xml_parser;

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

    //setCssTheme("Polished");
    Wt::WContainerWidget* north = new Wt::WContainerWidget();
    Wt::WContainerWidget* center = new Wt::WContainerWidget();
    Wt::WContainerWidget* east = new Wt::WContainerWidget();
    east->setContentAlignment(Wt::AlignTop | Wt::AlignRight);
    Wt::WContainerWidget* west = new Wt::WContainerWidget();
    Wt::WContainerWidget* south = new Wt::WContainerWidget();
    south->setContentAlignment(Wt::AlignBottom | Wt::AlignCenter);

    //Start of North frame code
    new Wt::WText("Current ",north);
    tickNum = new Wt::WText("Tick Value: 0", north);
    tickNum->setMargin(5,Wt::Right);
    new Wt::WBreak(north);

    Wt::WGroupBox *tLines = new Wt::WGroupBox("Available Timelines", north);
    for(int i = 0; i<wServer->extTimelinesSize(); i++)
    {
        std::string name = wServer->extTimelinesName(i);
        tLineMap[name]=true;
        Wt::WCheckBox* temp = new Wt::WCheckBox(name, tLines);
        temp->setChecked(true);//
        temp->setObjectName(name);
        temp->changed().connect(this, &WitreApplication::timeLineChange);
    }
    //End of North code

    //Start of East code
    menu = new Wt::WComboBox(east);
    for(int i = 0; i<wServer->extTimelinesSize(); i++)
    {
        std::string name = wServer->extTimelinesName(i);
        if(wServer->acceptsGoal(wServer->extTimelinesName(i)))
        {
            menu->addItem(name);
        }
    }
    input= new Wt::WLineEdit(east);
    input->setMargin(5, Wt::Left);
    enter = new Wt::WPushButton("Post Goal", east);
    enter->setMargin(5,Wt::Left);
    enter->clicked().connect(this, &WitreApplication::attributePopup);
    enter->clicked().connect(enter, &Wt::WWidget::disable);
    input->enterPressed().connect(boost::bind(&WitreApplication::attributePopup, this));
    input->enterPressed().connect(enter, &Wt::WWidget::disable );

    popup = new Goalpopup(east);
    popup->finished().connect(this, &WitreApplication::clientPostGoal);
    popup->finished().connect(enter, &Wt::WPushButton::enable);
    popup->cancelled().connect(enter, &Wt::WPushButton::enable);
    //End of East Code

    //Start of South Code
    timeLineSlider = new Wt::WSlider(Wt::Horizontal, south);
    //timeLineSlider->setTickPosition(Wt::WSlider::TicksBothSides);
    timeLineSlider->setTickInterval(wServer->tickDuration());
    timeLineSlider->setRange(0, wServer->getFinalTick());
    timeLineSlider->resize(400, 50);
    timeLineSlider->valueChanged().connect(this, &WitreApplication::sliderChanged );
    timeLineSlider->valueChanged().connect(this, &WitreApplication::sliderText);

    sliderTime = new Wt::WText(south);
    //End of South Code

    //Start of Center Code
    messages = new Wt::WContainerWidget(center);

    observations = wServer->receiveObs();
    Wt::WTimer::singleShot(100, this, &WitreApplication::syncObservations); //Calls function after 100mill

    Wt::WScrollArea* centerScroll = new Wt::WScrollArea();
    centerScroll->setWidget(center);
    centerScroll->setHorizontalScrollBarPolicy(Wt::WScrollArea::ScrollBarAlwaysOff);
    centerScroll->setVerticalScrollBarPolicy(Wt::WScrollArea::ScrollBarAsNeeded);
    //End of Center Code

    Wt::WBorderLayout* layout = new Wt::WBorderLayout(root());
    layout->addWidget(north, Wt::WBorderLayout::North);
    layout->addWidget(east, Wt::WBorderLayout::East);
    layout->addWidget(west, Wt::WBorderLayout::West);
    layout->addWidget(centerScroll, Wt::WBorderLayout::Center);
    layout->addWidget(south, Wt::WBorderLayout::South);

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
        reorder(); // Reorder the panels after ever tick
        Wt::WApplication::instance()->triggerUpdate();
        return;
    }

    std::stringstream xml;
    xml<<observ;
    boost::property_tree::ptree doc;
    read_xml(xml, doc, xml::no_comments|xml::trim_whitespace);
    std::string arg = doc.get<std::string>("Token");

    if(currentPanels.find(name)!=currentPanels.end() && panelsXML[currentPanels[name]].get<std::string>("Token")==arg)
    {

    }
    else
    {
        Wt::WPanel* panel = new Wt::WPanel();
        panelsXML[panel]=doc;
        panel->setCentralWidget(new Wt::WText(observ+" since "+time)); // Addes the most recent observation to webpage
        panel->setObjectName(name); // Names the box by the timeline
        if(!tLineMap[name])
        {
            panel->hide();
        }
        currentPanels[name] = panel;
    }
    observations.pop(); //Pops the most recent observation
    insert(time, currentPanels[name]);
}

void WitreApplication::insert(std::string time, Wt::WPanel* wid)
{
    //const Wt::WAnimation animate(Wt::WAnimation::Fade, Wt::WAnimation::Ease, 10000);
    //wid->setAnimation(animate);
    if(boxPanels[time]==NULL)
    {
        //Groupbox that holds the container for all panels
        std::stringstream xml;
        xml<<"<Tick><LowerTick></LowerTick><UpperTick>"<<time<<"</UpperTick></Tick>";
        boost::property_tree::ptree doc;
        read_xml(xml, doc, xml::no_comments|xml::trim_whitespace);
        Wt::WGroupBox* box = new Wt::WGroupBox(time);
        box->setObjectName(time);
        groupXML[box]=doc;
        //End of Groupbox
        //Container that holds all panels
        Wt::WContainerWidget* temp = new Wt::WContainerWidget(box);
        boxPanels[time]= temp;
        groupPanels[time] = box;
        messages->insertWidget(0, box);
    }
    boxPanels[time]->insertWidget(0, wid);
    Wt::WApplication::instance()->triggerUpdate();
}

void WitreApplication::reorder()
{
    enum placement { Top=0, Bottom=1};
    std::string name = messages->widget(Top)->objectName();
    if(name=="")
    {
        return;
    }
    Wt::WContainerWidget* container[2];
    Wt::WGroupBox* pContainer[2];
    container[Top] = boxPanels[name];
    pContainer[Top] = groupPanels[name];
    //Sets all the current panels to the top of the postings
    std::map<std::string, Wt::WPanel*>::iterator it;
    for(it=currentPanels.begin(); it!=currentPanels.end(); it++)
    {
        container[Top]->insertWidget(Top, it->second);
    }
    //Removes the empty panels and changes the Panels title
    if(messages->count()>1)
    {
        name = messages->widget(Bottom)->objectName();
        container[Bottom] = boxPanels[name];
        pContainer[Bottom] = groupPanels[name];
        boost::property_tree::ptree doc[2];
        std::string lowerArg[2];
        std::string upperArg[2];
        for(int i=0; i<2; i++)
        {
            doc[i] = groupXML[pContainer[i]];
            lowerArg[i] = doc[i].get<std::string>("Tick.LowerTick");
            upperArg[i] = doc[i].get<std::string>("Tick.UpperTick");
        }

        if(container[Bottom]->count()==0)
        {
            doc[Top].put("Tick.LowerTick", ((lowerArg[Bottom]!="")? lowerArg[Bottom]:upperArg[Bottom]));
            groupXML[pContainer[Top]]=doc[Top];
            std::stringstream xml;
            xml <<doc[Top].get<std::string>("Tick.LowerTick")<<" - "<<doc[Top].get<std::string>("Tick.UpperTick");
            pContainer[Top]->setTitle(xml.str());

            //Removing the empty panel
            Wt::WWidget* deleteWidget = messages->widget(Bottom);
            messages->removeWidget(deleteWidget);
            delete deleteWidget;
            groupXML.erase(groupPanels[name]);
            groupPanels.erase(name);
            boxPanels.erase(name);
            //End of removing
        }
        else
        {
            int uBottom = boost::lexical_cast<int>(upperArg[Bottom]);
            int uTop = boost::lexical_cast<int>(upperArg[Top]);
            uTop--;
            doc[Bottom].put("Tick.UpperTick", ((uBottom==uTop)?upperArg[Bottom]:upperArg[Top]));
            doc[Bottom].put("Tick.LowerTick", ((lowerArg[Bottom]!="")? lowerArg[Bottom]:(uBottom==uTop)?"":upperArg[Bottom]));
            groupXML[pContainer[Bottom]]=doc[Bottom];
            std::stringstream xml;
            xml <<doc[Bottom].get<std::string>("Tick.LowerTick")
                <<((doc[Bottom].get<std::string>("Tick.LowerTick")!="")?" - ":"")
                <<doc[Bottom].get<std::string>("Tick.UpperTick");
            pContainer[Bottom]->setTitle(xml.str());

        }
    }

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
        popup->setPosition(input);
        popup->valRange(wServer->getCurrentTick(),wServer->getFinalTick());
        popup->setVisable();
    }
    else{
        Wt::StandardButton incorrect = Wt::WMessageBox::show("Incorrect", "Invalid input for goal"
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
