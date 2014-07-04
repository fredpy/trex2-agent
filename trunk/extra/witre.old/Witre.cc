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
#include "WitreGraph.hh"

#include <trex/utils/TREXversion.hh>
#include <trex/utils/Plugin.hh>

#include <Wt/WFileResource>
#include <Wt/WStandardItem>

using namespace TREX::utils;
using namespace TREX::witre;

namespace xml = boost::property_tree::xml_parser;

namespace {

  singleton::use<log_manager> s_log;

}

TREX::witre::WitreApplication *TREX::witre::createWitre(Wt::WEnvironment const &e, WitreServer* Server) {
      WitreApplication *app = new WitreApplication(e,Server);
      // hard coded for now

      bool found;
      std::string file = s_log->use("witre_loc.xml", found).string();

      if( found ) {
            file.erase(file.end()-4, file.end());
            s_log->syslog("witre", info)<<"set locale to "<<file;
            app->messageResourceBundle().use(file);
      } else {
	s_log->syslog("witre", info)<<"Did not find locale "<<file;
      }

      return app;
}


WitreApplication::WitreApplication(Wt::WEnvironment const &env, WitreServer* Server)
  :Wt::WApplication(env), logModel(NULL) {
    wServer = Server;
    wServer->connect(this, boost::bind(&WitreApplication::post, this));
    enableUpdates(true);

    setTitle("Witre - trex "+TREX::version::str());

    // Javascript code for highlighting the dependencies of the timelines
    std::stringstream highlight;
    highlight<<"function highlight(sender, color)"
             <<"{ var child = sender.parentNode.childNodes; var array = sender.getAttribute(\"dependencies\").split(\"&&\");"
             <<" for(var i=0; i<child.length; i++)"
             <<" { for(name in array) { if(array[name]==child[i].getAttribute(\"name\"))"
             <<" { child[i].lastChild.style.backgroundColor = (color ? \"#E0FFFF\":\"white\"); }}}}";
    this->declareJavaScriptFunction("highlight", highlight.str());

    //setCssTheme("polished");
    //Creating the containers for layout
    Wt::WContainerWidget* north = new Wt::WContainerWidget();
    Wt::WContainerWidget* center = new Wt::WContainerWidget();
    center->setAttributeValue("name", "center");
    Wt::WContainerWidget* east = new Wt::WContainerWidget();
    east->setContentAlignment(Wt::AlignTop | Wt::AlignRight);
    Wt::WContainerWidget* west = new Wt::WContainerWidget();
    Wt::WContainerWidget* south = new Wt::WContainerWidget();
    south->setContentAlignment(Wt::AlignBottom | Wt::AlignCenter);
    //end of Containers

    //Creating the layout
    Wt::WBoxLayout* pagelayout = new Wt::WBoxLayout(Wt::WBoxLayout::LeftToRight, root());
    webpage = new Wt::WStackedWidget(); //Webpage is the stackwidget that keeps one webpage visiable
    pagelayout->addWidget(webpage);
    Wt::WContainerWidget* basicpage = new Wt::WContainerWidget(webpage); //Basicpage is the basic layout of the website
    //End of layout

    //Start of North frame code
    new Wt::WText("Current Tick Value: ",north);
    tickNum = new Wt::WText("0", north);
    tickNum->setMargin(5,Wt::Right);
    new Wt::WBreak(north);

    tLines = new Wt::WGroupBox("Available Timelines", north);
    for(int i = 0; i<wServer->extTimelinesSize(); i++)
    {
        std::string name = wServer->extTimelinesName(i);
        tLineMap[name]=true;
        Wt::WPushButton* temp = new Wt::WPushButton(name, tLines);
        temp->setMargin(10,Wt::Right);
        temp->setObjectName(name);
        temp->clicked().connect(this, &WitreApplication::timeLineChange);
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
    new Wt::WBreak(east);
    new Wt::WBreak(east);
    //links in the east box
    Wt::WGroupBox* links = new Wt::WGroupBox("Links",east);
    links->setList(true);
    /* Wt::WAnchor* lgraph = */ new Wt::WAnchor(Wt::WLink(Wt::WLink::InternalPath, "/graph"), "Graph", links);
    new Wt::WBreak(links);
    /* Wt::WAnchor* ltrex = */ new Wt::WAnchor(Wt::WLink(Wt::WLink::InternalPath, "/log/TREX.log"), "Trex log", links);
    new Wt::WBreak(links);
    new Wt::WBreak(links);
    Wt::WSignalMapper<Wt::WLineEdit*> *logmap = new Wt::WSignalMapper<Wt::WLineEdit*>(this);
    logmap->mapped().connect(this, &WitreApplication::searchLog);
    Wt::WLineEdit* log = new Wt::WLineEdit(links);
    log->setEmptyText("Search");
    logmap->mapConnect(log->enterPressed(), log);
    //End of links
    //End of East Code

    //Start of West code
    popup = new Goalpopup(west); //Add straight to page to make the formating work
    popup->finished().connect(this, &WitreApplication::clientPostGoal);
    popup->finished().connect(enter, &Wt::WPushButton::enable);
    popup->cancelled().connect(enter, &Wt::WPushButton::enable);
    //End of West Code


    //Start of South Code
    
    // log table model
    logModel = new Wt::WStandardItemModel(south);
    logModel->insertColumns(logModel->columnCount(), 4);
    // headers
    logModel->setHeaderData(0, Wt::WString("Tick"));
    logModel->setHeaderData(1, Wt::WString("Source"));
    logModel->setHeaderData(2, Wt::WString("Kind"));
    logModel->setHeaderData(3, Wt::WString("Message"));
    
    logTable = new Wt::WTableView(south);
    logTable->setSortingEnabled(false);
    logTable->setModel(logModel);
    logTable->setRowHeight(22);
    logTable->setColumnWidth(0, 50);
    logTable->setColumnWidth(3, Wt::WLength(75, Wt::WLength::Percentage));
    logTable->resize(Wt::WLength(100, Wt::WLength::Percentage), 100);
    
    
    
#if 0    
    timeLineSlider = new Wt::WSlider(Wt::Horizontal, south);
    //timeLineSlider->setTickPosition(Wt::WSlider::TicksBothSides);
    timeLineSlider->setTickInterval(boost::chrono::duration_cast< boost::chrono::duration<double> >(wServer->tickDuration()).count());
    timeLineSlider->setRange(0, wServer->getFinalTick());
    timeLineSlider->resize(400, 50);
    timeLineSlider->sliderMoved().connect(this, &WitreApplication::sliderChanged);
    timeLineSlider->sliderMoved().connect(this, &WitreApplication::sliderText);

    sliderTime = new Wt::WText(south);
#endif
    //End of South Code

    //Start of Center Code
    future = new Wt::WGroupBox("Future",center);

    messages = new Wt::WContainerWidget(center);

    observations = wServer->receiveObs();
    Wt::WTimer::singleShot(100, this, &WitreApplication::syncObservations); //Calls function after 100mill

    Wt::WScrollArea* centerScroll = new Wt::WScrollArea();
    centerScroll->setWidget(center);
    centerScroll->setHorizontalScrollBarPolicy(Wt::WScrollArea::ScrollBarAlwaysOff);
    centerScroll->setVerticalScrollBarPolicy(Wt::WScrollArea::ScrollBarAsNeeded);
    //End of Center Code

    //Adding containers to layout
    Wt::WBorderLayout* layout = new Wt::WBorderLayout(basicpage);
    layout->addWidget(north, Wt::WBorderLayout::North);
    layout->addWidget(east, Wt::WBorderLayout::East);
    layout->addWidget(west, Wt::WBorderLayout::West);
    layout->addWidget(centerScroll, Wt::WBorderLayout::Center);
    layout->addWidget(south, Wt::WBorderLayout::South);
    //end

    WApplication::instance()->internalPathChanged().connect(this, &WitreApplication::urlPage);
    urlPage(WApplication::instance()->internalPath());

}

WitreApplication::~WitreApplication()
{
    wServer->disconnect(this);
    enableUpdates(false);
}

void WitreApplication::urlPage(const std::string& path)
{
    Wt::WWidget* temp = webpage->currentWidget();
    if(path.empty())
    {
        webpage->setCurrentIndex(0);
        WApplication::instance()->setInternalPath("/default");
    }
    else if(internalPathMatches("/graph"))
    {
        WitreGraphContainer* image = new WitreGraphContainer(webpage, wServer->get_graph());
        webpage->setCurrentWidget(image->getWidget());
    }
    else if(internalPathMatches("/log") && !internalPathNextPart("/log/").empty())
    {
        bool found;
        string path = internalPathNextPart("/log/");
        std::string file = s_log->locate(s_log->log_file(path).string(), found).string();
        if(found)
        {
          // file = s_log->locate(file, found).string();
            Wt::WFileResource* log = new Wt::WFileResource(file, this);
            std::stringstream output;
            log->write(output);
            std::stringstream webout;
            string line;
            while(getline(output,line))
            {
                webout<<line<<"<br />";
            }
            Wt::WContainerWidget* holder = new Wt::WContainerWidget(webpage);
            holder->setOverflow(Wt::WContainerWidget::OverflowAuto, Wt::Vertical);
            /* Wt::WText* text = */new Wt::WText(webout.str(), Wt::XHTMLText, holder);
            webpage->setCurrentWidget(holder);
        }
        else
        {
            this->doJavaScript("alert('Did not find "+path+"')");
            WApplication::instance()->setInternalPath("/default");
        }
    }
    else
    {
        webpage->setCurrentIndex(0);
        WApplication::instance()->setInternalPath("/default");
    }
    //Checking if the old webpage needs to be deleted
    if(webpage->currentIndex()==0 && temp!=webpage->currentWidget())
    {
        webpage->removeWidget(temp);
        delete temp;
    }
}

void WitreApplication::searchLog(Wt::WLineEdit* sender)
{
    WApplication::instance()->setInternalPath("/log/"+sender->text().toUTF8(),true);
    sender->setText("");
}

void WitreApplication::post()
{
    std::string observ = observations.front();
    //Getting the observation data into a ptree
    std::stringstream xml;
    xml<<observ;
    boost::property_tree::ptree doc;
    read_xml(xml, doc, xml::no_comments|xml::trim_whitespace);
    //End of getting data
    std::string name = doc.get<std::string>("Token.<xmlattr>.on");
    std::string time = doc.get<std::string>("Token.<xmlattr>.tick");

    if(name=="Tick")
    {
        updateTick(observ);
        observations.pop();
      //timeLineSlider->setValue(timeLineSlider->value()+1);
      //sliderText(timeLineSlider->value());
        reorder(time); // Reorder the panels after ever tick
        WApplication::instance()->triggerUpdate();
        return;
    }

    if(groupPanels[time]==NULL)
    {
        std::stringstream xml;
        xml<<"<Tick><LowerTick></LowerTick><UpperTick>"<<time<<"</UpperTick></Tick>";
        boost::property_tree::ptree doc;
        read_xml(xml, doc, xml::no_comments|xml::trim_whitespace);
        postingDoc.add_child("Group"+time, doc);
        //Container that holds all the panles
        Wt::WGroupBox* box = new Wt::WGroupBox(time);
        box->setObjectName(time);
        box->setAttributeValue("time", time);
        //box->setAttributeValue("onchange","alert(\"Did it\")");
        groupPanels[time] = box;
        messages->insertWidget(0, box);
    }

    std::string level = doc.get<std::string>("Token.<xmlattr>.level");
    postingDoc.add_child("Tick"+time+".Panel", doc);
    Wt::WPanel* panel = new Wt::WPanel();
    allPanels[name].push_front(panel);
    panel->setCentralWidget(new Wt::WText(observ+" since "+time)); // Addes the most recent observation to webpage
    panel->setObjectName(name); // Names the box by the timeline
    panel->setAttributeValue("name", name);
    panel->setAttributeValue("level", level);
    panel->setAttributeValue("since", time);
    panel->setAttributeValue("dependencies", wServer->getDependencies(name));
    panel->setAttributeValue("onmouseover", javaScriptClass()+".highlight("+panel->jsRef()+", true)");
    panel->setAttributeValue("onmouseout", javaScriptClass()+".highlight("+panel->jsRef()+", false)");
    if(!tLineMap[name])
    {
        panel->hide();
    }
    if(boost::lexical_cast<int>(level)>-1)
    {
        currentPanels[name] = panel;
    }
    observations.pop(); //Pops the most recent observation
    insert(time, panel);
    needsUpdated= true;
  for(;;) {
    entry next;
    {
      log_queue::scoped_lock lock(m_logs);
      if( m_logs->empty() )
        return;
      next = m_logs->front();
      m_logs->pop();
    }
    logModel->insertRow(0, next);
  }
}

void WitreApplication::insert(std::string time, Wt::WPanel* wid)
{
    //const Wt::WAnimation animate(Wt::WAnimation::Fade, Wt::WAnimation::Ease, 10000);
    //wid->setAnimation(animate);
    groupPanels[time]->insertWidget(0, wid);
    Wt::WApplication::instance()->triggerUpdate();
}

void WitreApplication::reorder(std::string time)
{
    enum placement { Top=0, Bottom=1};
    std::string name[2];
    name[Top] = messages->widget(Top)->objectName();
    if(name[Top]=="") return;
    //Arrays for data
    Wt::WGroupBox* pContainer[2];
    boost::property_tree::ptree doc[2];
    std::string lowerArg[2];
    std::string upperArg[2];
    //End of Arrays
    pContainer[Top] = groupPanels[name[Top]];

    if(needsUpdated)
    {
        //Sets all the current panels to the top of the postings
        std::map<std::string, Wt::WPanel*>::iterator it;
        for(it=currentPanels.begin(); it!=currentPanels.end(); it++)
        {
            pContainer[Top]->insertWidget(Top, it->second);
        }
        std::stringstream javascript;
        javascript<<"sender = document.getElementById(\""<<pContainer[Top]->id()<<"\");"
                  <<"var child = sender.childNodes;"
                  <<"for(var i=1; i<child.length; i++){ var temp = child[i];"
                  <<"for(var y=i+1; y<child.length; y++){ var temp2 = child[y];"
                  <<"var level = temp.getAttribute(\"level\"); var level2 =temp2.getAttribute(\"level\");"
                  <<"if(level>level2){sender.insertBefore(temp2, temp); temp=temp2; y--;}"
                  <<"else if(level==level2){ if(temp.getAttribute(\"since\")<temp2.getAttribute(\"since\"))"
                  <<"{sender.insertBefore(temp2, temp); temp=temp2; y--; }}}}";
        pContainer[Top]->doJavaScript(javascript.str());
    }
    //Removes the empty panels and changes the Panels title
    if(messages->count()>1 && needsUpdated)
    {
        name[Bottom] = messages->widget(Bottom)->objectName();
        pContainer[Bottom] = groupPanels[name[Bottom]];
        for(int i=0; i<2; i++)
        {
            doc[i] = postingDoc.get_child("Group"+name[i]);
            lowerArg[i] = doc[i].get<std::string>("Tick.LowerTick");
            upperArg[i] = doc[i].get<std::string>("Tick.UpperTick");
        }

        if(pContainer[Bottom]->count()==0)
        {
            doc[Top].put("Tick.LowerTick", ((lowerArg[Bottom]!="")? lowerArg[Bottom]:upperArg[Bottom]));
            postingDoc.put_child("Group"+name[Top], doc[Top]);
            setXMLTitle(pContainer[Top]);

            //Removing the empty panel
            Wt::WWidget* deleteWidget = messages->widget(Bottom);
            messages->removeWidget(deleteWidget);
            delete deleteWidget;
            groupPanels.erase(name[Bottom]);
            //End of removing
        }
        else
        {
            int uBottom = boost::lexical_cast<int>(upperArg[Bottom]);
            int uTop = boost::lexical_cast<int>(upperArg[Top]);
            uTop--;
            doc[Bottom].put("Tick.UpperTick", ((uBottom==uTop)?upperArg[Bottom]:upperArg[Top]));
            doc[Bottom].put("Tick.LowerTick", ((lowerArg[Bottom]!="")? lowerArg[Bottom]:(uBottom==uTop)?"":upperArg[Bottom]));
            postingDoc.put_child("Group"+name[Bottom], doc[Bottom]);
            setXMLTitle(pContainer[Bottom]);
        }
    }
    else
    {
        doc[Top] = postingDoc.get_child("Group"+name[Top]);
        lowerArg[Top] = doc[Top].get<std::string>("Tick.LowerTick");
        upperArg[Top] = doc[Top].get<std::string>("Tick.UpperTick");
        doc[Top].put("Tick.LowerTick", ((lowerArg[Top]=="")? (upperArg[Top]==time)?"":upperArg[Top]:lowerArg[Top]));
        doc[Top].put("Tick.UpperTick", time);
        postingDoc.put_child("Group"+name[Top], doc[Top]);
        setXMLTitle(pContainer[Top]);
    }
    needsUpdated=false;

}

void WitreApplication::setXMLTitle(Wt::WGroupBox *container)
{
    std::string name = container->objectName();
    boost::property_tree::ptree doc = postingDoc.get_child("Group"+name);
    std::stringstream xml;
    xml <<doc.get<std::string>("Tick.LowerTick")
        <<((doc.get<std::string>("Tick.LowerTick")=="")?"":" - ")
        <<doc.get<std::string>("Tick.UpperTick");
    container->setTitle(xml.str());

}

void WitreApplication::addObs(std::string temp)
{
    observations.push(temp);
    return;
}

void WitreApplication::addLog(boost::optional<unsigned long long> date,
                              std::string const &who, std::string const &what,
                              std::string const &msg) {
  std::vector<Wt::WStandardItem *> entry(4);
  
  if( date ) {
    std::ostringstream str_date;
    str_date<<(*date);
    entry[0] = new Wt::WStandardItem(Wt::WString(str_date.str()));
  } else
    entry[0] = new Wt::WStandardItem;
  entry[1] = new Wt::WStandardItem(Wt::WString(who));
  entry[2] = new Wt::WStandardItem(Wt::WString(what));
  entry[3] = new Wt::WStandardItem(Wt::WString(msg));
  
  {
    log_queue::scoped_lock lock(m_logs);
    m_logs->push(entry);
  }
}


void WitreApplication::timeLineChange()
{
    Wt::WObject* timeLine = sender();
    std::string tName = timeLine->objectName();
    std::stringstream javascript;
    javascript<<"document.getElementById(\""<<timeLine->id()<<"\").style.backgroundColor ="
              <<"("<<tLineMap[tName]<<")?'white':'';";
    this->doJavaScript(javascript.str());
    std::map<std::string, Wt::WGroupBox*>::iterator it;
    for(it=groupPanels.begin(); it!=groupPanels.end(); it++)
    {
        Wt::WGroupBox* container = (*it).second;
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
  
  newPlanToken(wServer->plan());
  
}

void WitreApplication::attributePopup()
{
    std::string object = menu->currentText().toUTF8();
    std::string predicate = input->text().toUTF8();
    bool found;
    std::string file = s_log->locate(predicate, found).string();
    if(found)
    {
        stringstream err;
        s_log->use(file,found);
        try
        {
            stringstream msg;
            boost::property_tree::ptree config;
            read_xml(file,config,xml::no_comments|xml::trim_whitespace);
            if(!config.empty())
            {
                int count=0;
                if( config.size()==1 && !TREX::utils::is_tag(config.front(), "Goal") )
                    config = config.front().second;
                boost::property_tree::ptree::assoc_iterator i, last;
                boost::tie(i, last) = config.equal_range("Goal");
                msg<<"alert(\"";
                for(; last!=i; ++i, count++)
                {
                  TREX::transaction::token_id
                    g = wServer->clientGoalPost(*i);
                  if( g )
                    msg<<"Goal: "<<*g<<"\\n";
                }
                msg<<"\");";
                doJavaScript(msg.str());
            } else {
            doJavaScript("alert('File ("+file+") is empty!');");
            }
        } catch( std::exception const &e ) {
            err<<"alert('Exception: ("<<e.what()<<")');";
            doJavaScript(err.str());
        } catch(...) {
            doJavaScript("alert('Unknown error');");
        }
        input->setText("");
        enter->enable();
    }
    else if(!object.empty() && !predicate.empty())
    {
        popup->setPosition(input);
        popup->valRange(wServer->current_tick(),wServer->final_tick());
        popup->setVisable();
    }
    else {
        /* Wt::StandardButton incorrect = */Wt::WMessageBox::show("Incorrect", "Invalid input for goal"
                                                             , Wt::Ok);
        enter->enable();
    }
}

void WitreApplication::clientPostGoal(std::map<string, transaction::int_domain> standards,
                                        std::map<string,transaction::float_domain> attributes)
{
    std::string object = menu->currentText().toUTF8();
    std::string predicate = input->text().toUTF8();
    if(!object.empty() && !predicate.empty())
    {
        TREX::transaction::token goal = wServer->getGoal(object,predicate);
        goal.restrict_start(standards.find("Start")->second);
        goal.restrict_duration(standards.find("Duration")->second);
        goal.restrict_end(standards.find("End")->second);

        std::map<string,transaction::float_domain>::iterator etc;
        for(etc = attributes.begin(); etc!=attributes.end(); etc++)
        {
            transaction::var temp(etc->first, etc->second);
            goal.restrict_attribute(temp);
        }
        TREX::transaction::token_id goalid = wServer->clientGoalPost(goal);
        if(goalid!=NULL)
        {
            std::stringstream oss;
            oss<<"<Token on=\""<<object<<"\" tick=\""<<wServer->current_tick()<<"\" level=\"-1\" >"
               <<"On Timeline <on><b>"<<goalid->object()<<"</b></on>, placed goal "
               <<"<pred><b>"<<goalid->predicate()<<"</b></pred> "
               <<"at the time: "<<wServer->getTime_t()<<"<br />"
               <<"Starting: "<<standards.find("Start")->second<<"  Duration: "<<standards.find("Duration")->second
               <<"  End: "<<standards.find("End")->second<<"<br />";
               for(etc = attributes.begin(); etc!=attributes.end(); etc++)
               {
                   oss<<etc->first<<": "<<etc->second<<"  ";
               }
            oss<<"</Token>";
            observations.push(oss.str());
            post();
        }

    }
    popup->clearText();

}

void WitreApplication::sliderChanged(int value)
{
    std::map<std::string, Wt::WGroupBox*>::iterator it;
    for(it = groupPanels.begin(); it!= groupPanels.end(); it++ )
    {
        if(boost::lexical_cast<int>((*it).first) > value)
        {
            (*it).second->hide();
        }
        else
        {
            (*it).second->show();
        }
    }
}

void WitreApplication::sliderText(int value)
{
  //sliderTime->setText(boost::lexical_cast<std::string>(value));
    Wt::WApplication::instance()->triggerUpdate();
}

void WitreApplication::addTimeline(std::string name)
{
    tLineMap[name]=true;
    Wt::WPushButton* temp = new Wt::WPushButton(name, tLines);
    temp->setMargin(10,Wt::Right);
    temp->setObjectName(name);
    temp->clicked().connect(this, &WitreApplication::timeLineChange);
    Wt::WDialog* popup = new Wt::WDialog("New Timeline: <b>"+name+"</b>");
    popup->setModal(false);
    popup->show();
    std::stringstream javascript;
    javascript<<"setTimeout(\""<<popup->jsRef()<<".style.top = \'5px\';"
              <<popup->jsRef()<<".id=\'header\';\",100)";
    std::stringstream javascript2;
    javascript2<<"setTimeout(\"var n=document.getElementById(\'header\'); n.parentNode.removeChild(n)\",30000)";
    this->doJavaScript(javascript.str());
    this->doJavaScript(javascript2.str());
    //Updates the combo menu
    addMenuItems();
}

void WitreApplication::addMenuItems()
{
    menu->clear();
    for(int i = 0; i<wServer->extTimelinesSize(); i++)
    {
        std::string name = wServer->extTimelinesName(i);
        if(wServer->acceptsGoal(wServer->extTimelinesName(i)))
        {
            menu->addItem(name);
        }
    }
}

void WitreApplication::newPlanToken(const WitreServer::timed_goal& plan)
{
    future->clear();
    WitreServer::timed_goal::const_iterator t;
    for(t = plan.begin(); t!=plan.end(); ++t)
    {
        const token_id& goal = (*t);
        std::ostringstream planStr;
        planStr <<"<Token timeline=\'"<<goal->object()<<"\' pred=\'"<<goal->predicate()
             <<"\' value=\'"<<goal<<"\'>"
             <<"["<<goal->object()<<"."<<goal->predicate()<<"] plan: "
             <<"Start = "<<goal->start()<<" Duration = "<<goal->duration()<<" End = "<<goal->end()
             <<"</Token>";
        ostringstream stime;
        stime<< wServer->current_tick();
        string time = stime.str();
        string name = goal->object().str();
        Wt::WPanel* panel = new Wt::WPanel();
        panel->setCentralWidget(new Wt::WText(planStr.str()));
        panel->setObjectName(name);
        panel->setAttributeValue("name", name);
        panel->setAttributeValue("tick", time);
        future->insertWidget(0,panel);
    }
}
