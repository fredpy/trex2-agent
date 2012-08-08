#include "Popup.hh"

#include<Wt/WBreak>
#include<Wt/WCssDecorationStyle>
#include<Wt/WColor>
#include<Wt/WBorder>
#include<Wt/WBorderLayout>
#include<Wt/WInPlaceEdit>
#include<Wt/WPanel>
#include<boost/lexical_cast.hpp>
#include<Wt/WDoubleValidator>

using namespace TREX::witre;
using namespace TREX::utils;
using namespace TREX::transaction;
using namespace std;

Goalpopup::Goalpopup(Wt::WContainerWidget* parent, int current, int final)
                :Wt::WCompositeWidget(parent), finished_(this), delSignal_(this, "delSignal")
{
    this->setImplementation(attributes = new Wt::WContainerWidget());
    this->setPopup(true);
    this->setHidden(true);
    this->setPositionScheme(Wt::Fixed);
    this->decorationStyle().setBackgroundColor(Wt::WColor("silver"));
    this->decorationStyle().setBorder(Wt::WBorder(Wt::WBorder::Solid,Wt::WBorder::Medium,Wt::WColor("black")));

    //JSignal
    delSignal_.connect(this, &Goalpopup::deleteAttribute);
    //Javascript slots
    script = new Wt::JSlot(attributes);
    disableScript = new Wt::JSlot(attributes);

    numRange = new Wt::WIntValidator(1,final);
    middleRange = new Wt::WIntValidator(1,final);

    //Creating Containers
    inputs = new Wt::WContainerWidget(attributes);
    inputs->setPadding(3,Wt::Bottom);
    Wt::WContainerWidget* buttons = new Wt::WContainerWidget(attributes);
    //End of Containers

    std::string names[] = {"Start","Duration","End"};
    for(int i=0; i<3; i++)
    {
        //Creating all the containers
        Wt::WPanel* row = new Wt::WPanel(inputs);
        row->setStyleClass("");
        row->setAttributeValue("style","text-align:right;");
        Wt::WContainerWidget* inCont = new Wt::WContainerWidget();
        row->setCentralWidget(inCont);
        //End of Containers

        inCont->addWidget(new Wt::WText(names[i]));
        Wt::WLineEdit* input = new Wt::WLineEdit(inCont);
        input->setId(names[i]);
        input->setEmptyText("minus_inf");
        input->setValidator((names[i]!="Duration")?numRange:middleRange);
        input->setMargin(10, Wt::Left);
        if(names[i]!="Duration")
            input->keyWentUp().connect(*disableScript);
        new Wt::WText(" < ", inCont);
        Wt::WLineEdit* inputEnd = new Wt::WLineEdit(inCont);
        inputEnd->setId(names[i]+"end");
        inputEnd->setEmptyText("plus_inf");
        inputEnd->setValidator((names[i]!="Duration")?numRange:middleRange);
        if(names[i]!="Duration")
            inputEnd->keyWentUp().connect(*script);
        standards.insert(make_pair(names[i], make_pair(input,inputEnd)));
    }

    //Buttons
    ok = new Wt::WPushButton("Enter",buttons);
    ok->setId("AttributesPassed");
    ok->clicked().connect(this, &Wt::WWidget::hide);
    ok->clicked().connect(this, &Goalpopup::done);
    ok->setMargin(10,Wt::Right);

    Wt::WPushButton* reject = new Wt::WPushButton("Cancel",buttons);
    reject->clicked().connect(this, &Wt::WWidget::hide);
    reject->clicked().connect(this, &Goalpopup::cancel);

    Wt::WPushButton* add = new Wt::WPushButton("add",buttons);
    add->clicked().connect(this, &Goalpopup::addAttribute);
    add->setMargin(10, Wt::Left);

}

void Goalpopup::addAttribute()
{
    //Creating containers
    Wt::WPanel* row = new Wt::WPanel(inputs);
    row->setObjectName(row->id());
    row->setStyleClass("");
    row->setAttributeValue("style","text-align:right;");
    Wt::WContainerWidget* inCont = new Wt::WContainerWidget();
    row->setCentralWidget(inCont);
    //End of Containers

    Wt::WInPlaceEdit* predName = new Wt::WInPlaceEdit("Name?", inCont);
    predName->lineEdit()->setAttributeValue("size","7");
    predName->cancelButton()->setText("Delete");
    Wt::WDoubleValidator* valid = new Wt::WDoubleValidator();
    Wt::WLineEdit* input = new Wt::WLineEdit(inCont);
    input->setValidator(valid);
    input->setEmptyText("plus_inf");
    input->setMargin(5, Wt::Left);
    additions.push_back(make_pair(predName, input));
    stringstream code;
    code<<"function(){ Wt.emit("<<this->jsRef()<<",'delSignal',"
        <<"'"<<row->objectName()<<"','"<<predName->id()<<"','"<<input->id()<<"');}";
    predName->cancelButton()->clicked().connect(code.str());
}

void Goalpopup::deleteAttribute(string container, string name, string input)
{
    std::list<std::pair<Wt::WInPlaceEdit*,Wt::WLineEdit*> >::iterator it;
    for(it = additions.begin(); it!=additions.end(); it++)
    {
        if(it->first->id()==name)
        {
            additions.erase(it);
            break;
        }
    }
    Wt::WWidget* del = inputs->find(container);
    inputs->removeWidget(del);
    delete del;
}

void Goalpopup::clearText()
{
    typedef std::map<string, pair<Wt::WLineEdit*,Wt::WLineEdit*> >::iterator pairMap;
    for(pairMap it = standards.begin(); it!= standards.end(); it++)
    {
        it->second.first->setText("");
        it->second.second->setText("");
    }
}

void Goalpopup::valRange(int bottom, int top)
{
    numRange->setTop(top);
    numRange->setBottom(bottom);
    middleRange->setTop(top);
    //As of right now I will repeat js code here to update values
    stringstream jsDisable;
    jsDisable<<"function disableEnter(sender, event)"
             <<"{ var x = sender.value;"
             <<"  var ele = document.getElementById(\"AttributesPassed\");"
             <<"  if((Number(x)<"<<bottom<<" || Number(x)>"<<top<<") && x!=\"\" )"
             <<"  { ele.disabled=true; sender.style.backgroundColor=\"Crimson\";"
             <<"  } else { ele.disabled=false; sender.style.backgroundColor=\"white\"; } }";
    disableScript->setJavaScript(jsDisable.str());
    stringstream jscript;
    jscript<<"function checkValues(sender, event)"
           <<"{ var str = sender.id; str=str.replace(\"end\",\"\");" //Getting the id of sender then getting the id of left input
           <<"  var Lvalue = document.getElementById(str).value;" //Getting value of left input
           <<"  var Rvalue = sender.value;"
           <<"  var ele = document.getElementById(\"AttributesPassed\");"
           <<"  if((Number(Rvalue)<"<<bottom<<" || Number(Rvalue)>"<<top<<" || Number(Lvalue)>Number(Rvalue)) && Rvalue!=\"\")"
           <<"  { ele.disabled=true; sender.style.backgroundColor=\"Crimson\";"
           <<"  } else { ele.disabled=false; sender.style.backgroundColor=\"white\"; } }";
    script->setJavaScript(jscript.str());
}

void Goalpopup::done()
{
    std::map<string, transaction::IntegerDomain> values;
    std::map<string, transaction::FloatDomain> etcValues;
    typedef std::map<string, pair<Wt::WLineEdit*,Wt::WLineEdit*> >::iterator pairMap;
    for(pairMap it = standards.begin(); it!= standards.end(); it++)
    {
        Wt::WLineEdit* first = it->second.first;
        Wt::WLineEdit* second = it->second.second;
        string name = it->first;
        values.insert(make_pair(name,transaction::IntegerDomain(string_cast(IntegerDomain::minus_inf,first->text().toUTF8()),
                                                                   string_cast(IntegerDomain::plus_inf,second->text().toUTF8()))));
    }
    typedef std::list<std::pair<Wt::WInPlaceEdit*,Wt::WLineEdit*> >::iterator additionMap;
    for(additionMap it = additions.begin(); it!=additions.end(); it++)
    {
        string name = it->first->text().toUTF8();
        Wt::WLineEdit* first = it->second;
        try
        {
            etcValues.insert(make_pair(name,transaction::FloatDomain(boost::lexical_cast<double>(first->text().toUTF8()))));
        }catch(boost::bad_lexical_cast bad)
        {
            stringstream msg;
            msg<<"alert('Exception caught with "<<name<<": could not convert input to correct format');";
            this->doJavaScript(msg.str());
        }
    }
    finished_.emit(values, etcValues);
}
