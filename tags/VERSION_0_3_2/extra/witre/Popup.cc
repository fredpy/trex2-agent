#include "Popup.hh"

#include<Wt/WBreak>
#include<Wt/WCssDecorationStyle>
#include<Wt/WColor>
#include<Wt/WBorder>

using namespace TREX::witre;
using namespace TREX::utils;
using namespace TREX::transaction;
using namespace std;

Goalpopup::Goalpopup(Wt::WContainerWidget* parent, int current, int final)
                :Wt::WCompositeWidget(parent), finished_(this)
{
    this->setImplementation(attributes = new Wt::WContainerWidget());
    this->setPopup(true);
    this->setHidden(true);
    this->setPositionScheme(Wt::Fixed);
    this->decorationStyle().setBackgroundColor(Wt::WColor("silver"));
    this->decorationStyle().setBorder(Wt::WBorder(Wt::WBorder::Solid,Wt::WBorder::Medium,Wt::WColor("black")));

    //Javascript slots
    script = new Wt::JSlot(attributes);
    disableScript = new Wt::JSlot(attributes);

    numRange = new Wt::WIntValidator(1,final);
    middleRange = new Wt::WIntValidator(1,final);

    this->implementStateless(&Goalpopup::validateEndinputs);

    //Start attribute
    attributes->addWidget(new Wt::WText("Start"));
    start = new Wt::WLineEdit(attributes);
    start->setId("start");
    start->setEmptyText("minus_inf");
    start->setValidator(numRange);
    start->setMargin(10, Wt::Left);
    start->keyWentUp().connect(*disableScript);
    new Wt::WText(" < ", attributes);
    startEnd = new Wt::WLineEdit(attributes);
    startEnd->setId("startend");
    startEnd->setEmptyText("plus_inf");
    startEnd->setValidator(numRange);
    startEnd->keyWentUp().connect(*script);
    new Wt::WBreak(attributes);
    //Duration attribute
    new Wt::WText("Duration", attributes);
    duration = new Wt::WLineEdit(attributes);
    duration->setId("duration");
    duration->setEmptyText("minus_inf");
    duration->setValidator(middleRange);
    duration->setMargin(10, Wt::Left);
    new Wt::WText(" < ", attributes);
    durationEnd = new Wt::WLineEdit(attributes);
    durationEnd->setId("durationend");
    durationEnd->setEmptyText("plus_inf");
    durationEnd->setValidator(middleRange);
    durationEnd->keyWentUp().connect(*script);
    new Wt::WBreak(attributes);
    //End attribute
    new Wt::WText("End", attributes);
    end = new Wt::WLineEdit(attributes);
    end->setId("end");
    end->setEmptyText("minus_inf");
    end->setValidator(numRange);
    end->setMargin(10, Wt::Left);
    end->keyWentUp().connect(*disableScript);
    new Wt::WText(" < ", attributes);
    endEnd = new Wt::WLineEdit(attributes);
    endEnd->setId("endend");
    endEnd->setEmptyText("plus_inf");
    endEnd->setValidator(numRange);
    endEnd->keyWentUp().connect(*script);
    new Wt::WBreak(attributes);
    //ValidateStatus
    //valS = new Wt::WValidationStatus(start);
    //valS->validated().connect(this, &Goalpopup::okStatus);
    //valD = new Wt::WValidationStatus(duration);
    //valD->validated().connect(this, &Goalpopup::okStatus);
    //valE = new Wt::WValidationStatus(end);
    //valE->validated().connect(this, &Goalpopup::okStatus);
    //startEnd->changed().connect(this, &Goalpopup::validateEndinputs);
    //durationEnd->changed().connect(this, &Goalpopup::validateEndinputs);
    //endEnd->changed().connect(this, &Goalpopup::validateEndinputs);

    //Buttons
    ok = new Wt::WPushButton("Enter",attributes);
    ok->setId("AttributesPassed");
    ok->clicked().connect(this, &Wt::WWidget::hide);
    ok->clicked().connect(this, &Goalpopup::done);
    ok->setMargin(10,Wt::Right);

    Wt::WPushButton* reject = new Wt::WPushButton("Cancel",attributes);
    reject->clicked().connect(this, &Wt::WWidget::hide);
    reject->clicked().connect(this, &Goalpopup::cancel);

}

void Goalpopup::clearText()
{
    start->setText(""); startEnd->setText("");
    duration->setText(""); durationEnd->setText("");
    end->setText(""); endEnd->setText("");
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

void Goalpopup::validateEndinputs()
{
    if((start->text().toUTF8()<=startEnd->text().toUTF8()) || (duration->text().toUTF8()<=durationEnd->text().toUTF8())
        || (end->text().toUTF8()<=endEnd->text().toUTF8()) )
    {
        ok->disable();
    }
    else
    {
        ok->enable();
    }
}

void Goalpopup::done()
{
    transaction::IntegerDomain s(string_cast(IntegerDomain::minus_inf,start->text().toUTF8()),string_cast(IntegerDomain::plus_inf,startEnd->text().toUTF8()));
    transaction::IntegerDomain d(string_cast(IntegerDomain::minus_inf,duration->text().toUTF8()),string_cast(IntegerDomain::plus_inf,durationEnd->text().toUTF8()));
    transaction::IntegerDomain e(string_cast(IntegerDomain::minus_inf,end->text().toUTF8()),string_cast(IntegerDomain::plus_inf,endEnd->text().toUTF8()));
    finished_.emit(s,d,e);
}
