#ifndef POPUP
#define POPUP
#include <Wt/WCompositeWidget>
#include <Wt/WContainerWidget>
#include <Wt/WIntValidator>
#include <Wt/WText>
#include <Wt/WLineEdit>
#include <Wt/WPushButton>
#include <Wt/WWidget>
#include <Wt/WFlags>
#include <Wt/WSignal>
#include <Wt/WValidationStatus>
#include <Wt/WDatePicker>
#include <Wt/WDateTime>
#include <Wt/WGroupBox>
#include <Wt/WJavaScript>
#include <string>
#include <iostream>
#include <boost/version.hpp>
#include <trex/domain/IntegerDomain.hh>

using namespace TREX::utils;
using namespace TREX::transaction;
using namespace std;

namespace TREX {
  namespace witre {
        class Goalpopup : public Wt::WCompositeWidget {
            private:
            Wt::WContainerWidget* attributes;
            Wt::WCompositeWidget* root;
            Wt::WLineEdit* start;
            Wt::WLineEdit* startEnd;
            Wt::WLineEdit* duration;
            Wt::WLineEdit* durationEnd;
            Wt::WLineEdit* end;
            Wt::WLineEdit* endEnd;
            Wt::WPushButton* ok;
            Wt::WIntValidator* numRange;
            Wt::WIntValidator* middleRange;
            Wt::Signal<transaction::IntegerDomain, transaction::IntegerDomain, transaction::IntegerDomain> finished_;
            Wt::Signal< > cancelled_;
            //Validators of information
            Wt::WValidationStatus* valS;
            Wt::WValidationStatus* valD;
            Wt::WValidationStatus* valE;

            void cancel() { cancelled_.emit(); };
            void okStatus(bool val) { if(valS->valid()&&valD->valid()&&valE->valid()) ok->enable(); else  ok->disable(); };
            void done()
            {
                transaction::IntegerDomain s(string_cast(IntegerDomain::minus_inf,start->text().toUTF8()),string_cast(IntegerDomain::plus_inf,startEnd->text().toUTF8()));
                transaction::IntegerDomain d(string_cast(IntegerDomain::minus_inf,duration->text().toUTF8()),string_cast(IntegerDomain::plus_inf,durationEnd->text().toUTF8()));
                transaction::IntegerDomain e(string_cast(IntegerDomain::minus_inf,end->text().toUTF8()),string_cast(IntegerDomain::plus_inf,endEnd->text().toUTF8()));
                finished_.emit(s,d,e);
            }
            void validateEndinputs()
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
            };

            public:
            Wt::Signal<transaction::IntegerDomain, transaction::IntegerDomain, transaction::IntegerDomain>& finished() { return finished_; };
            Wt::Signal< >& cancelled() { return cancelled_; };
            void setPosition(Wt::WWidget* input) { this->positionAt(input, Wt::Vertical); };
            void clear() { start->setText(""); duration->setText(""); end->setText(""); };
            const void setHide() { this->setHidden(true); };
            const void setVisable() { this->setHidden(false); };
            void valRange(int bottom, int top)
            { numRange->setTop(top); numRange->setBottom(bottom); middleRange->setTop(top);};
            void clearText() { start->setText(""); startEnd->setText("");
                               duration->setText(""); durationEnd->setText("");
                               end->setText(""); endEnd->setText(""); };

            Goalpopup(Wt::WContainerWidget* parent = 0, int current=0, int final=10)
                :Wt::WCompositeWidget(parent), finished_(this)
                {
                    this->setImplementation(attributes = new Wt::WContainerWidget());
                    this->setPopup(true);
                    this->setHidden(true);
                    this->setPositionScheme(Wt::Fixed);
                    this->decorationStyle().setBackgroundColor(Wt::WColor("silver"));
                    this->decorationStyle().setBorder(Wt::WBorder(Wt::WBorder::Solid,Wt::WBorder::Medium,Wt::WColor("black")));

                    stringstream jscript;
                    jscript<<"function checkValues(sender, event)"
                           <<"{ var str = sender.id; str=str.replace(\"end\",\"\");"
                           <<"  var x = new Number(document.getElementById(str).value);"
                           <<"  if(x>Number(sender.value)){ alert(\"Incorrect Value\"); sender.value=x+1; }"
                           <<"}";
                    stringstream jsDisable;
                    jsDisable<<"function disableEnter(sender, event)"
                             <<"{ var x = new Number(sender.value);"
                             <<"  var ele = document.getElementById(\"AttributesPassed\");"
                             <<"  if(x<"<<current<<" || x>"<<final<<") { ele.disabled=true;"
                             <<"  } else { ele.disabled=false; } }";

                    Wt::JSlot* script = new Wt::JSlot(jscript.str(), attributes);
                    Wt::JSlot* disableScript = new Wt::JSlot(jsDisable.str(), attributes);

                    numRange = new Wt::WIntValidator(current,final);
                    middleRange = new Wt::WIntValidator(1,final);

                    this->implementStateless(&Goalpopup::validateEndinputs);

                    //Start attribute
                    attributes->addWidget(new Wt::WText("Start"));
                    start = new Wt::WLineEdit(attributes);
                    start->setId("start");
                    start->setEmptyText("minus_inf");
                    start->setValidator(numRange);
                    start->setMargin(10, Wt::Left);
                    start->blurred().connect(*disableScript);
                    new Wt::WText(" < ", attributes);
                    startEnd = new Wt::WLineEdit(attributes);
                    startEnd->setId("startend");
                    startEnd->setEmptyText("plus_inf");
                    startEnd->setValidator(numRange);
                    startEnd->blurred().connect(*script);
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
                    durationEnd->blurred().connect(*script);
                    new Wt::WBreak(attributes);
                    //End attribute
                    new Wt::WText("End", attributes);
                    end = new Wt::WLineEdit(attributes);
                    end->setId("end");
                    end->setEmptyText("minus_inf");
                    end->setValidator(numRange);
                    end->setMargin(10, Wt::Left);
                    new Wt::WText(" < ", attributes);
                    endEnd = new Wt::WLineEdit(attributes);
                    endEnd->setId("endend");
                    endEnd->setEmptyText("plus_inf");
                    endEnd->setValidator(numRange);
                    endEnd->blurred().connect(*script);
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
                    reject->clicked().connect(ok, &Wt::WWidget::enable);

                };


        };
  }
}

#endif
