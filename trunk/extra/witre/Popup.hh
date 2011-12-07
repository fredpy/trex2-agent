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
#include <string>
#include <boost/version.hpp>
#include <trex/domain/IntegerDomain.hh>

using namespace TREX::utils;
using namespace TREX::transaction;

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

            public:
            Wt::Signal<transaction::IntegerDomain, transaction::IntegerDomain, transaction::IntegerDomain>& finished() { return finished_; };
            Wt::Signal< >& cancelled() { return cancelled_; };
            void setPosition(Wt::WWidget* input) { this->positionAt(input, Wt::Vertical); };
            void clear() { start->setText(""); duration->setText(""); end->setText(""); };
            const void setHide() { this->setHidden(true); return; };
            const void setVisable() { this->setHidden(false); return; };
            void valRange(int bottom, int top)
            { numRange->setTop(top); numRange->setBottom(bottom); middleRange->setTop(top);};
            void clearText() { start->setText(""); startEnd->setText("");
                               duration->setText(""); durationEnd->setText("");
                               end->setText(""); endEnd->setText(""); };

            Goalpopup(Wt::WContainerWidget* parent = 0, int current=0, int final=10)
                :Wt::WCompositeWidget(parent), finished_(this)
                {
                    root = this;
                    this->setImplementation(attributes = new Wt::WContainerWidget());
                    this->setPopup(true);
                    this->setHidden(true);
                    this->setPositionScheme(Wt::Fixed);
                    this->decorationStyle().setBackgroundColor(Wt::WColor("silver"));
                    this->decorationStyle().setBorder(Wt::WBorder(Wt::WBorder::Solid,Wt::WBorder::Medium,Wt::WColor("black")));

                    numRange = new Wt::WIntValidator(current,final);
                    middleRange = new Wt::WIntValidator(1,final);

                    //Start attribute
                    attributes->addWidget(new Wt::WText("Start"));
                    start = new Wt::WLineEdit(attributes);
                    start->setEmptyText("minus_inf");
                    start->setValidator(numRange);
                    start->setMargin(10, Wt::Left);
                    new Wt::WText(" < ", attributes);
                    startEnd = new Wt::WLineEdit(attributes);
                    startEnd->setEmptyText("plus_inf");
                    startEnd->setValidator(numRange);
                    new Wt::WBreak(attributes);
                    //Duration attribute
                    new Wt::WText("Duration", attributes);
                    duration = new Wt::WLineEdit(attributes);
                    duration->setEmptyText("minus_inf");
                    duration->setValidator(middleRange);
                    duration->setMargin(10, Wt::Left);
                    new Wt::WText(" < ", attributes);
                    durationEnd = new Wt::WLineEdit(attributes);
                    durationEnd->setEmptyText("plus_inf");
                    durationEnd->setValidator(numRange);
                    new Wt::WBreak(attributes);
                    //End attribute
                    new Wt::WText("End", attributes);
                    end = new Wt::WLineEdit(attributes);
                    end->setEmptyText("minus_inf");
                    end->setValidator(numRange);
                    end->setMargin(10, Wt::Left);
                    new Wt::WText(" < ", attributes);
                    endEnd = new Wt::WLineEdit(attributes);
                    endEnd->setEmptyText("plus_inf");
                    endEnd->setValidator(numRange);
                    new Wt::WBreak(attributes);
                    //ValidateStatus
                    valS = new Wt::WValidationStatus(start);
                    valS->validated().connect(this, &Goalpopup::okStatus);
                    valD = new Wt::WValidationStatus(duration);
                    valD->validated().connect(this, &Goalpopup::okStatus);
                    valE = new Wt::WValidationStatus(end);
                    valE->validated().connect(this, &Goalpopup::okStatus);


                    //Buttons
                    ok = new Wt::WPushButton("Enter",attributes);
                    ok->clicked().connect(root, &Wt::WWidget::hide);
                    ok->clicked().connect(this, &Goalpopup::done);
                    ok->setMargin(10,Wt::Right);

                    Wt::WPushButton* reject = new Wt::WPushButton("Cancel",attributes);
                    reject->clicked().connect(root, &Wt::WWidget::hide);
                    reject->clicked().connect(this, &Goalpopup::cancel);

                };


        };
  }
}

#endif
