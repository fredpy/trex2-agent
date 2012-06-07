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
#include <Wt/WTemplate>
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
            Wt::JSlot* script;
            Wt::JSlot* disableScript;
            //Validators of information
            Wt::WValidationStatus* valS;
            Wt::WValidationStatus* valD;
            Wt::WValidationStatus* valE;

            void cancel() { cancelled_.emit(); };
            void okStatus(bool val) { if(valS->valid()&&valD->valid()&&valE->valid()) ok->enable(); else  ok->disable(); };
            void done();
            void validateEndinputs();

            public:
            Wt::Signal<transaction::IntegerDomain, transaction::IntegerDomain, transaction::IntegerDomain>& finished() { return finished_; };
            Wt::Signal< >& cancelled() { return cancelled_; };
            void setPosition(Wt::WWidget* input) { this->positionAt(input, Wt::Vertical); };
            const void setHide() { this->setHidden(true); };
            const void setVisable() { this->setHidden(false); };
            void valRange(int bottom, int top);
            void clearText();

            Goalpopup(Wt::WContainerWidget* parent = 0, int current=0, int final=10);


        };
  }
}

#endif
