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
#include <trex/domain/FloatDomain.hh>

using namespace TREX::utils;
using namespace TREX::transaction;
using namespace std;

namespace TREX {
  namespace witre {
        class Goalpopup : public Wt::WCompositeWidget {
            private:
            Wt::WContainerWidget* attributes;
            Wt::WCompositeWidget* root;
            Wt::WPushButton* ok;
            Wt::WIntValidator* numRange;
            Wt::WIntValidator* middleRange;
            Wt::Signal<std::map<string, transaction::IntegerDomain>,
                        std::map<string,transaction::FloatDomain> > finished_;
            Wt::Signal< > cancelled_;
            Wt::JSlot* script;
            Wt::JSlot* disableScript;

            Wt::WContainerWidget* inputs;
            std::map<string, std::pair<Wt::WLineEdit*,Wt::WLineEdit*> > standards;
            std::list<std::pair<Wt::WInPlaceEdit*, Wt::WLineEdit*> > additions;
            Wt::JSignal<string, string, string> delSignal_;

            void cancel() { cancelled_.emit(); };
            void done();
            void addAttribute();
            void deleteAttribute(string container, string name, string input);

            public:
            Wt::Signal<std::map<string, transaction::IntegerDomain>,
                        std::map<string,transaction::FloatDomain> >& finished() { return finished_; };
            Wt::Signal< >& cancelled() { return cancelled_; };
            Wt::JSignal<string, string, string>& delSignal() { return delSignal_; };
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
