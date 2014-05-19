#include "msg_board.hh"

#include <Wt/WApplication>
#include <Wt/WGridLayout>
#include <Wt/WPanel>
#include <Wt/WText>

using namespace lsts_hub;
namespace bpt=boost::property_tree;

/*
 * class lsts_hub::msg_board
 */

// structors

msg_board::msg_board(Wt::WContainerWidget *parent)
:Wt::WContainerWidget(parent), m_me(NULL) {
  // Create the timer to display updates
  m_update_timer = new Wt::WTimer(this);
  m_update_timer->setInterval(1000);
  m_update_timer->timeout().connect(this, &msg_board::page_update);
  
  inject_notification_script();
  
  m_messages = new Wt::WGroupBox("Operator's messages");
  m_messages->setOverflow(Wt::WContainerWidget::OverflowAuto);
  
  m_observations = new Wt::WGroupBox("AUV updates");
  m_observations->setOverflow(Wt::WContainerWidget::OverflowAuto);
  
  m_goals = new Wt::WGroupBox("AUV commands");
  m_goals->setOverflow(Wt::WContainerWidget::OverflowAuto);
  
  m_messages->setMinimumSize(Wt::WLength::Auto,
                             Wt::WLength(200));
  m_goals->setMinimumSize(Wt::WLength::Auto,
                          Wt::WLength(200));
  
  Wt::WGridLayout *layout = new Wt::WGridLayout;
  layout->addWidget(m_observations, 0, 0, 2, 1);
  layout->addWidget(m_goals, 0, 1);
  layout->addWidget(m_messages, 1, 1);

  setLayout(layout);
  m_update_timer->start();
}

msg_board::~msg_board() {
  m_me = NULL;
  m_update_timer->stop();
}

// Initializers

void msg_board::inject_notification_script() {
  std::ostringstream js;
  
  js<<"function (title, body) {\n"
  // Build a unique id
  <<"  function s4() {\n"
  <<"    return Math.floor((1+Math.random())*0x10000).toString(16).substring(1);\n"
  <<"  }\n"
  <<" var uuid = s4()+s4()+'-'+s4()+'-'+s4()+'-'+s4()+s4()+s4()+s4();"
  // Check for notification compatibility
  <<"  if( !'Notification' in window ) {\n"
  // Note I may want to set a flag in my C++ class
  //  to refelect the lack of support
  <<"     return;" // No support
  <<"  } \n"
  // <<"  concole.log(Notification.permission);\n"
  <<"  if( Notification.permission === 'default' ) {\n"
  <<"    Notification.requestPermission(function () {\n"
  // I can do  extra stuff here handling permission level
  // update
  <<"    });\n"
  <<"  } else if( Notification.permission === 'granted' ) {\n"
  <<"    var n = new Notification(\n"
  <<"      title,\n"
  <<"      {\n"
  <<"         'body': body,\n"
  <<"         'tag': uuid\n"
  <<"      }\n"
  <<"    );\n"
  // Close notification when clicked
  <<"    n.onclick = function() { this.close(); };\n"
  <<"  }\n"
  <<"};";
 
  m_me = Wt::WApplication::instance();
  m_me->declareJavaScriptFunction("notify", js.str());
}


// Manipulators

void msg_board::op_update(client::date_type const &when,
                          std::string const &what) {
  mutex::scoped_lock lock(m_mtx);
  m_msg[when] = what;
}

void msg_board::trex_update(client::date_type const &when,
                            bpt::ptree const &what) {
  mutex::scoped_lock lock(m_mtx);
  m_trex[when] = what;
}

// refresh management

void msg_board::send_notify(std::string const &title, std::string const &msg,
                            client::date_type const &when) {
  if( !when.is_special() ) {
    client::date_type limit = boost::posix_time::second_clock::universal_time();
    limit -= boost::posix_time::hours(2);
    if( limit>when )
      return;
  }
  if( NULL!=m_me )
    m_me->doJavaScript(m_me->javaScriptClass()+".notify('"+title+"', '"+msg+"')");
}

void msg_board::page_update() {
  size_t count = 0;
  size_t const max(20);
  
  for( ; display_op() && count<max; ++count);
  for( ; display_trex() && count<max; ++count);
  
  m_me->triggerUpdate();
}

bool msg_board::display_op() {
  std::string title, msg;
  client::date_type when;
  {
    mutex::scoped_lock lock(m_mtx);
    
    if( m_msg.empty() )
      return false;
    else {
      msg_events::iterator i = m_msg.begin();
      std::ostringstream dss;
      
      when = i->first;
      dss<<when<<" UTC";
      title = dss.str();
      
      msg = i->second;
      m_msg.erase(i);
    }
  }
  Wt::WPanel *entry = new Wt::WPanel(this);
  entry->setTitle(title);
  entry->setCentralWidget(new Wt::WText(msg));
  entry->setCollapsible(false);
  m_messages->insertWidget(0, entry);
  send_notify("OP message", msg, when);
  
  return true;
}

bool msg_board::display_trex() {
  std::string type, token, attributes;
  client::date_type when;
  Wt::WString txt;
  {
    mutex::scoped_lock lock(m_mtx);
    if( m_trex.empty() )
      return false;
    else {
      trex_events::iterator i = m_trex.begin();

      when = i->first;
      type = i->second.get<std::string>("op");
      token = i->second.get<std::string>("timeline")+"."
        +i->second.get<std::string>("predicate");
      
      boost::optional<bpt::ptree &> attr = i->second.get_child_optional("attributes");
      if( attr ) {
        for(bpt::ptree::const_iterator j=attr->begin();
            attr->end()!=j; ++j) {
          std::string name = j->second.get<std::string>("name");
          boost::optional<std::string> val = j->second.get_optional<std::string>("value");
          if( !val ) {
            std::ostringstream oss;
            oss<<'['<<j->second.get<std::string>("min")<<", "
               <<j->second.get<std::string>("max")<<']';
            val = oss.str();
          }
          if( val )
            attributes += "<b>"+name+"</b> = "+(*val)+"<br/>";
        }
      }
      m_trex.erase(i);
    }
  }
  Wt::WPanel *entry = new Wt::WPanel(this);
  std::ostringstream title;
  title<<when<<" UTC - <em>"<<token<<"</em>";
  entry->setTitle(title.str());
  entry->setCentralWidget(new Wt::WText(attributes));
  entry->setCollapsible(true);
  entry->setCollapsed(true);
  if( "observation"==type )
    m_observations->insertWidget(0, entry);
  else
    m_goals->insertWidget(0, entry);
  send_notify("TREX "+type,
              token+" at "+boost::posix_time::to_iso_extended_string(when)+" UTC",
              when);
  return true;
}





