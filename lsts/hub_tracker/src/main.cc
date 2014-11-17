#include <iostream>

#include "server.hh"
#include "msg_board.hh"

#include <boost/make_shared.hpp>
#define BOOST_SPIRIT_THREADSAFE
#include <boost/property_tree/json_parser.hpp>
#include <boost/thread.hpp>

#include <Wt/WTabWidget>
#include <Wt/WGoogleMap>
#include <Wt/WBorderLayout>
#include <Wt/WText>
#include <Wt/WToolBar>
#include <Wt/WPushButton>
#include <Wt/WOverlayLoadingIndicator>
#include <Wt/WStackedWidget>

#include <DUNE/Coordinates/WGS84.hpp>

using namespace lsts_hub;
namespace bpt=boost::property_tree;
namespace json=bpt::json_parser;

void client_error(boost::system::error_code ec) {
  std::cerr<<"Error: "<<ec.message()<<std::endl;
  exit(1);
}

void trex_update(client::date_type when, bpt::ptree msg) {
  std::cout<<">>> "<<boost::posix_time::to_iso_extended_string(when)<<" <<<\n";
  json::write_json(std::cout, msg);
  std::cout<<std::endl;
}

namespace {
  
  class hub_application :public Wt::WApplication, public server::App {
  public:
    hub_application(Wt::WEnvironment const &env,
                    boost::shared_ptr<server> const &s)
    :Wt::WApplication(env), m_server(s) {
      m_update_timer = new Wt::WTimer(this);
      m_update_timer->setInterval(100);
      m_update_timer->timeout().connect(this, &hub_application::page_update);
    
      this->setLoadingIndicator(new Wt::WOverlayLoadingIndicator);

      Wt::WBorderLayout *layout = new Wt::WBorderLayout;
      root()->setLayout(layout);
      this->setCssTheme("polished");
      styleSheet().addRule(".hub-time",
                           "font-family: Verdana;"
                           "font-size: 10pt;"
                           "font-weight: bold;"
                           "color: grey;");
      
      Wt::WTabWidget *tabs = new Wt::WTabWidget;
      tabs->contentsStack()->setTransitionAnimation(Wt::WAnimation(Wt::WAnimation::Fade|Wt::WAnimation::SlideInFromTop, Wt::WAnimation::EaseIn));
      
      messages = new msg_board;
      tabs->addTab(messages, "Messages");
      
      map = new Wt::WGoogleMap(Wt::WGoogleMap::Version3);
      // A hard coded initial position of Olhão experiment
      Wt::WGoogleMap::Coordinate olhao(36.949716, -7.7278297);
      map->setCenter(olhao, 12);
      
      tabs->addTab(map, "Map");
      layout->addWidget(tabs, Wt::WBorderLayout::Center);
      
      Wt::WToolBar *bar = new Wt::WToolBar;
      
      time = new Wt::WText;
      bar->addWidget(time);
      time->setSelectable(false);
      time->setToolTip("Current UTC date");
      time->addStyleClass("hub-time");
      bar->addSeparator();
      
      date = new Wt::WText;
      date->setSelectable(false);
      date->setToolTip("Duration since last message");
      date->addStyleClass("hub-time");
      bar->addWidget(date, Wt::AlignRight);
      
      poll = new Wt::WPushButton("Polling");
      poll->setDisabled(true);
      poll->setSelectable(false);
      poll->clicked().connect(this, &hub_application::send_poll);
      poll->addStyleClass("hub-time");
      poll->Wt::WWebWidget::setToolTip("Trigger polling from hub");
      
      // bar->addButton(poll, Wt::AlignRight);
      
      layout->addWidget(bar, Wt::WBorderLayout::North);
      layout->addWidget(poll, Wt::WBorderLayout::South);
      layout->setSpacing(0);
      
      setTitle("LSTS hub tracker for TREX operation");
      
      m_server->connect(this);
      m_update_timer->start();
    }
    ~hub_application() {
      m_update_timer->stop();
      m_server->disconnect(this);
    }
                              
    void send_poll() {
      m_server->poll();
    }
   
  private:
    boost::recursive_mutex m_mtx;
    client::date_type m_date, m_next;
    bool m_state_updated;
    
    typedef std::vector<Wt::WGoogleMap::Coordinate> line;
    
    
    void set_point(line &l, double scale, double sin_a, double cos_a,
                   Wt::WGoogleMap::Coordinate &center) {
      double x, y, n, e, dummy;
      
      for(line::iterator i=l.begin(); l.end()!=i; ++i) {
        double lat_rad = center.latitude(), lon_rad = center.longitude();
        x = i->longitude()*scale;
        y = i->latitude()*scale;
        n = x*cos_a + y*sin_a;
        e = y*cos_a - x*sin_a;
        lat_rad *= M_PI/180.0;
        lon_rad *= M_PI/180.0;
        
        DUNE::Coordinates::WGS84::displace(n, e, 0,
                                           &lat_rad, &lon_rad, &dummy);
        i->setLatitude(lat_rad*180.0/M_PI);
        i->setLongitude(lon_rad*180.0/M_PI);
        std::cerr<<"Rotated point: ("<<i->latitude()<<", "<<i->longitude()<<")"<<std::endl;
      }
    }

    
    typedef std::map< client::date_type,
                      std::pair<line, Wt::WColor> > circle_map;
    
    circle_map m_circles;
    
    boost::shared_ptr<server> m_server;
    msg_board      *messages;
    Wt::WGoogleMap *map;
    Wt::WText      *time;
    Wt::WText      *date;
    Wt::WPushButton    *poll;
    Wt::WTimer     *m_update_timer;
    
    
    
    void page_update() {
      boost::recursive_mutex::scoped_lock lock(m_mtx);
      std::ostringstream oss;
      client::date_type cur = boost::posix_time::second_clock::universal_time();
      
      
      if( m_state_updated ) {
        m_state_updated = false;
        if( m_next.is_special() ) {
          poll->setDisabled(true);
          poll->setText("Polling");
          this->loadingIndicator()->widget()->show();
        } else {
          oss<<m_next<<" UTC";
          poll->setText(oss.str());
          poll->setDisabled(false);
          oss.str(std::string());
          this->loadingIndicator()->widget()->hide();
        }
      }
      oss<<cur<<" UTC&nbsp;";
      time->setText(oss.str());
      
      oss.str(std::string());
      oss<<"&nbsp;Last msg: "<<(cur-m_date)<<" ago&nbsp;";
      date->setText(oss.str());
      
      while( !m_circles.empty() ) {
        circle_map::iterator i = m_circles.begin();
        map->addPolyline(i->second.first, i->second.second);
        m_circles.erase(i);
      }
    }
    
    void set_state(client::date_type const &next) {
      client::date_type::date_type day = next.date();
      client::date_type::time_duration_type
        hour = next.time_of_day();
      
      // Filter out subsecond info
      hour = client::date_type::time_duration_type(hour.hours(),
                                                   hour.minutes(),
                                                   hour.seconds());
      
      {
        boost::recursive_mutex::scoped_lock lock(m_mtx);
      
        m_next = client::date_type(day, hour);
        m_state_updated = true;
      }
    }
    void new_date(client::date_type const &d) {
      boost::recursive_mutex::scoped_lock lock(m_mtx);
      m_date = d;
    }
    
    void op_update(client::date_type const &when,
                   std::string const &what) {
      messages->op_update(when, what);
    }
    void trex_update(client::date_type const &when,
                     bpt::ptree const &what) {
      std::cerr<<"Send op to message view"<<std::endl;
      messages->trex_update(when, what);
      
      
      try {
        // Extract data
        std::string type = what.get<std::string>("op");
        Wt::WColor color;
        if( type=="request" )
          color = Wt::WColor(255, 0, 0);
        else if( type=="observation" ) {
          std::string pred = what.get<std::string>("predicate");
          if( pred=="Survey" )
            color = Wt::WColor(0, 255, 0);
          else if( pred=="DoSurvey" )
            color = Wt::WColor(255, 255, 0);
          else
            return;
        } else
          return;
        boost::optional<bpt::ptree const &> attrs = what.get_child_optional("attributes");
        boost::optional<double> radius, lat, lon, sin_a, cos_a;
        double angle=0.0;
          
        if( attrs ) {
          for(bpt::ptree::const_iterator i=attrs->begin();
              attrs->end()!=i; ++i) {
            std::string name = i->second.get<std::string>("name");
            
            if( "size"==name )
              radius = i->second.get<double>("value");
            else if( "center_lat"==name ) {
              double tmp;
              std::istringstream iss(i->second.get<std::string>("value"));
              iss>>tmp;
              lat = tmp;
            } else if( "center_lon"==name ) {
              double tmp;
              std::istringstream iss(i->second.get<std::string>("value"));
              iss>>tmp;
              lon = tmp;
            } else if( "heading"==name ) {
              double tmp;
              std::istringstream iss(i->second.get<std::string>("value"));
              iss>>tmp;
              angle = tmp*M_PI/180.0;
              sin_a = std::sin(angle);
              cos_a = std::cos(angle);
            } else if( "sin_angle"==name )
              sin_a = i->second.get<double>("value");
            else if( "cos_angle"==name )
              cos_a = i->second.get<double>("value");
          }
          if( radius && lat && lon && sin_a && cos_a ) {
            line tmp(5);
            tmp[0] = Wt::WGoogleMap::Coordinate(-0.5, 0.5);
            tmp[1] = Wt::WGoogleMap::Coordinate(0.5, 0.5);
            tmp[2] = Wt::WGoogleMap::Coordinate(0.5, -0.5);
            tmp[3] = Wt::WGoogleMap::Coordinate(-0.5, -0.5);
            tmp[4] = tmp[0];
              
            Wt::WGoogleMap::Coordinate center(*lat, *lon);
            set_point(tmp, *radius, *sin_a, *cos_a, center);
            {
              boost::recursive_mutex::scoped_lock lock(m_mtx);
              m_circles[when].first = tmp;
              m_circles[when].second = color;
            }
          }
        }
      } catch(...) {
        std::cerr<<"Something something BAD ..."<<std::endl;
      }
    }
    
  };
  
  hub_application *new_hub_view(Wt::WEnvironment const &env,
                                boost::shared_ptr<server> serv) {
    return new hub_application(env, serv);
  }

  
}

int main(int argc, char **argv) {
  try {
    // Initiate a Wt http server
    Wt::WServer wserv(argv[0]);
    boost::shared_ptr<server> me = boost::make_shared<server>(boost::ref(wserv));
    
    me->hub().on_error().connect(boost::bind(&client_error, _1));
    me->hub().on_trex().connect(boost::bind(&trex_update, _1, _2));

    wserv.setServerConfiguration(argc, argv, WTHTTP_CONFIGURATION);
    wserv.addEntryPoint(Wt::Application, boost::bind(&new_hub_view, _1, me));
    me->init("http://hub.lsts.pt/api/v1/iridium");

    
    boost::thread_group thr;
    thr.create_thread(boost::bind(&boost::asio::io_service::run,
                                  &(me->hub().service())));

    if( wserv.start() ) {
      int sig = Wt::WServer::waitForShutdown(argv[0]);
      
      std::cerr<<"Server shutdown with status: "<<sig<<std::endl;
      wserv.stop();
    }
  } catch(std::exception const &e) {
    std::cerr<<"Exception caught: "<<e.what()<<std::endl;
    return 1;
  }
}

//
//#include <boost/property_tree/json_parser.hpp>
//#include <boost/thread.hpp>
//#include <boost/make_shared.hpp>
//
//#include <Wt/WGroupBox> 
//#include <Wt/WHBoxLayout>
//#include <Wt/WVBoxLayout>
//
//#include <Wt/WTemplate>
//
//#include <Wt/WPanel>
//#include <Wt/WPushButton>
//#include <Wt/WText>
//#include <Wt/WTextArea>
//
//namespace asio=boost::asio;
//namespace bpt=boost::property_tree;
//namespace json=bpt::json_parser;
//
//using namespace lsts_hub;
//
//
//namespace {
//  
//  
//  class hub_application: public Wt::WApplication, public server::App {
//  public:
//    
//    hub_application(Wt::WEnvironment const &env,
//                    boost::shared_ptr<server> const &s)
//    :Wt::WApplication(env), m_server(s)
//    , m_fresh(true), m_last(boost::posix_time::not_a_date_time) {
//      m_timer = new Wt::WTimer(this);
//      m_timer->setInterval(1000);
//      m_timer->timeout().connect(this, &hub_application::page_update);
//      
//      inject_notification_script();
//      
//      setCssTheme("polished");
//      messages = new Wt::WGroupBox("Operator's Messages");
//      messages->setOverflow(Wt::WContainerWidget::OverflowAuto);
//      
//      trex = new Wt::WGroupBox("AUV updates");
//      trex->setOverflow(Wt::WContainerWidget::OverflowAuto);
//
//      commands = new Wt::WGroupBox("AUV commands");
//      commands->setOverflow(Wt::WContainerWidget::OverflowAuto);
//      
//      messages->setMinimumSize(Wt::WLength::Auto,
//                               Wt::WLength(200));
//      commands->setMinimumSize(Wt::WLength::Auto,
//                               Wt::WLength(200));
//      
//      date = new Wt::WText;
//      date->setHeight(Wt::WLength(20));
//      date->setMaximumSize(Wt::WLength::Auto,
//                           Wt::WLength(20));
//      date->setSelectable(false);
//
//      fresh = new Wt::WPushButton;
//      fresh->setHeight(Wt::WLength(20));
//      fresh->setMaximumSize(Wt::WLength::Auto,
//                           Wt::WLength(20));
//      fresh->setSelectable(false);
//      fresh->clicked().connect(this, &hub_application::trigger_poll);
//      fresh->setToolTip("Trigger message polling from hub");
//      
//      Wt::WGridLayout *layout = new Wt::WGridLayout;
//
//      layout->addWidget(date, 2, 0);
//      layout->addWidget(fresh, 2, 1);
//      layout->setRowResizable(2, true, Wt::WLength(22));
//      layout->addWidget(trex, 0, 0, 2, 1);
//      layout->addWidget(commands, 0, 1);
//      layout->addWidget(messages, 1, 1);
//      
//      root()->setLayout(layout);
//      
//      m_server->connect(this);
//      m_timer->start();
//    }
//    
//    ~hub_application() {
//      m_timer->stop();
//      m_server->disconnect(this);
//    }
//    
//  private:
//    boost::shared_ptr<server> m_server;
//    client::date_type m_nxt;
//    size_t         m_dots;
//    Wt::WTimer    *m_timer;
//    Wt::WText     *date;
//    Wt::WPushButton    *fresh;
//    Wt::WGroupBox *messages;
//    Wt::WGroupBox *trex;
//    Wt::WGroupBox *commands;
//    
//    typedef std::map<client::date_type, std::string> msg_events;
//    typedef std::map<client::date_type, boost::property_tree::ptree> trex_events;
//    
//    boost::recursive_mutex m_mtx;
//    msg_events m_messages;
//    trex_events m_trex;
//    bool              m_fresh;
//    client::date_type m_last;
//    
//    void trigger_poll() {
//      m_server->poll();
//    }
//    
//    void inject_notification_script() {
//      std::ostringstream js;
//      
//      js<<"function (title, body) {\n"
//      // Build a unique id
//      <<"  function s4() {\n"
//      <<"    return Math.floor((1+Math.random())*0x10000).toString(16).substring(1);\n"
//      <<"  }\n"
//      <<" var uuid = s4()+s4()+'-'+s4()+'-'+s4()+'-'+s4()+s4()+s4()+s4();"
//      // Check for notification compatibility
//      <<"  if( !'Notification' in window ) {\n"
//      // Note I may want to set a flag in my C++ class
//      //  to refelect the lack of support
//      <<"     return;" // No support
//      <<"  } \n"
//      // <<"  concole.log(Notification.permission);\n"
//      <<"  if( Notification.permission === 'default' ) {\n"
//      <<"    Notification.requestPermission(function () {\n"
//      // I can do  extra stuff here handling permission level
//      // update
//      <<"    });\n"
//      <<"  } else if( Notification.permission === 'granted' ) {\n"
//      <<"    var n = new Notification(\n"
//      <<"      title,\n"
//      <<"      {\n"
//      <<"         'body': body,\n"
//      <<"         'tag': uuid\n"
//      <<"      }\n"
//      <<"    );\n"
//      // Close notification when clicked
//      <<"    n.onclick = function() { this.close(); };\n"
//      <<"  }\n"
//      <<"};";
//      this->declareJavaScriptFunction("notify", js.str());
//    }
//    
//    void page_update() {
//      bool d_fresh = false;
//      client::date_type d;
//      
//      // Display the current date as reference
//      std::ostringstream oss;
//      oss<<"<b style=\"font-family: Verdana; font-size: 10pt;"
//      " color: grey;\">Current date: "
//      <<boost::posix_time::second_clock::universal_time()
//      <<" UTC</b>";
//      date->setText(oss.str());
//
//      // Update the title depending on hub client state
//      client::date_type nxt;
//      bool update = false;
//      size_t dots;
//      {
//        boost::recursive_mutex::scoped_lock lock(m_mtx);
//        nxt = m_nxt;
//        if( nxt.is_special() ) {
//          m_dots = (m_dots+1)%8;
//          if( 0==(m_dots%2) ) {
//            dots = m_dots/2;
//            update = true;
//          }
//        }
//        if( m_fresh ) {
//          m_fresh = false;
//          d_fresh = true;
//          d = m_last;
//        }
//      }
//      if( d_fresh ) {
//        oss.str(std::string());
//        oss<<"Last update: "<<d<<" UTC";
//        fresh->setText(oss.str());
//      }
//      if( nxt.is_special() ) {
//        if( fresh->isEnabled() )
//          fresh->disable();
//      } else if( fresh->isDisabled() )
//        fresh->enable();
//
//      if( update )
//        setTitle("Polling"+std::string(m_dots/2, '.'));
//      // look for extra updates
//      while( display_op() );
//    }
//    
//    void set_state(client::date_type const &nxt) {
//      boost::recursive_mutex::scoped_lock lock(m_mtx);
//      m_nxt = nxt;
//      m_dots = 0;
//      if( m_nxt.is_special() )
//        setTitle("Polling");
//      else
//        setTitle("Next poll: "+boost::posix_time::to_iso_extended_string(m_nxt)+" UTC");
//    }
//    
//    void new_date(client::date_type const &date) {
//      boost::recursive_mutex::scoped_lock lock(m_mtx);
//      m_last = date;
//      m_fresh = true;
//    }
//    
//    /** @brief Send a web page notification
//     *
//     * @param[in] title notification title
//     * @param[in] msg notification body
//     *
//     * This function executes javascript code that, for browsers that support it
//     * , will create anotification ofrom the web page. 
//     *
//     * @note Right now only Safari for mac os X.9 (maverick) support this 
//     * feature
//     */
//    void send_notify(std::string const &title, std::string const &msg,
//                     client::date_type const &when) {
//      if( !when.is_special() ) {
//        client::date_type limit = boost::posix_time::second_clock::universal_time()-boost::posix_time::hours(2);
//        if( limit>when )
//          return;
//      }
//      this->doJavaScript(this->javaScriptClass()+".notify('"+title+"', '"+msg+"')");
//    }
//
//    
//    bool display_op() {
//      std::string title, msg;
//      client::date_type when;
//      {
//        boost::recursive_mutex::scoped_lock lock(m_mtx);
//        if( m_messages.empty() )
//          return display_trex();
//        else {
//          msg_events::iterator i = m_messages.begin();
//          std::ostringstream dss;
//          when = i->first;
//          dss<<when<<" UTC";
//          title = dss.str();
//          std::ostringstream oss;
//          msg = i->second;
//          m_messages.erase(i);
//        }
//      }
//      Wt::WPanel *entry = new Wt::WPanel;
//      entry->setTitle(title);
//      entry->setCentralWidget(new Wt::WText(msg));
//      entry->setCollapsible(false);
//      messages->insertWidget(0, entry);
//      send_notify("OP message", msg, when);
//      return true;
//    }
//    
//    bool display_trex() {
//      std::string title, msg;
//      Wt::WGroupBox *dest = commands;
//      Wt::WString text;
//      {
//        boost::recursive_mutex::scoped_lock lock(m_mtx);
//        if( m_trex.empty() )
//          return false;
//        else {
//          client::date_type when;
//          trex_events::iterator i = m_trex.begin();
//
//          std::string type = i->second.get<std::string>("op");
//
//          if( type=="observation" )
//            dest = trex;
//          
//          std::string token = i->second.get<std::string>("timeline")+"."+
//          i->second.get<std::string>("predicate");
//          
//          
//          std::ostringstream dss;
//          dss<<i->first<<" UTC - <em style=\"color: #FFFF00\">"
//            <<token<<"</em>";
//          title = dss.str();
//          
//          send_notify("TREX "+type, token+" at "+boost::posix_time::to_iso_extended_string(i->first), i->first);
//          
//          boost::optional<bpt::ptree &> attr = i->second.get_child_optional("attributes");
//          if( attr ) {
//            size_t rank = 0;
//            for(bpt::ptree::const_iterator j=attr->begin();
//                attr->end()!=j; ++j, ++rank) {
//              std::string name = j->second.get<std::string>("name");
//              
//              boost::optional<std::string> val = j->second.get_optional<std::string>("value");
//              if( !val ) {
//                std::ostringstream oss;
//                oss<<'['<<j->second.get<std::string>("min")<<", "
//                <<j->second.get<std::string>("max")<<']';
//                val = oss.str();
//              }
//              if( val )
//                text += "<b>"+name+"</b> = "+(*val)+"<br/>";
//            }
//          }
//          
//          m_trex.erase(i);
//        }
//      }
//      Wt::WPanel *entry = new Wt::WPanel(root());
//      entry->setTitle(title);
//      entry->setCentralWidget(new Wt::WText(text));
//      entry->setCollapsible(true);
//      entry->setCollapsed(true);
//      // entry->setDisabled(false);
//      dest->insertWidget(0, entry);
//      return true;
//    }
//
//    void op_update(client::date_type const &when,
//                   std::string const &what) {
//      boost::recursive_mutex::scoped_lock lock(m_mtx);
//      m_messages[when] = what;
//    }
//
//    void trex_update(client::date_type const &when,
//                     boost::property_tree::ptree const &what) {
//      boost::recursive_mutex::scoped_lock lock(m_mtx);
//      m_trex[when] = what;
//    }
//
//  };
//  
//  hub_application *new_hub_view(Wt::WEnvironment const &env,
//                                boost::shared_ptr<server> serv) {
//    return new hub_application(env, serv);
//  }
//  
//}
//
//int main(int argc, char* argv[]) {
//  try {
//    // Initiate a Wt http server
//    Wt::WServer wserv(argv[0]);
//    boost::shared_ptr<server> me = boost::make_shared<server>(boost::ref(wserv));
//    
//    
//    me->hub().on_error().connect(boost::bind(&client_error, _1));
//    me->hub().on_trex().connect(boost::bind(&trex_update, _1, _2));
//    
//    wserv.setServerConfiguration(argc, argv, WTHTTP_CONFIGURATION);
//    wserv.addEntryPoint(Wt::Application, boost::bind(&new_hub_view, _1, me));
//    me->init("http://hub.lsts.pt/api/v1/iridium");
//
//    boost::thread_group thr;
//    thr.create_thread(boost::bind(&boost::asio::io_service::run,
//                                  &(me->hub().service())));
//    
//    if( wserv.start() ) {
//      int sig = Wt::WServer::waitForShutdown(argv[0]);
//      std::cerr<<"Server shutdown with status: "<<sig<<std::endl;
//      wserv.stop();
//    }
//    
//    // If we reachthis point cli has nothing left to do
//    return 0;
//  } catch(std::exception &e) {
//    // If we reach this point some calls in cli did
//    // produce an exception
//    std::cerr<<"Exception caught: "<<e.what()<<std::endl;
//    return 1;
//  }
//}