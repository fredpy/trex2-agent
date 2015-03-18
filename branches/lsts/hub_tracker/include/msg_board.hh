#ifndef H_msg_board
# define H_msg_board

# include "client.hh"

# include <Wt/WContainerWidget>
# include <Wt/WGroupBox>
# include <Wt/WTimer>

# include <boost/thread/recursive_mutex.hpp>

namespace lsts_hub {
  
  class msg_board :public Wt::WContainerWidget {
  public:
    msg_board(Wt::WContainerWidget *parent = 0);
    ~msg_board();
    
    void op_update(client::date_type const &when,
                   std::string const &what);
    void trex_update(client::date_type const &when,
                     boost::property_tree::ptree const &what);
    
  private:
    Wt::WTimer    *m_update_timer;
    Wt::WGroupBox *m_observations;
    Wt::WGroupBox *m_goals;
    Wt::WGroupBox *m_messages;
    Wt::WApplication *m_me;
    
    typedef std::map<client::date_type, std::string>                 msg_events;
    typedef std::map<client::date_type, boost::property_tree::ptree> trex_events;
    
    typedef boost::recursive_mutex mutex;
    
    mutex m_mtx;
    msg_events   m_msg;
    trex_events  m_trex;
    
    void page_update();
    bool display_op();
    bool display_trex();
        
    void inject_notification_script();
    void send_notify(std::string const &title, std::string const &msg,
                     client::date_type const &when);
    
    
  };
  
}

#endif // H_msg_board