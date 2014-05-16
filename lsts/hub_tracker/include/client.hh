#ifndef H_hub_client
# define H_hub_client

# include <boost/asio/deadline_timer.hpp>

# include <Wt/Http/Client>
# include <Wt/WTimer>

# include <boost/date_time/posix_time/posix_time.hpp>
# include <boost/property_tree/ptree.hpp>
# include <boost/atomic.hpp>

#include "message_handler.hh"

namespace lsts_hub {
  
  class client :boost::noncopyable {
  public:
    enum http_errors {
      success = 0, // wheever I have a 200
      server_error // ideallyt I should reflect all the http codes
    };
    
    typedef boost::posix_time::ptime date_type;
    
    explicit client(Wt::WIOService &io);
    ~client();
    
    void connect(std::string const &url);
    
    typedef Wt::Signal<boost::system::error_code> error_sig;
    typedef Wt::Signal<date_type, boost::property_tree::ptree> trex_sig;
    typedef Wt::Signal<date_type, std::string> msg_sig;
    typedef Wt::Signal<date_type> state_sig;
    typedef Wt::Signal<date_type> date_sig;
    
    error_sig &on_error() {
      return m_on_error;
    }
    trex_sig &on_trex() {
      return m_trex;
    }
    msg_sig &on_msg() {
      return m_msg;
    }
    state_sig &on_state() {
      return m_state;
    }
    date_sig &on_fresh() {
      return m_fresh;
    }
    
    boost::asio::io_service &service();
    
    void poll();
    
  private:
    Wt::WIOService   &m_io;
    Wt::Http::Client  m_http;
    boost::asio::deadline_timer   m_timer;
    boost::asio::io_service::work m_work;
    message_handler handle;
    boost::atomic<bool> m_polling;
    
    
    std::string      m_base_url;
    boost::posix_time::ptime    m_last;
    
    error_sig m_on_error;
    state_sig m_state;
    trex_sig  m_trex;
    msg_sig   m_msg;
    date_sig m_fresh;
    
    void send_request();
    void process_request(boost::system::error_code,
                         Wt::Http::Message);
    
    void process_body(std::string txt);
    void on_timeout(boost::system::error_code const &ec);
    
    
    client();
  };
  
} // lsts_hub

#endif // H_hub_client
