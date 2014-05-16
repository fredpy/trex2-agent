#ifndef H_hub_server
# define H_hub_server

# include <boost/noncopyable.hpp>
# include <boost/enable_shared_from_this.hpp>

# include <Wt/WServer>

# include <set>

# include "client.hh"
# include <boost/thread/recursive_mutex.hpp>

namespace bpt=boost::property_tree;

namespace lsts_hub {
  
  class server :boost::noncopyable,
  public boost::enable_shared_from_this<server> {
  public:
    class App {
    protected:
      App() {}
      virtual ~App() {}
      
    public:
      virtual void set_state(client::date_type const &next)=0;
      virtual void op_update(client::date_type const &when,
                             std::string const &what) =0;
      virtual void trex_update(client::date_type const &when,
                               boost::property_tree::ptree const &what) =0;
      virtual void new_date(client::date_type const &date) =0;
    };
    
    explicit server(Wt::WServer &wt);
    ~server();
    
    void init(std::string const &url);
    
    client &hub() {
      return m_client;
    }
    
    void connect(App *);
    void disconnect(App *);
    
    void poll() {
      m_client.poll();
    }
    
  private:
    Wt::WServer &m_server;
    client       m_client;

    client::date_type m_state;
    
    boost::recursive_mutex m_mtx;
    std::set<App *> m_clients;
    typedef std::map<client::date_type, std::string> msg_map;
    typedef std::map<client::date_type, bpt::ptree> trex_map;
    msg_map m_messages;
    trex_map m_trex;
    
    client::date_type m_date;
    
    void trex_update(client::date_type when,
                     boost::property_tree::ptree msg);
    void op_update(client::date_type when,
                   std::string what);
    void state_update(client::date_type date);
    void date_update(client::date_type date);
  };
  
}

#endif // H_hub_server