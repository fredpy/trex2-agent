#ifndef H_DrifterTracker 
# define H_DrifterTracker

# include <map>

# include "AMQP_listener.hh"
# include "MessageHandler.hh"

# include <trex/transaction/TeleoReactor.hh>

namespace mbari {

  void geo_to_utm(double lat, double lon, double &north, double &east);
  void utm_to_geo(double north, double east, double &lat,  double &lon);

  class DrifterTracker :public TREX::transaction::TeleoReactor {
  public:
    DrifterTracker(TREX::transaction::TeleoReactor::xml_arg_type arg);
    ~DrifterTracker();

  private:
    void handleInit();
    bool synchronize();
    void handleRequest(TREX::transaction::goal_id const &g);
    void handleRecall(TREX::transaction::goal_id const &g);
    
    void goalHandler(std::string const &timeline, MessageHandler *handle);
    
    // Basic AMQP message handling
    amqp::connection              m_connection;
    amqp::queue_ref               m_queue;
    amqp::msg_buffer              m_messages;    
    std::auto_ptr<amqp::listener> m_listener;
    std::auto_ptr<boost::thread>  m_thread;

    typedef std::multimap<std::string, 
			  boost::shared_ptr<MessageHandler> > handle_map;
    handle_map m_message_handlers;
    std::multimap<std::string, MessageHandler *> m_goal_handlers;

    // std::map<TREX::utils::Symbol, point >  m_drifters;
    TREX::utils::Symbol            m_trexMsg;

    TREX::utils::SingletonUse<MessageHandler::factory> m_msg_factory;
        
    void trex_msg(amqp::queue::message const &msg);

    friend class MessageHandler;
  };

}

#endif // H_DrifterTracker
