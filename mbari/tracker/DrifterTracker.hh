#ifndef H_DrifterTracker 
# define H_DrifterTracker

# include <map>

# include "AMQP_listener.hh"
# include "MessageHandler.hh"

# include <trex/transaction/TeleoReactor.hh>

namespace mbari {

  class DrifterTracker :public TREX::transaction::TeleoReactor {
  public:
    DrifterTracker(TREX::transaction::TeleoReactor::xml_arg_type arg);
    ~DrifterTracker();

  private:
    void handleInit();
    bool synchronize();
    
    // Basic AMQP message handling
    amqp::connection              m_connection;
    amqp::queue_ref               m_queue;
    amqp::msg_buffer              m_messages;    
    std::auto_ptr<amqp::listener> m_listener;
    std::auto_ptr<boost::thread>  m_thread;

    struct point {
      point() 
	:valid(false), with_speed(false) {}
      void update(time_t t, double n, double e);
      bool is_valid() const {
	return valid;
      }
      bool has_speed() const {
	return with_speed;
      }

      std::pair<double, double> const &speed() const {
	return m_speed;
      }
      time_t last_update() const {
	return date;
      }
      std::pair<double, double> position(time_t now, long int &delta) const;      

    private:
      time_t date;
      std::pair<double, double> m_position, m_speed;
      bool valid, with_speed;
    };

    std::map<TREX::utils::Symbol, point >  m_drifters;
    TREX::utils::Symbol            m_trexMsg;

    TREX::utils::SingletonUse<MessageHandler::factory> m_msg_factory;
    
    
    void trex_msg(amqp::queue::message const &msg);
    void drifter_msg(amqp::queue::message const &msg);
  };

}

#endif // H_DrifterTracker
