#ifndef H_AQMP_listener
# define H_AMQP_listener 

# include "AMQP_queue.hh"

# include <boost/thread.hpp>
# include <trex/utils/SharedVar.hh>

namespace mbari {
  namespace amqp {

    class listener {
    public:
      class handler {
      public:
	handler() {}
	virtual ~handler() {}

      protected:
	virtual void handle(boost::shared_ptr<queue::message> const &msg) =0;
	
	friend class listener;
      };

      listener(queue_ref const &q, handler &handle);
      listener(listener const &other);
      ~listener();
      


      void add_handler(handler &);

      void operator()() {
	if( !is_running() )
	  run();
      }
      void stop() {
	m_running = false;
      }
      bool is_running() const;
    private:
      void run();

      
      mutable TREX::utils::SharedVar<bool> m_running;
      queue_ref m_queue;
      handler *m_handler;
    };

    class msg_buffer :public listener::handler {
    public:
      msg_buffer() {}
      ~msg_buffer() {}

      bool empty() const;
      boost::shared_ptr<queue::message> pop();
      
    private:
      void handle(boost::shared_ptr<queue::message> const &msg);

      typedef std::list<boost::shared_ptr<queue::message> > queue_type;

      mutable TREX::utils::SharedVar<queue_type> m_queue;
    };
    
  }
}

#endif // H_AMQP_listener
