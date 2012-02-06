#ifndef H_AMQP_queue
# define H_AMQP_queue

# include <memory>

# include "AMQP_connection.hh"

namespace mbari {
  namespace amqp {

    class queue {
    public:
      ~queue();
      
      std::string name() const {
	return std::string(static_cast<char *>(m_name.bytes), m_name.len);
      }

      void configure(bool no_local, bool no_ack, bool exclusive=false);
      void bind(std::string const &exchange, std::string const &key);

      class message {
      public:
	message()
	  :m_size(0), m_body(NULL) {}
	~message() {
	  if( NULL!=m_body )
	    delete[] m_body;
	}

	std::string const &exchange() const {
	  return m_exchange;
	}
	std::string const &key() const {
	  return m_key;
	}
	size_t size() const {
	  return m_size;
	}
	char const *body() const {
	  return m_body;
	}
	
	
      private:
	message(std::string const &exch, std::string const &key, size_t len)
	  :m_exchange(exch), m_key(key), m_size(len), m_body(NULL) {
	  if( len>0 )
	    m_body = new char[len];
	}

	std::string m_exchange, m_key;
	size_t m_size;
	char  *m_body;

	friend class queue;
      };
      
      boost::shared_ptr<queue::message> consume();

    private:
      explicit queue(connection &cn);
      queue(connection &cn, std::string const &name);
      
      
      connection     &m_conn;
      size_t          m_channel;
      amqp_bytes_t    m_name;

      friend class connection;

      queue(); // Intentionally left with no code
    };

  }
}

#endif // H_AMQP_queue
