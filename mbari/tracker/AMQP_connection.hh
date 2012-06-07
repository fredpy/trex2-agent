#ifndef H_AMQP_connection
# define H_AMQP_connection

# include <cstdlib>
# include <cstdio>
# include <cstring>

# include <stdint.h>

# include <amqp.h>
# include <amqp_framing.h>

# include <string>
# include <stdexcept>

# include <boost/shared_ptr.hpp>

namespace mbari {
  namespace amqp {

    class queue;
    typedef boost::shared_ptr<queue> queue_ref;

    namespace details {
      int const FRAME_MAX = 131072;
    }

    class connection {
    public:
      class error :public std::runtime_error {
      public:
	error(std::string const &msg) throw()
	  :std::runtime_error("amqp: "+msg) {}
	virtual ~error() throw() {}
      }; // mbari::amqp::connection::error

      connection();
      ~connection();

      bool is_open() const {
	return m_socket>=0;
      }
      
      void open(std::string const &host, int port);
      void login(std::string const &accnt, std::string const &pwd,
		 std::string const &vhost="/");
      void close();

      queue_ref create_queue();
      queue_ref create_queue(std::string const &name);

    private:
      int                     m_socket;
      amqp_connection_state_t m_conn;
      size_t                  m_channels;
      ssize_t                 m_consumer;

      static void check_rpc_reply(amqp_rpc_reply_t const &ans,
				  std::string const &context);
      void check_rpc_reply(std::string const &context) const;

      size_t new_channel();
      void close_channel(size_t chan);

      friend class queue;
    }; // mbari::amqp::connection

  } // mbari::amqp
} // mbari

#endif // H_AMQP_connection
