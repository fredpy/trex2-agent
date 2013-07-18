/*********************************************************************
 * Software License Agreement (BSD License)
 * 
 *  Copyright (c) 2011, MBARI.
 *  All rights reserved.
 * 
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 * 
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the TREX Project nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 * 
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */
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
  /** @brief amqp connection classes
   *
   * A set of classes used as an interface to connect to an amqp server and 
   * collect messages from it.
   *
   * @author Frederic Py <fpy@mbari.org>
   * @ingroup amqp
   */
  namespace amqp {

    class queue;
    typedef boost::shared_ptr<queue> queue_ref;

    namespace details {
      int const FRAME_MAX = 131072;
    }

    /** @brief amqp connection
     *
     * This class implements the connection to an amqp server. The connectuon is 
     * the main entry point in order to create amqp queues that can then be bound 
     * to certain messages dsitributed by the server.
     *
     * @author Frederic Py <fpy@mbari.org>
     * @ingroup amqp
     */
    class connection {
    public:
      /** @brief amqp error
       *
       * The exception used to experess an amqp error
       *
       * @author Frederic Py <fpy@mbari.org>
       * @ingroup amqp
       * @relates connection
       */
      class error :public std::runtime_error {
      public:
        /** @brief Constructor
         *
         * @param[in] masg A text message
         * 
         * Create a new instance with the error message @p msg
         */
	error(std::string const &msg) throw()
	  :std::runtime_error("amqp: "+msg) {}
        /** @brief Destructor
         */
	virtual ~error() throw() {}
      }; // mbari::amqp::connection::error

      /** @brief Constructor 
       *
       * Creates a new instance. The instance is not yet connected to any server
       */
      connection();
      /** @brief Destructor
       *
       * If the connection is open it will discoonnect it from the server
       * @sa close()
       */
      ~connection();

      /** @brief Check for conenection
       *
       * Checks if this insatnce is connected to a server
       *
       * @retval true if the connection is open
       * @retval false if not cvonnected to a server
       */
      bool is_open() const {
	return m_socket>=0;
      }
      
      /** @brief Open connection
       *
       * @param[in] host An host address
       * @param[in] port A post number
       *
       * @pre This connection is not open
       * @pre A amqp server is avaailabel at @p host:@p port address
       *
       * Open this conecction to the server on @p host using the port @p port
       *
       * @throw connection::error An error occured, probably due to one  of the precondition 
       * not being respected
       * @post The connection is open to the server
       *
       * @sa is_open() const
       * @sa close()
       * @sa login(std::string const &, std::string const &, std::string const &)
       */
      void open(std::string const &host, int port);
      /** @brief Log-in to the server
       * @param[in] accnt Account name
       * @param[in] pwd Password
       * @param[in] vhost virtual host path
       *
       * Login to the server using the given parameters.
       *
       * @pre The connection is open
       * @throw connection::error login error 
       * @sa is_open() const
       * @sa open(std::string const &, int)
       */
      void login(std::string const &accnt, std::string const &pwd,
		 std::string const &vhost="/");
      /** @brief Close connection
       *
       * Disconnect from the server if the instance was connected
       *
       * @throw connection::error An error occured while trying to close the connection  
       * @post the conection is closed
       *
       * @sa is_open() const
       */
      void close();

      /** @brief Create a new queue
       *
       * @param[in] name Name of the queue
       *
       * Create new queue on the server in order to send and/or receive 
       * messages. If  @p name is given the queue will be named accordingly
       * otherwise it will have an arbitrary name provided by the server.
       *
       * @pre The connection is open
       * @throw connection::error An error ocured while trying to creat the queue
       *
       * @return A pointer to the queue
       * @sa is_open() const
       * @sa queue
       * @{
       */
      queue_ref create_queue();
      queue_ref create_queue(std::string const &name);
      /** @} */
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
