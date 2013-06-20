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
#ifndef H_AMQP_queue
# define H_AMQP_queue

# include <memory>

# include "AMQP_connection.hh"

namespace mbari {
  namespace amqp {

    /** @brief amqp queue
     *
     * This class represent a amqp queue as creted by the connection class. 
     * The queue can be bound to server exchanges allowing to send and receive 
     * messages from the server.
     *
     * @author Frederic Py <fpy@mbari.org>
     * @ingroup amqp
     * @relates connection
     */
    class queue {
    public:
      /** @brief Destructor */
      ~queue();
      
      /** @brief queue name
       *
       * Gives the name of the queue as requested at construction or as provided 
       * by the server
       *
       * @return The name of the queue
       */
      std::string name() const {
	return std::string(static_cast<char *>(m_name.bytes), m_name.len);
      }

      /** @brief configure queue
       *
       * @param[in] no_local  @c true if the server should not deliver messages produced locally
       * @param[in] no_ack    @c true if the server does not need to be acknowledged of messages reception;
       *                      @c false if the server should expect explicit aknowledgment
       * @param[in] exclusive @c true if this queue is exclusive (only one  client)
       *
       * Set connection parameters for this queue
       * @throw connection::error Erro while setting connection parameters 
       */
      void configure(bool no_local, bool no_ack, bool exclusive=false);
      /** @brief Bnf the queue
       *
       * @param[in] exchange Name of an exchange
       * @param[in] key A routing key
       *
       * Bind this queue to the exechange @p exchange with the routing key @p key. Any 
       * messages corresponding to this @p exchange  and @p key will be sent to this queue.
       *
       * @throw connection::error Binding failed
       *
       * @sa message::exchange() const
       * @sa message::key() const
       */
      void bind(std::string const &exchange, std::string const &key);

      /** @brief amqp message
       *
       * A message received on an amqp queue
       *
       * @author Frederic Py 
       * @relates queue
       * @ingroup amqp
       */
      class message {
      public:
        /** @brief Constructor
         * 
         * Creates ane empty message
         */
	message()
	  :m_size(0), m_body(NULL) {}
        /** @brief Destructor */
	~message() {
	  if( NULL!=m_body )
	    delete[] m_body;
	}

        /** @brief Echange origin
         *
         * Indicates the exchange where this message is coming from.
         *
         * @return the name of the exchange
         * @sa queue::bind(std::string const &, std::string const &)
         * @sa key() const
         */
	std::string const &exchange() const {
	  return m_exchange;
	}
        /** @brief Routing key origin
         *
         * Indicates the routing key of this message.
         *
         * @return the routing key
         * @sa queue::bind(std::string const &, std::string const &)
         * @sa exchange() const        
         */
	std::string const &key() const {
	  return m_key;
	}
        /** @brief Message length
         *
         * @return the size of the message
         * @sa body() const
         */
	size_t size() const {
	  return m_size;
	}
        /** @brief Message content
         *
         *
         * @return a pointer to the body of the message.
         *
         * @sa size() const
         */
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
      
      /** @brief Get next message
       *
       * This blcoking call wait for the next message coming in this queue.
       *
       * @throw connection::error An error occured while receviing the message
       * @return A pointer to the new message
       */
      SHARED_PTR<queue::message> consume();

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
