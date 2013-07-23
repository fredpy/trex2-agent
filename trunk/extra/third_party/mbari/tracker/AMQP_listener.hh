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
#ifndef H_AQMP_listener
# define H_AMQP_listener 

# include "AMQP_queue.hh"

# include <boost/thread.hpp>
# include <trex/utils/SharedVar.hh>

namespace mbari {
  namespace amqp {

    /** @brief amqp message listener
     *
     * This class allows to listen to an aqmp queue and process the 
     * incoming messages.
     *
     * The interface of this class is compatible with boost thread 
     * implementation allowing to spawn this handler in a different 
     * thread.
     *
     * @author Frederic Py <fpy@mbari.org>
     * @ingroup amqp
     * @relates queue
     */
    class listener {
    public:
      /** @brief Message handler
       *
       * Abstarct interface to handle incoming messages in an amqp 
       * listener
       *
       * @author Frederic Py
       * @relates listener
       * @ingroup amqp
       */
      class handler {
      public:
        /** @brief Constructor */
	handler() {}
        /** @brief Destructor */
	virtual ~handler() {}

      protected:
        /** @brief New message callback
         *
         * @param[in] msg An amqp message
         * 
         * This method is called by the listner whenever a new message @p msg is 
         * received.
         */
	virtual void handle(SHARED_PTR<queue::message> const &msg) =0;
	
	friend class listener;
      }; // mbari::amqp::listener::handler

      /** @brief Constructor 
       * @param[in] q        An amqp queue
       * @param[in] handle  A message handler
       *
       * Create a new instance that will listen to message coming from the 
       * queue @p q and will process them through @p handle
       *
       * @sa queue
       */ 
      listener(queue_ref const &q, handler &handle);
      /** @brief Copy constructor
       *
       * @param[in] other Another listener
       *
       * Create a copy of @p other
       */
      listener(listener const &other);
      /** @brief Destructor */
      ~listener();
      

      /** @brief Set message handler
       *
       * @param[in] handle A message handler
       *
       * Sets the message ahandler to tjhis listner to @p handle
       *
       * @post All message recive from this point will be processed by @p handle
       */
      void add_handler(handler &handle);

      /** @brief Start to listen
       *
       * Starts to listen to the queue and send messages to the different handler 
       * of this instance. This call is expected to be executed on 
       * a separate thread and will contiue its execution until it is stopped or 
       * an error occured.
       *
       * The listener routinely get new message from the queue and process them 
       * using the associated handler 
       *
       * @sa stop()
       * @sa is_running() const
       */ 
      void operator()() {
	if( !is_running() )
	  run();
      }
      /** @brief Stop execution
       *
       * Request this instance to stop its execution. The end of the execution 
       * may not be immediate as the execution thread should complete its 
       * reception cycle before completeing.  
       */
      void stop() {
	m_running = false;
      }
      /** @brief Test if running
       *
       * Check if the listner is still active 
       *
       * @retval true The listner is still running
       * @retval false The listner is not running or is going to end soon
       */
      bool is_running() const;
    private:
      void run();

      mutable TREX::utils::SharedVar<bool> m_running;
      queue_ref m_queue;
      handler *m_handler;
    }; // amqp::que::listener

    /** @brief AMQP message buffer
     *
     * This class is a simple amqp listener hanler that just stores the messages
     * received in a buffer that can be read later. It's implementation is thread 
     * safe and it should be able to be attached to multiple listeners merging 
     * all the message in its buffer. 
     *
     * @author Frederic Py <fpy@mbari.org>
     * @ingroup amqp
     * @sa listener
     */
    class msg_buffer :public listener::handler {
    public:
      /** @brief Constructor */
      msg_buffer() {}
      /** @brief Destructor */
      ~msg_buffer() {}

      /** @brief Check for no message
       *
       * Check if the message buffer is empty
       *
       * @retval true if the message buffer is empty
       * @retval false otherwise
       */
      bool empty() const;
      /** @brief Get next message
       *
       * gets the next message in the buffer. Messages are sorted from the 
       * oldest to the newest. So this call wil always give the oldest message 
       * received which ihas not been get yet.
       *
       * @pre the buffer is not empty
       *
       * @return A pointer to the oldest messasge in the buffer
       *
       * @post the returned message is not in the buffer anymore
       */
      SHARED_PTR<queue::message> pop();
      
    private:
      void handle(SHARED_PTR<queue::message> const &msg);

      typedef std::list<SHARED_PTR<queue::message> > queue_type;

      mutable TREX::utils::SharedVar<queue_type> m_queue;
    }; // mbari::amqp::msg_buffer
    
  } // mbari::amqp
} // mbari

#endif // H_AMQP_listener
