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
#ifndef H_DrifterTracker 
# define H_DrifterTracker

# include <map>

# include "AMQP_listener.hh"
# include "MessageHandler.hh"

# include <trex/transaction/TeleoReactor.hh>

namespace mbari {

  /** @brief Track assets using MBARI amqp server messages
   *
   * This class is used for our shore side T-REX agent in order to track assets 
   * position and TREX controlled AUV feedback through message collected on our 
   * amqp server.
   *
   * It provides dynamically new timelines when a new asset gives its position 
   * and estimate assets position based on the computed speed between the last 
   * two updates. 
   *
   * At the same time it provides statically few timelines that indicates if our 
   * TREX controlled AUV is currently executing a goal or waiting for new 
   * instructions. 
   *
   * By its nature this reactor timelines (all Internal) is very dynamic and 
   * will grow as it identifies a new assets.
   *
   * @note The overal interntion of this reactor is more to display information
   * on asset and not to provide a control interface. Therefore most of (if not 
   * all) the timelines provided by this reactor do not accept goal.
   *
   * @sa MessageHandler
   * 
   * @author Frederic Py <fpy@mbari.org>
   * @ingroup tracker
   */ 
  class DrifterTracker :public TREX::transaction::TeleoReactor {
  public:
    /** @brief Constructor 
     * @param[in] arg factory XML argment 
     *
     * Creates a new instance of this reactor. The XML format for this reactor 
     * is as follow:
     * @code 
     * <DrifterTracker name="<reactor_name>" lookahead="<look_ahead>" latency="<latency>"
     *                 log="<log_flag>">
     *   [ tracker_definition ]
     * </DrifterTracker>
     * @endcode 
     * Where:
     * @li @p reactor_name is the name of the reactor
     * @li @p look_ahead is the look_ahead of the reactor (recommended to be 
     *     @c 0 or @c 1 depending if you want to receive goals or not)
     * @li @p latency is the reactor latency (often @c 0)
     * @li @p log_flag indicates if this reactor should log its transaction 
     *     (recommended to be @c 1 so you can replay this mission later)
     * @li @p tracker_defintion is the XML definitions of all the MessageHandler
     *     you want to use within this reactor to populate it.
     *
     * @note To have a usefull reactor you need at least to have one MessageHandler 
     * associated to i
     * @bug At this time, the amqp connection is hard-coded to connect to mbari 
     * amqp server. 
     * @bug Due to the nature of its implementation you cannot have multiple 
     * DrifterTracker connecting to the same server.
     */
    DrifterTracker(TREX::transaction::TeleoReactor::xml_arg_type arg);
    /** @brief Destructor */
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
    UNIQ_PTR<amqp::listener> m_listener;
    UNIQ_PTR<boost::thread>  m_thread;

    typedef std::multimap<std::string, 
			  SHARED_PTR<MessageHandler> > handle_map;
    handle_map m_message_handlers;
    std::multimap<std::string, MessageHandler *> m_goal_handlers;

    TREX::utils::Symbol            m_trexMsg;

    TREX::utils::SingletonUse<MessageHandler::factory> m_msg_factory;
        
    void trex_msg(amqp::queue::message const &msg);

    friend class MessageHandler;
  }; // mbari::DrifterTracker

}

#endif // H_DrifterTracker
