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
#ifndef H_MessageHandler 
# define H_MessageHandler

# include "AMQP_queue.hh"
# include <trex/utils/XmlFactory.hh>
# include <trex/transaction/TeleoReactor.hh> 

namespace mbari {

  class DrifterTracker;

  /** @brief AMQP message handler for DrifterTracker reactor
   *
   * This is the abstract interface that defines AMQP message handler used with the
   * DrifterTracker reactors. Each message handler can bind theamqp queue of 
   * the reactor to a new exchange and route key and collect incoming messages
   * that they can manipulate to create new transactions in the DrifterTraker.
   *
   * @author Frederic Py
   * @relates DrifterTracker
   * @ingroup tracker
   */
  class MessageHandler {
  public:
    typedef TREX::transaction::TeleoReactor::duration_type duration_type; 
    typedef TREX::transaction::TeleoReactor::date_type     date_type;
    
    /** @brief XML parsing factory
     *
     * The factory used by DrifterTracker to create new MessageHandler
     */
    typedef TREX::utils::XmlFactory<MessageHandler, boost::shared_ptr<MessageHandler>, 
				    DrifterTracker *> factory;
    /** @brief XML factory argument
     * The type of the argeument provided by the factory while constructing  a 
     * new instance.
     */
    typedef factory::argument_type  xml_arg;
    
    /** @brief Exchange name
     *
     * @return the name of the amqp exchange this handler needs to be bound to
     */
    std::string const &exchange() const {
      return m_exchange;
    }
    /** @brief Route key
     *
     * @return the amqp routing key this handler needs to be bound to or an 
     *     empty string if no route key was provided.
     */
    std::string const &route() const {
      return m_route;
    }

    /** @brief XML constructor
     *
     * @param[in] arg an XML argument
     *
     * Constructor used to create new handlers from the factory. The expected 
     * XML structure is:
     * @code
     * < <HandlerType> exchange="<exchange>" routing="<routing>" />
     * @endcode 
     * With:
     * @li @p HandlerType is the type of the handler as defined in the factory
     * @li @p exchange indicate the amqp exchange to be bound to
     * @li @p routing indicate the aqmp trouting to ber bound to. If not 
     *   specified this routing will be set to "" meaning that any message from 
     *   the exchange are accepted. 
     */
    MessageHandler(xml_arg const &arg);
    /** @brief Destructor */
    virtual ~MessageHandler() {}

  protected:    
    /** @brief handle new message
     *
     * @param[in] message An amqp message
     *
     * This calback is called whenever a new message is received.
     *
     * @retval false this message is not relevant for this handler
     * @retval true message processed resulting on the DrifterTracker 
     *    not notifying any other handlers.
     */
    virtual bool handleMessage(amqp::queue::message &message) =0;
    /** @brief handle request
     *
     * @param[in] g A TREX goal
     *
     * Notifies the handler that a new goal @p g has been received by the 
     * reactor. Allowing this reactor to process the goal.
     *
     * @retval false this goal is not relevant for this handler
     * @retval true @p g processed resulting on the DrifterTracker 
     *    not notifying any other handlers.
     */
    virtual bool handleRequest(TREX::transaction::goal_id const &g) {
      return false;
    }
    /** @brief handler synchronization
     *
     * This method is called by the DrifterTracker whenever it synchronizes. 
     * It is often used by the handler to produce its new observations for TREX.
     *
     * @retval true synchronization successful
     * @retval false Failed to synchronize resulting on the immediate reactor 
     *      failure.
     */
    virtual bool synchronize() {
      return true;
    }

    /** @brief Current tick
     *
     * @return The current tick date in TREX
     */
    TREX::transaction::TICK now() const;
    /** @brief TREX tick to date
     *
     * @param[in] date A TREX tick
     *
     * convert @p date into a "real" date
     *
     * @return real date for @p date
     */    
    date_type tickToTime(TREX::transaction::TICK date) const;
    /** @brief TREX tick duration
     *
     * @retyurn the duration of a TREX tick (often in seconds)
     */
    duration_type tickDuration() const;

    /** @brief Declare new timeline
     *
     * @param[in] timeline Timelien name
     * @param[in] control goal flag
     *
     * Request the reactor to create the new Internal timeline @p timeline. 
     * If @p control is true then this timline will also accept goals.
     * 
     * @retval true if the creation was successful
     * @retval false otherwise
     */
    bool provide(std::string const &timeline, bool control=false);
    /** @biref post new observation
     *
     * @param[in] obs An observation
     *
     * Post the observation @p obs in the reactor
     *
     * @pre @p obs object has been provided by this reactor
     * @post @p obs is schedulled to be posted for this tick
     */
    void notify(TREX::transaction::Observation const &obs);

    std::string m_exchange;
    std::string m_route;

    TREX::utils::log::stream
    syslog(TREX::utils::Symbol const &kind = TREX::utils::log::null);
  private:
    DrifterTracker &m_tracker;

    friend class DrifterTracker;
  }; // mbari::MessageHandler

} // mbari

#endif // H_MessageHandler
