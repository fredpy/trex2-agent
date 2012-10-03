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
#ifndef H_TREX_mbari_DoradoReactor
# define H_TREX_mbari_DoradoReactor

# include "StatePacket.h"

# include <trex/transaction/TeleoReactor.hh>

# include <boost/asio.hpp>


namespace TREX {
  namespace mbari {
       
    /** @brief conection interafec with dorado software
     *
     * This class implements the conection to dorado LayeredControl and 
     * StatePublisher through socket interfaces.
     * It allow trex to interract with this component by inserting new 
     * behaviors and receiving status updates that are then converted into 
     * timelines. The code here is very similar to what we use on the initial 
     * version of TREX anfd jsut include adaptations to the new version of the 
     * architecture.
     * 
     * @author Frederic Py <fpy@mbari.org>
     * @ingroup mbari
     */
    class DoradoReactor :public transaction::TeleoReactor {
    public:
      using transaction::TeleoReactor::xml_arg_type;
      
      DoradoReactor(xml_arg_type &arg);
      ~DoradoReactor();
      
    private:
      void handleInit();
      void handleTickStart();
      bool synchronize();
      
      void handleRequest(transaction::goal_id const &g);
      void handleRecall(transaction::goal_id const &g);
            
      static boost::asio::io_service s_io_service;
      static utils::Symbol const     s_depth_enveloppe;
      static utils::Symbol const     s_sp_timeline;
      
      /** @brief execution temporal uncertainty
       *
       * This value includes the temporal uncertainty of execution observation 
       * of behavior. It is set to 2 as it includes the worst case scenario of 
       * the behavior starting and ending right between two ticks resulting on 
       * 2 extra ticks of duration.
       *
       * As a result thois reactor decrease the duration deadline sent to the 
       * vehicle by this amount of time when possibel in order to hide to trex 
       * this possible uncertainty.
       */
      static transaction::IntegerDomain::bound const s_temporal_uncertainty;
      
      boost::asio::ip::udp::socket m_sp_socket;  // server
      boost::asio::deadline_timer  m_sp_timer;

      /** Collect messages from StatePublisher
       *
       * @param[in] timeout maximum blocking time
       *
       * This methods listen to the @c m_sp_socket socket for new incoming 
       * messages for a mximum time of @p timeout. Inf new messages are 
       * received before this time, it will process them depending on their 
       * type 
       *
       * @retval true A message was recieved and succesfully processed
       * @retval false no message received before @p timeout
       *
       * @sa get_behavior()
       * @sa get_satepacket(uint16_t)
       */
      bool get_sp_updates(boost::posix_time::milliseconds const &timeout);
      /** @brief Receive and parse behavior state update
       *
       * This method is used internally in order to process behavior state update 
       * messages and reflect thenm in the corresponding timelines.        
       */
      void get_behavior();
      /** @brief Receive state packet updates
       *
       * This method is called when a statpacket message is recived. It checks 
       * basic vvalidity of the packet and indicastes that a new statepacket has 
       * been received for later processing when this message is valid.
       */
      void get_statepacket(uint16_t size);
      
      boost::asio::ip::tcp::endpoint m_vcs_server;
      boost::asio::ip::tcp::socket   m_vcs_socket; // client
      
      /** @brief Send string to VCS server
       *
       * @param[in] msg A string message
       *
       * Send @p msg to the VCS server this reactor is connected to.
       */
      void send_string(std::string const &msg);
      /** @brief insert new behavior
       *
       * @param[in] g A behavior goal request 
       *
       * This methods create and send a new behavior insertion message
       * based on the goal @p g. It is called internally by the reactor 
       * depending on the type of the behavior @p g is related to.
       */
      void send_request(transaction::goal_id const &g);
      
      boost::filesystem::path    m_init_plan;
      bool                       m_layered_control_ready, m_pinged;
      std::map< int32_t, std::pair<transaction::goal_id, bool> > m_sent_cmd;
      
      XDR    m_xdr_stream;
      char  *m_sp_buff;
      char  *m_xdr_buff;
      size_t m_xdr_buff_size;
      bool   m_fresh_xdr;
      
      std::map<utils::Symbol, std::pair<bool, utils::Symbol> > m_behaviors;
      std::map<utils::Symbol, std::list<transaction::Observation> > m_queued_obs;
      std::list<transaction::goal_id> m_pending;
      
      /** @brief Queue behaviors status observation for posting
       *
       * @param[in] obs A behavior state observation
       *
       * This method register a behavior state update as pending. 
       * The motivation to use this instead of a regular postObservation is to 
       * handle the case where a behavior has multiple state chages within a tick.
       * As the planner need to observae all the behviorsbeing executed the 
       * system makes sure that such observationswill be posted in sequential 
       * ticks. This increase arbitrarily the duration of these bahviors but at 
       * least allow the plan to remain correct.
       */
      void queue_obs(transaction::Observation const &obs);
      /** @brief Process pending sequential behaviors requests.
       *
       * This method handle the dispatching of pending sequential behaviors. 
       * Indeed as the system can execute only one sequential behavior at once
       * the goal requested are queued and disspatch as the reactor observe 
       * feedback that indicates that the next behavior can be dispatched without 
       * breaking the plan.
       */
      void process_pendings();
      
      size_t next_id(); 
      size_t m_id, m_behavior_count, m_sequential_count;
      bool m_sequential_execute;
      
    }; // TREX::mbari::DoradoReactor
    
  } // TREX::mbari
} // TREX

#endif // H_TREX_mbari_DoradoReactor