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

# include <trex/transaction/TeleoReactor.hh>


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
      /** @brief Constructor 
       *
       * @param[in] arg A xml node defintion
       *
       * Create a new instance based on the XML definition provided by @p arg
       * The XML structure is as follow:
       * @code 
       * <DoradoReactor name="<name>" log="<bool>" vcs_host="<address>" 
       *                vcs_port="<port>" port="<port>" imitial_plan="<file>"
      *                 config="<file>" >
       *   <Timeline name="<name>" sequential="<bool>" alias="<name>" />
       *   ...
       * </DoradoReactor>
       * @endcode
       * Where
       * @li @p name is the name of the reactor
       * @li @p log indicates if the reactor logs its transaction (default is true)
       * @li @p vcs_host is the host address where vcsServer is running
       * @li @p vcs_port is the TCP port number vcsServer is listening (default is 8004)
       * @li @p port is the UDP post number this reactor listens for state updates 
       *    (default is 8002)
       * @li @p initial_plan The initial stack of behaviors to send to the vcs (default is
       *     auv_init.cfg)
       * @li @p config is an optional poitner to another config file that will help 
       *   populate this node. For example where all the timelines and port are specified.
       *
       * The timeline sub-tags indicates the behaviors provided by this reactor. Each 
       * of them have the following attributes
       * @li name The name of the timeline
       * @li sequential an optional flag to indicate that the corresponding behavior 
       *     is sequential or not (default is true)
       * @li alias real name of the behavior for vcs (default is the same value as @p name)
       *
       * @note This reactor does not havw latency or lookahead parameters. 
       * Indeed they are hard-coded  as repectively 0 and 1.
       */
      DoradoReactor(transaction::TeleoReactor::xml_arg_type arg);
      ~DoradoReactor();
      
    private:
      void handleInit();
      void handleTickStart();
      bool synchronize();
      
      void handleRequest(transaction::goal_id const &g);
      void handleRecall(transaction::goal_id const &g);
            
      void initial_plan(boost::filesystem::path const &file);
      void started(transaction::goal_id g);
      void completed(transaction::goal_id g);
      
      
      std::set<utils::Symbol> m_sequential;
      
      bool is_sequential(utils::Symbol const &name) const {
        return m_sequential.end()!=m_sequential.find(name);
      }
      
      class msg_api;

      
      SHARED_PTR<msg_api> m_api;
      size_t m_seq_count;
      SHARED_PTR<transaction::Observation> m_last_state;
      transaction::TICK m_last_packet;
      
      static utils::Symbol const s_inactive;
      static utils::Symbol const s_depth_envelope;
      static utils::Symbol const s_sp_timeline;
            
    }; // TREX::mbari::DoradoReactor
    
  } // TREX::mbari
} // TREX

#endif // H_TREX_mbari_DoradoReactor