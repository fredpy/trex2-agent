/** @file "LightSwitch.hh"
 *  @brief lightswitch plug-in utilities
 *
 *  This file defines the utilities provided by the lightswitch plug-in. 
 *  
 *  @ingroup lightswitch
 *  @author Frederic Py <fpy@mbari.org>
 */
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
#ifndef H_LightSwitch
# define H_LightSwitch

# include <trex/transaction/TeleoReactor.hh>

namespace TREX {
  /** @brief lightswitch plug-in utilities
   *
   * This namespace embeds the classes and functions provided by the
   * lightswitch plug-in
   * 
   * @ingroup lightswitch
   *
   * @author Frederic Py <fpy@mbari.org>
   */
  namespace lightswitch {

    /** @brief Light reactor definition
     *
     * This class implements a very simple reactor that emulates a light
     * switch. It provides a @c light timeline that can be either on
     * (@c Light.On) or off (@c Light.Off)
     *
     * @author Frederic Py <fpy@mbari.org>
     * @ingroup lightswitch
     */
    class Light :public TREX::transaction::TeleoReactor {
    public:
      /** @brief XML constructor
       * @param arg An XML node definition
       *
       * This constructor is called to generate a Light reactor
       * based on an XML node. The expected XML format is the following:
       * @code
       * <Light name="<name>" latency="<int>" lookahead="<int>" state="<bool>" 
       *        verbose="<bool>" />
       * @endcode
       * Where :
       * @li @c state is a boolean value indicating which is the initial
       *     state of the @c light timeline (default is @c true)
       * @li @c verbose is a boolean used to indicate whther we should 
       *     repeat the light state at every tick or not (default is @c false)
       */
      Light(TREX::transaction::TeleoReactor::xml_arg_type arg);
      /** @brief Destructor */
      ~Light();      

    private:
      bool synchronize();
      void handleRequest(TREX::transaction::goal_id const &g);
      void handleRecall(TREX::transaction::goal_id const &g);

      /** @brief State of the timeline */
      bool m_on, m_verbose;
      TREX::transaction::TICK m_nextSwitch;
      /** @brief Is the state already posted as observation ? */
      bool m_firstTick;

      void setValue(bool val);
      
      std::list<TREX::transaction::goal_id> m_pending;
      UNIQ_PTR<TREX::transaction::Observation> m_light_state;

      /** @brief Name of the predicate on */
      static TREX::utils::symbol const onPred;
      /** @brief Name of the predicate off */
      static TREX::utils::symbol const offPred;
      /** @brief Name of the predicate up */
      static TREX::utils::symbol const upPred;
      /** @brief Name of the predicate down */
      static TREX::utils::symbol const downPred;
      /** @brief Name of the predicate broken */
      static TREX::utils::symbol const brokenPred;

      /** @brief Name of the timeline for light */
      static TREX::utils::symbol const lightObj;
      /** @brief Name of the timeline for light */
      static TREX::utils::symbol const switchObj;
    };

  }
}

#endif // H_LightSwitch
