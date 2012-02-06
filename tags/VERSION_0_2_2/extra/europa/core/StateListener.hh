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
#ifndef H_trex_europa_StateListener
# define H_trex_europa_StateListener

# include <PLASMA/Token.hh>

namespace TREX {
  namespace europa {

    namespace details {
      class CurrentState;
    } // TREX::europa::details

    /** @brief Listen to state updates on Internal timelines
     * 
     * This abstract interface listen to new state updates from 
     * deliberation on an Internal timeline. It provides callbacks
     * that notifies on updates on the state updates (either a new 
     * token created or the refinement on the previous token) on all 
     * the timelines the listener is attached to.
     *
     * This listener is especially usefull in order for the reactor to
     * efficiently identifies its internal state updates and post them 
     * as observations.
     *
     * @author Frederic Py <fpy@mbari.org>
     */
    class StateListener {
    public:
      /** @brief Destructor */
      virtual ~StateListener() {}

    protected:
      /** @brief Constructor */ 
      StateListener() {}

      /** @brief New token started 
       * 
       * @param[in] token The newly started token
       *
       * This method is called to notify that the token @p token
       * has been started for this tick
       */
      virtual void newState(EUROPA::TokenId const &token) =0;
      /** @brief Former token attributes refined/restricted
       * 
       * @param[in] token The token refined
       *
       * This method is called to notify that the token @p token
       * has been refined.
       *
       * @note As of today the main refinement is mostly the duration 
       *       and end of the token being extended to include the current 
       *       tick.
       * @note It is expected that @p token is identical (or at least 
       *       compatible to the last token passed as argument to @c newState
       *       callback
       */
      virtual void refinedState(EUROPA::TokenId const &token) =0;
      
      friend class TREX::europa::details::CurrentState;
    }; // TREX::europa::StateListener

  } // TREX::europa
} // TREX 

#endif // H_trex_europa_StateListener
