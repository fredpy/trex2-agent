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
/** @file "trex/europa/TrexThreatDecisionPoint.hh"
 * @brief TREX specific threat decision point definition
 *
 * This header defines utilites for handling europa threat within a trex 
 * reactor. Default europa handling tends to prefer resolving he threat as 
 * early as possible which in trex ofte results on inserting a token in the 
 * past. Utilities defined here change this preferenc so token will preferrably 
 * be inserted after the execution frontier.
 * 
 * @ingroup europa
 * @author Frederic Py <fpy@mbari.org>
 */
#ifndef H_trex_europa_TrexThreatDecisionPoint
# define H_trex_europa_TrexThreatDecisionPoint

# include <trex/europa/config.hh>

// include plasma header as system files in order to disable warnings
# define TREX_PP_SYSTEM_FILE <PLASMA/ThreatDecisionPoint.hh>
# include <trex/europa/bits/system_header.hh>

namespace TREX {
  namespace europa {

    class Assembly;

    /** @brief A threat decision point that excludes insertion to the past
     *
     * This class implements a europa threat decision point that excludes decisions
     * which will insert the token necessarily into the past. It is useful to ensure 
     * that planning is made in the future and do not mistakenly insert a token in a 
     * past location (left open for example by the archiving of past tokens)
     *
     * @ingroup europa
     * @author Frderic Py
     */
    class TrexThreatDecisionPoint :public EUROPA::SOLVERS::ThreatDecisionPoint {
    public:
      /** @brief Constructor 
       *
       * @param[in] client A client to the plan database
       * @param[in] tokenToOrder the token to be ordered by this
       *              decision point
       * @param[in] configData Configuration information 
       * @param[in] rexplanation A string used to explain the decision
       *              source
       */
      TrexThreatDecisionPoint(EUROPA::DbClientId const &client,
			      EUROPA::TokenId const &tokenToOrder,
			      EUROPA::TiXmlElement const &configData,
			      EUROPA::LabelStr const &explanation = "trex");
      virtual ~TrexThreatDecisionPoint();
            
    protected:
      /** @brief initialize decision choices
       *
       * This method identifies all the valid choices available for
       * ordering this token.  It uses the same technique as the
       * default decision point but aafter that removes all the
       * insertion choices that would put the token to order
       * necessarily before the current tick -- dentified using the
       * global AGENT_CLOCK model variable
       *
       * @sa now()
       */
      void handleInitialize();
      
      /** @brief Current tick
       *
       * Extract the current tick date by extracting the lower bound
       * of the AGENT_CLOCK varaiable
       */
      EUROPA::eint now() const;
      
      virtual std::string toString() const;
      virtual std::string toShortString() const;


    private:
      EUROPA::ConstrainedVariableId m_clock;
    }; // TREX::europa::TrexThreatDecisionPoint

  } // TREX::europa
} // TREX

#endif // H_trex_europa_TrexThreatDecisionPoint
