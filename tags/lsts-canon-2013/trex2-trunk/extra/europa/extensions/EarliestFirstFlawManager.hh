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
#ifndef H_trex_europa_GreedyFlawManager
# define H_trex_europa_GreedyFlawManager

# include <trex/europa/config.hh>

// include plasma header as system files in order to disable warnings
# define TREX_PP_SYSTEM_FILE <PLASMA/OpenConditionManager.hh>
# include <trex/europa/bits/system_header.hh>

namespace TREX {
  namespace europa {

    /** @brief earliest first open condition manager
     *
     * This class implement an open condition manager -- which deals with token 
     * to be inserted to the plan and the order in which they need to be evaluated
     * -- that will eveluates the earliest tokens first.
     *
     * It orders tokens based on their start time by picking first the one that 
     * should start the earlier.
     *
     * @author Frederic Py <fpy@mbari.org>
     * @ingroup europa
     */
    class EarliestFirstFlawManager :public EUROPA::SOLVERS::OpenConditionManager {
    public:
      /** @brief Constructor
       *
       * @param[in] cfg XML configuration
       */
      EarliestFirstFlawManager(EUROPA::TiXmlElement const &cfg);
      
    private:
      /** @brief Flaw sorter
       *
       * @param[in] a An entity
       * @param[in] b An entity
       * @param[out] explanation A string
       *
       * This method allow the solver to sort the open condition flaws based on 
       * their start and potetnially end domains. It indicates whther @p a should 
       * be evaluated before @p b and give a textual explanation in @p explanation
       *
       * @retval true if @p a should be evaluated before @p b
       * @retval false otherwise
       */
      bool betterThan(EUROPA::EntityId const &a, EUROPA::EntityId const &b,
		      EUROPA::LabelStr &explanation);
      
    };


  } // TREX::europa
} // TREX

#endif // H_trex_europa_GreedyFlawManager
