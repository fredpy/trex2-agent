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
#ifndef H_trex_europa_DeliberationFilter 
# define H_trex_europa_DeliberationFilter

# include "config.hh"

# include <PLASMA/Filters.hh>

namespace TREX {
  namespace europa {
    
    class Assembly;

    /** @brief T-REX token filter
     *
     * A europa token filter that is connected to an Assembly in order 
     * to gather data from it during its filtering.
     *
     * @author Frederic Py <fpy@mbari.org>
     * @ingroup europa
     */
    class TokenFilter :public EUROPA::SOLVERS::FlawFilter {
    public:
      /** @brief Constructor
       * @param[in] cfg An Xml configuration tag
       */
      TokenFilter(EUROPA::TiXmlElement const &cfg);
      /** @brief Destructor */
      virtual ~TokenFilter() {}
      
    protected:
      /** @brief Check for Assembly
       *
       * Identifies if the Assembly for this filter has been identified
       *
       * @retval true If this filter is attached to an Assembly
       * @retval false otherwise
       */
      bool have_assembly() const {
	return NULL!=m_assembly;
      }
      /** @brief Get Assembly
       *
       * Gives the assembly associated to this reactor
       *
       * @pre The assembly have been identified
       * @throw EuropaException No Assermbly identified for this filter
       * @return The assembly associated to this filter
       *
       * @sa has_assembly() const
       */
      Assembly const &assembly() const;

      /** @brief Token filter specialization
       *
       * @param[in] token A token
       *
       * This method can be specialized to filter specific tokens
       *
       * @retval true if @p token should be filtered out
       * @retval false otherwise
       */
      virtual bool doTest(EUROPA::TokenId const &token) =0;
      
    private:
      /** @brief Check for token scope
       * @param[in] token A token
       *
       * Checks if @p token is within the general Assembly planning scope
       *
       * @retval true if @p token is explicitely ignored by the assembly
       * @retval true if @p token necessarily ends before the mission initial tick
       * @retval true if @p token necessarily starts beyond the mission final tick 
       * @retval false otherwise
       */
      bool tokenCheck(EUROPA::TokenId const &token);
      /** @brief Filter test
       *
       * @param[in] entity An entity
       *
       * test if @p entity should be filtered out or not.
       *
       * @retval false if @p entity is not a Token or is outside the mission scope
       * @retval doTest(entity) otherwise
       *
       * @sa doTest(EUROPA::Token const &)
       * @sa tokenCheck(EUROPA::Token const &)
       */
      bool test(EUROPA::EntityId const &entity);
      /** @brief Attach to an Assembly
       *
       * @param[in] component An europa component
       *
       * Attach this filter to the Assembly @p component only if this filter is not 
       * attached to an Assembly yet.
       *
       * @pre @p component is an Assembly
       * @throw EuropaException @p component is not an Assembly
       */
      void set_assembly(EUROPA::EngineComponentId const &component);

      Assembly *m_assembly;
    }; // TREX::europa::TokenFilter

    /** @brief Reactor deliberation scope
     *
     * This class is used by Assembluy in order to focus the planning to 
     * only the tokens within the planning horizon of this reactor. It is 
     * automatically added to the planner configuration by the Assembly.
     *
     * @author Frederic Py <fpy@mbari.org>
     * @relates Assembly
     * @sa SynchronizationScope
     * @ingroup europa
     */
    class DeliberationScope :public TokenFilter {
    public:
      /** @brief Constructor
       * @param[in] cfg Xml configuration
       */
      DeliberationScope(EUROPA::TiXmlElement const &cfg)
	:TokenFilter(cfg) {}
      /** @brief Destructor
       */
      ~DeliberationScope() {}
      
    private:
      /** @brief Filter test
       *
       * @param[in] tok A token 
       * @retval true if @p tok necessarily ends before current tick or starts 
       * after the Assembly planning scope
       * @retval false otherwise
       */
      bool doTest(EUROPA::TokenId const &tok);
    }; // TREX::europa::DeliberationScope

    /** @brief synchronization deliberation scope
     *
     * This class is used by Assembly in order to focus the planning probalem to 
     * only tokens that can overlap the current tick. It is 
     * automatically added to the synchronizer configuration by the Assembly.
     *
     * @author Frederic Py <fpy@mbari.org>
     * @relates Assembly
     * @sa DeliberationScope
     * @ingroup europa
     */
    class SynchronizationScope :public TokenFilter {
    public:
      /** @brief Constructor
       * @param[in] cfg Xml configuration
       */
      SynchronizationScope(EUROPA::TiXmlElement const &cfg)
	:TokenFilter(cfg) {}
      /** @brief Destructor
       */
      ~SynchronizationScope() {}
      
    private:
      /** @brief Filter test
       *
       * @param[in] tok A token 
       * @retval true if @p tok necessarily ends before current tick or starts 
       * after the next tick
       * @retval false otherwise
       */
      bool doTest(EUROPA::TokenId const &tok);
    }; // TREX::europa::SynchronizationScope

  } // europa
} // TREX

/** @brief Name of the Deliberation scope filter
 * 
 * The name used in XML configuration for the DeliberationScope filter
 *
 * @relates TREX::europa::DeliberationScope
 * @ingroup europa
 */
# define TREX_DELIB_FILT TrexDeliberationScope
/** @brief Name of the Synchronization scope filter
 * 
 * The name used in XML configuration for the SynchronizationScope filter
 *
 * @relates TREX::europa::SynchronizationScope
 * @ingroup europa
 */
# define TREX_SYNCH_FILT TrexSynchronizationScope

#endif // H_trex_europa_DeliberationFilter
