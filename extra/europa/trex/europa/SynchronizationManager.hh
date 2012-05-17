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
#ifndef H_trex_europa_SynchronizationManager
# define H_trex_europa_SynchronizationManager

# include "config.hh"

# include <PLASMA/FlawManager.hh>

namespace TREX {
  namespace europa {

    class Assembly;

    namespace details {
      class UpdateFlawIterator;
    } // TREX::europa::details
    
    /** @brief TREX synchronization flaws manager
     * 
     * A class that identifies and manage the new flaws specific to a reactor
     * synchronization.
     *
     * Indeed synchronization introduce a new kind of flaw that requires the 
     * reactor to identifies its state at the current tick. This implies that 
     * for all its internal timelines it requires to resolve what is the token 
     * that will hold for this tick (ie ending at least 1 tick in the future). 
     * 
     * This class will manage this new kind of flaw and will ba automatically 
     * injected by the reactor for its solver dedicated to synchronization.
     *
     * @note This flaw manager is automatically added to the synchronizer of 
     *       an Assembly. 
     *    
     * @note In past versions we used to do this by hand which presented many 
     *       issues:
     *       @li trex was messing directly with the plan database which is painfull 
     *           to implement at best and often resulted on bugs diffcult to 
     *           understand/capture as we were tapping in non public europa interfaces
     *       @li these decision where not embedded in any decision stack which made 
     *           it difficult to relax them. In fact we used a griddy approach whihc 
     *           assumed that the first possible choice for a timeline will not impact
     *           the choices for others. This kind of worked in practice (for the 
     *           domains we were using) but can easily be demonstrated as an invalid 
     *           assumption in the general case (just need to have a model instance 
     *           with two internal timelines having interdependent state).
     * This new way to handle synchronization is cleaner both theoretically 
     * (we can now backtrack on these choices) and practically (connection with 
     * europa is less hacky).
     *
     * @author Frederic Py <fpy@mbari.org>
     * @ingroup europa
     * @relates Assembly
     */
    class SynchronizationManager :public EUROPA::SOLVERS::FlawManager {
    public:
      /** @brief Constructor
       * @param[in] cfg The XML config 
       */
      SynchronizationManager(EUROPA::TiXmlElement const &cfg);
      /** @brief Destructor */
      ~SynchronizationManager() {}
      
      /** @brief Entity static Match
       *
       * @param[in] entity A europa entity
       *
       * @retval true if @p entity is A CurrentState
       * @retval false otherwise
       */
      bool staticMatch(EUROPA::EntityId const &entity);
      /** @brief Entity synamic Match
       *
       * @param[in] entity A europa entity
       *
       * @retval true if @p entity is A CurrentState and its current state 
       *         is not yet identified
       * @retval false otherwise
       */
      bool dynamicMatch(EUROPA::EntityId const &entity);

      /** @brief Decisions iterator
       *
       * @return An iterator through all the possible solution for this flaw
       */
      EUROPA::IteratorId createIterator();
      
      /** @brief String display
       *
       * @param[in] entity A flawed entity
       *
       * @pre @p entity is a CurrentState instance
       *
       * @return A string describing the flaw for @p entity
       */
      std::string toString(EUROPA::EntityId const &entity) const;
      
    private:
      /** @brief Initialization
       *
       * Complete this handler initialization
       *
       * @pre The plan database for this manager is created by an Assembly 
       * @throw EuropaException THe plan database is not associated to an Assembly
       */
      void handleInitialize();

      Assembly *m_assembly;

      friend class TREX::europa::details::UpdateFlawIterator;
    }; // TREX::europa::SynchronizationManager
    
  } // TREX::europa  
} // TREX

/** @brief Name of the synchronization manager
 *
 * The name of the synchronization manager as used on solver configuration file
 * @relates TREX::europa::SynchronizationManager
 */
# define TREX_SYNCH_MGR      TrexSynchronizer
/** @brief Name of the synchronization flaws handler
 *
 * The name of the synchronization handler as used on solver configuration file
 * @relates TREX::europa::details::CurrentState::DecisionPoint
 */
# define TREX_SYNCH_HANDLER  CurrentStateHandler

#endif // H_trex_europa_SynchronizationManager
