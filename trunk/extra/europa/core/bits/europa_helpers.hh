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
#ifndef H_trex_europa_helpers
# define H_trex_europa_helpers

#include <string>

#include "../../config.hh"

#include <PLASMA/Token.hh>
#include <PLASMA/ConstrainedVariable.hh>

#include <boost/noncopyable.hpp>

namespace TREX {
  namespace europa {

    class Assembly;

    namespace details {
      
      EUROPA::TokenId parent_token(EUROPA::ConstrainedVariableId const &var);
      Assembly &assembly_of(EUROPA::EngineComponentId const &component);
      std::string predicate_name(EUROPA::ObjectId const &obj, std::string const &predicate);

      inline std::string predicate_name(EUROPA::ObjectId const &obj, EUROPA::LabelStr const &predicate) {
	return predicate_name(obj, predicate.toString());
      }

      /** @brief Scoped token split
       *
       * This class allows to split a merged token for its existence scope and merge 
       * it back in destruction.
       * 
       * The scopped split is neeced by trex in order to restrict the base domain of the
       * tokens it creates during synchronization that will maintain the state of the 
       * reactor. Indeed, europa does not allow to restrict the base domain of the attributes 
       * of a merged token (only active or inactive tokens can be minpulated this way). This 
       * class avoid such limitation by splitting temproarily a merged token allowing in turn
       * to restrict its base domains.
       *
       * @note This is a very "hacky" way to hendle such limitation and should be used only when 
       *       we are sure that the new restriction of the base domains won't result in an 
       *       inconsistency. It is really important to check first that the domain is valid by 
       *       propagating constraints through the plan database.
       *       
       * @author Frederic Py <fpy@mbari.org>
       */
      class scoped_split :boost::noncopyable {
      public:
	/** @brief Constructor
	 * 
	 * @param[in] token a token
	 * 
	 * Associates this instance to the token @p token and split this token if it was
	 * merged while storing the token it eas merged to
	 * 
	 * @post @p token is not merged
	 */
	scoped_split(EUROPA::TokenId const &token);
	/** @brief Destructor
	 * 
	 * Destroy this instance while merging the associated token back if it was split during 
	 * construction.
	 */
	~scoped_split();

	/** @brief Get active token
	 * 
	 * This method give access to the "active" token associated to the token we manipulate 
	 * before the construction. 
	 * 
	 * The returned value will differ from the token we maniu[ulated if and only if the 
	 * associated token was split during construction.
	 *
	 * @note It is possible that the "active" token returned is not active. This will be
	 *       true if the associated token was neither active nor merged before construction
	 */
	EUROPA::TokenId active() const;
	
	EUROPA::Token *operator->() const;
	EUROPA::Token &operator* () const {
	  return *operator->();
	}

      private:
	EUROPA::TokenId m_token, m_active;
        EUROPA::TokenSet m_merged;
      }; // TREX::europa::details::scoped_split
      

      void restrict_base(EUROPA::TokenId const &tok, EUROPA::ConstrainedVariableId const &var, 
                         EUROPA::Domain const &dom);
      void restrict_bases(EUROPA::TokenId const &tok);
      
    } // TREX::europa::details
  } // TREX::europa
} // TREX

#endif // H_trex_europa_helpers
