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
/** @file europa_helpers.hh
 * @brief some utilities to help manipulate europa construct
 *
 * This header decaler a set of tools that helps manipulate europa constructs 
 * within the trex europa plug-in
 *
 * @note some of the tools provided here are kind of "hacky" 
 * @author Frederic Py <fpy@mbari.org>
 * @ingroup europa
 */
#ifndef H_trex_europa_helpers
# define H_trex_europa_helpers

#include <string>

#include "../config.hh"

#include <PLASMA/Token.hh>
#include <PLASMA/ConstrainedVariable.hh>

#include <boost/noncopyable.hpp>

namespace TREX {
  namespace europa {

    class Assembly;

    /** @brief Implementation detail for the europa plug-in
     *
     * This namespce include classes that represent implementation details for 
     * the europa plug-in. These include helper tools, and other constructs that 
     * are not meant to be part of the public interface of this plug-in.
     *
     * @ingroup europa
     */
    namespace details {
      
      /** @brief Get parent token
       *
       * @param[in] var A europa variable
       *
       * Identifies the Token that is associated to the varaiable @p var
       *
       * @pre var is a Token attribute or a local variable from a model rule
       *
       * @return The token associated to @p var or the noId pointer if no such 
       * token exists
       * @ingroup europa
       * @author Frederic Py <fpy@mbari.org>
       */
      EUROPA::TokenId parent_token(EUROPA::ConstrainedVariableId const &var);
      
      std::ostream &var_print(std::ostream &out, EUROPA::ConstrainedVariableId const &var);
      
      /** @brief Extract Assembly
       *
       * @param[in] component A europa component
       *
       * @pre @p component is associated to an Assembly
       * @throw EuropaException @p component is not associated to an Assembly
       * @return The Assembly assocaited to @p component
       * @ingroup europa
       * @author Frederic Py <fpy@mbari.org>
       * @relates TREX::europa::Assembly
       */
      Assembly &assembly_of(EUROPA::EngineComponentId const &component);
      /** @brief Complete predicate name
       *
       * @param[in] obj A europa object 
       * @param[in[ predicate A short predicate name
       *
       * This method gives the full name of the predicate @p predicate of the object @p obj. 
       * The full name of this predicate is @p predicate prefixed by the class name of @p obj with a 
       * @c '.' in between. 
       *
       * @return The complete predicate name
       * @ingroup europa
       * @author Frederic Py <fpy@mbari.org>
       * @{
       */
      std::string predicate_name(EUROPA::ObjectId const &obj, std::string const &predicate);
      
      inline std::string predicate_name(EUROPA::ObjectId const &obj, EUROPA::LabelStr const &predicate) {
	return predicate_name(obj, predicate.toString());
      }
      /** @} */

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
       * @note This is a very "hacky" way to handle such limitation and should be used only when 
       *       we are sure that the new restriction of the base domains won't result in an 
       *       inconsistency. It is really important to check first that the domain is valid by 
       *       propagating constraints through the plan database.
       * @deprecated The usage of this class has been replaced by restrict_bases method which have less impact
       * on europa. 
       *
       * @sa restrict_bases(EUROPA::TokenId const &)
       *       
       * @author Frederic Py <fpy@mbari.org>
       * @ingroup europa
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
	
        /** @brief Access operator
         *
         * Allow direct acces to the Token attributes and methods
         * @sa operator*() const
         */
	EUROPA::Token *operator->() const;
        /** @brief Derefernce operator
         *
         * Allow direct acces to the Token 
         * @sa operator->() const
         */
	EUROPA::Token &operator* () const {
	  return *operator->();
	}

      private:
	EUROPA::TokenId m_token, m_active;
        EUROPA::TokenSet m_merged;
      }; // TREX::europa::details::scoped_split
      

      /** @brief Restrict base domain
       *
       * @param[in] tok A token
       * @param[in,out] var A variable 
       * @param[in] dom A domain
       *
       * @pre The type of @p dom is compatible with the type of @p var
       * @pre @p var belongs to the token @p tok 
       * @pre The intersection of the base domain of @p var and @p dom is not empty
       *
       * Restrict the base domain of @p var to @p dom. As opposed to the europa equivalent, 
       * this function allow to do such restriction even if @p tok is not Active or is Merged. 
       *
       * @post The domain of @p var is restricted  by @p dom
       * @ingroup europa
       * @author Frederic Py <fpy@mbari.org>
       */
      void restrict_base(EUROPA::TokenId const &tok, EUROPA::ConstrainedVariableId const &var, 
                         EUROPA::Domain const &dom);
      
      void restrict_bases(EUROPA::TokenId const &dest, EUROPA::TokenId const &src);

      /** @brief Restrict base domains 
       *
       * @param[in] tok A token
       *
       * Restrict the base domain of all attributes of @p tok to their last domain. This restriction is done 
       * also to the start, duration and end of the token
       * As opposed to the europa equivalent, this function allow to do such restriction 
       * even if @p tok is not Active or is Merged.
       *
       * @sa restrict_base(EUROPA::TokenId const &, EUROPA::COnstrainedVariableId const &, EUROPA::Domain const &)
       * @sa restrict_attributes(EUROPA::TokenId const &tok)
       * @ingroup europa
       * @author Frederic Py <fpy@mbari.org>
       */
      void restrict_bases(EUROPA::TokenId const &tok);
      /** @brief Restrict base domains 
       *
       * @param[in] tok A token
       *
       * Restrict the base domain of all attributes of @p tok to their last domain. The restriction does not apply to the token temporal attributes (stra, duration and end).
       * As opposed to the europa equivalent, this function allow to do such restriction 
       * even if @p tok is not Active or is Merged.
       *
       * @sa restrict_base(EUROPA::TokenId const &, EUROPA::COnstrainedVariableId const &, EUROPA::Domain const &)
       * @sa restrict_bases(EUROPA::TokenId const &tok)
       * @ingroup europa
       * @author Frederic Py <fpy@mbari.org>
       */
      void restrict_attributes(EUROPA::TokenId const &tok);
      void restrict_attributes(EUROPA::TokenId const &tok, EUROPA::TokenId const &other);
     
      /** @brief rejectable test functor
       *
       * This functor test if a token is rejectable or not. A rejectable token 
       * is a token with the base domain of its state including the REJECTED value.
       * @author Frederic Py <fpy@mbari.org>
       * @ingroup europa
       */
      struct is_rejectable {
        /** @brief test call operator
         *
         * @param[in] tok A token
         * @retval true if @p tok is rejectable
         * @retval false otherwise
         */
        bool operator()(EUROPA::TokenId const &tok) const;
      }; 
      
    } // TREX::europa::details
  } // TREX::europa
} // TREX

#endif // H_trex_europa_helpers
