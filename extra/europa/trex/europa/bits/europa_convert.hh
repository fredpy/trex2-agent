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
 *     notice, this list of conditeions and the following disclaimer.
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
#ifndef H_trex_europa_convert
# define H_trex_europa_convert

# include <trex/europa/config.hh> // set the flags for europa

// include plasma header as system files in order to disable warnings
# define TREX_PP_SYSTEM_FILE <PLASMA/Domain.hh>
# include <trex/europa/bits/system_header.hh>
# define TREX_PP_SYSTEM_FILE <PLASMA/DataType.hh>
# include <trex/europa/bits/system_header.hh>

# include <trex/domain/DomainVisitor.hh>

namespace TREX {
  namespace europa {
    namespace details {
      /** @brief Europa to TREX domain conversion
       *
       * @param[in] dom A europa domain
       *
       * Converts the europa domain @p dom into the equivalent TREX domain
       *
       * @pre @p dom is a domain tyope supported by TREX
       *
       * @return The TREX representation of  @p dom
       *
       * @throw EuropaException Unable to matrch the type of @p dom with a
       *        TREX domain type
       *
       * @author Frederic Py <fpy@mbari.org>
       * @sa europa_domain
       * @ingroup europa
       */
      TREX::transaction::DomainBase *trex_domain(EUROPA::Domain const &dom);
      
      /** @brief TREX to Europa domain conversion
       *
       * This isitor class allows to convert a TREX domain into the equivalent
       * Europa representation.
       *
       * More accurately it restricts a europa domain using a TREX domain. The
       * Europa domain to be restircted can be either a new instance, the copy
       * of an already existing domain, or the exisitng domain itself.
       *
       * @author Frederic Py <fpy@mbari.org>
       * @sa trex_domain(EUROPA::Domain const &)
       * @ingroup europa
       */
      class europa_domain :public TREX::transaction::DomainVisitor {
      public:
        /** @brief Constructor
         *
         * @param[in] type An europa type
         *
         * Create a new instance starting with the base domain of @p type
         * The base domain corresponding to the mpost relaxed domain for
         * @p type, an instance created with this constructor is often used
         * to do a simple conversion for a TREX domain to its europa counterpart.
         *
         * @post The new instance is temporary which means that the europa domain
         * will be deleted with this instance
         */
        explicit europa_domain(EUROPA::DataTypeId const &type)
        :m_temporary(true), m_dom(type->baseDomain().copy()), m_type(type) {}
        /** @brief Constructor
         *
         * @param[in] dom An europa domain
         *
         * Create a new instance starting with a copy of @p dom. As this instsnce
         * manipulates a copy @p dom won't be modified even though this instaance
         * visits domains.
         *
         * @post The new instance is temporary which means that the europa domain
         * will be deleted with this instance
         */
        explicit europa_domain(EUROPA::Domain const &dom)
        :m_temporary(true), m_dom(dom.copy()), m_type(dom.getDataType()) {}
        /** @brief Constructor
         *
         * @param[in] dom A pointer to en europa domain
         *
         * @pre dom is a valid pointer to an existing europa domain instance
         *
         * Create a new instance that will modify directly the domain pointed by
         * @p dom. As a result any trex domain visited will directly modifies this
         * domain. Such instance is usefull when a trex domain is used to restrict
         * the domain of a varaiable already exisiting in the plan database (such
         * as for example a token attribute).
         *
         * @post The new insatnce is @e not temporary and the referred domain will
         *     not be deleted on destruction
         */
        explicit europa_domain(EUROPA::Domain *dom)
        :m_temporary(false), m_dom(dom), m_type(dom->getDataType()) {}
        /** @brief Copy constructor
         *
         * @param[in] other Another instance
         *
         * Create a new insatnce that duplicates other
         *
         * @note If @p other is temporary this constructor will crreate  a copy of
         * the domain managed by @p other. This is required in order to avoid
         * memory access issues. As a result the copy a temporary instance won't
         * alter the same domain but just a copy.
         *
         * @post the temporary state of the new instance is the same oas @p other
         */
        europa_domain(europa_domain const &other)
        :m_temporary(other.m_temporary), m_dom(other.m_dom),
        m_type(other.m_type) {
          if( m_temporary )
            // Create a copy of temporary domains to avoid double delete
            m_dom = m_dom->copy();
        }
        /** @brief destructor
         *
         * If the instance was temporary theb n the asscited domain is deleted.
         */
        ~europa_domain() {
          if( m_temporary )
            delete m_dom;
        }
        
        /** @brief Conversion visitors
         * @param[in] dom A T-REX domain
         *
         * Attempt to restrict the underlying europa domain with @p dom
         *
         * @pre @p dom type is compatible with the type of the europa domain
         *
         * @throw TREX::transaction::DomainAccess The types of @p dom is not
         * compatible with the type of the europa domain
         * @throw TREX::transaction::EmptyDomain The intersection of @p dom and
         * the europa domain is empty
         * @{
         */
        void visit(TREX::transaction::BasicEnumerated const *dom);
        void visit(TREX::transaction::BasicInterval const *dom);
        void visit(TREX::transaction::DomainBase const *dom, bool);
        /** @} */
        
        /** @brief Get europa domain
         *
         * @return the reuropa domain amnipulated by this visitor
         * @{
         */
        EUROPA::Domain const &domain() const {
          return *m_dom;
        }
        EUROPA::Domain &domain() {
          return *m_dom;
        }
        /** @} */
      private:
        /** @brief temporary flag
         *
         * This flag is used in order to identify if the asoacited domain
         * should be deleted on detsruction or not.
         */
        bool               m_temporary;
        /** @brief Europa domain
         *
         * The pointer to the domain to be restricted every time this instance
         * visits a TREX domain
         */
        EUROPA::Domain    *m_dom;
        /** @brief Europa type of the domain
         *
         * The type of the domain manipulated by this visitor
         */
        EUROPA::DataTypeId m_type;
        
        // following methods have purposedly no code
        europa_domain();
      }; // TREX::europa::details::europa_domain
      
      /** @brief T-REX to europa domain
       *
       * @param[in] var A europa variable
       * @param[in] dom A T-REX domain
       *
       * Restrict the base domain of @p var with @p dom
       *
       * @pre the type of @p dom can be converted to the type of @p var
       *
       * @throw TREX::transaction::DomainAccess The types of @p dom and @p var 
       * are incompatibles
       * @throw TREX::transaction::EmptyDomain The intersection of @p var and 
       * @p dom would be empty
       * @relates europa_domain
       * @ingroup europa
       */
      inline void europa_restrict(EUROPA::ConstrainedVariableId &var,
                                  TREX::transaction::DomainBase const &dom) {
        europa_domain convert(var->lastDomain());
        dom.accept(convert);
        var->restrictBaseDomain(convert.domain());
      }
      
    } // TREX::europa::details
    
    
  } // TREX::europa
} // TREX

#endif // H_trex_europa_convert
