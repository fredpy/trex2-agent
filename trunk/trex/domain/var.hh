/* -*- C++ -*- */
/** @file "Variable.hh"
 * @brief TREX variable representation
 *
 * This files defines a variable as used for TREX transactions
 *
 * @author Frederic Py <fpy@mbari.org>
 * @ingroup domains
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
#ifndef H_Variable 
# define H_Variable 

# include <functional>
# include <boost/static_assert.hpp>
# include <boost/type_traits.hpp>

# include <trex/utils/symbol.hh>
# include "abstract_domain.hh"

namespace TREX {
  namespace transaction {

    class Predicate;

    /** @brief Variable related exception
     *
     * This exception is thrown when a problem is detected during a
     * Variable manipulation
     *
     * @relates Variable
     * @ingroup domains
     */
    class VariableException :public TREX::utils::Exception {
    public:
      /** @brief Constructor
       * @param msg A message
       *
       * Creat a new instance with the associated message @e msg
       */
      VariableException(std::string const &msg) throw()
	:TREX::utils::Exception(msg) {}
      /** @brief Destructor */
      ~VariableException() throw() {}
    }; // TREX::transaction::VariableException
    

    /** @brief TREX variable representation
     *
     * This class implements a TREX variable. variables are
     * used for representing any attribute of an Observation or
     * Goal excahnged between reactors. They are represented by a
     * symbolic name and its associated domain.
     *
     * @author Frederic Py <fpy@mbari.org>
     * @ingroup domains
     */
    class var :public TREX::utils::ptree_convertible {
    private:
      class impl;

      UNIQ_PTR<impl> m_impl;
    public:
      /** @brief Constructor
       *
       * Create an unnamed varaible with no domain.
       *
       * @note This constructor is implemented only for allowing the
       * support of the Varibale class for standard C++ containers. It
       * should not be used by other components as it create an undefined
       * variable
       */
      var();
      /** @brief Constructor
       * @param name A symbolic name
       * @param domain A domain
       *
       * Create a new varaible named @e name with the domain @e domain
       * @pre @e name should not be empty
       * @throw VariableException Tried to create a variable with an
       * empty name
       */
      var(TREX::utils::symbol const &name,
          abstract_domain const &domain);
      /** @brief Constructor
       * @brief var Another instance
       *
       * Create a copy of @e var
       */
      var(var const &var);
      /** @brief Constructor
       * @param node A XML node
       *
       * Create a new instance by parsing the content of @e node. This
       * content should be similar to the following
       * @code
       * <Variable name="<name>">
       *   domain
       * </Variable>
       * @endcode
       *
       * Where @c @<name@> is a non empty string and @c domain is an XML
       * domain description supported by DomainBase::xml_factory.
       *
       * @throw Xmlerror @c name attribute was empty
       * @throw XmlError @c domain was missing
       * @throw XmlError An error occurred while parsing the domain
       */
      var(boost::property_tree::ptree::value_type &node);
      /** @brief Destructor */
      ~var();

      /** @brief Check for complete varaible definition
       *
       * This method checks if current instance is fully defined with a
       * correct name and domain.
       *
       * @retval true if fully defined
       * @retval false else
       */
      bool is_complete() const;
      
      /** @brief Assignment operator
       *
       * @param other Another instance
       *
       * This method copy the value of @e other in current
       * instance.
       *
       * @return current instance after operation
       */
      var &operator= (var const &other);
      
      /** @brief Variable name
       * @return the name of this variable
       * @sa bool isComplete() const
       */
      TREX::utils::symbol name() const;

      /** @brief Variable domain
       *
       * @pre The variable is complete
       * @return the domain of this variable
       * @throw VariableException the domain of this varaible is undefined
       * @sa bool isComplete() const
       * @sa template<class Ty> Ty const &typedDomain() const
       */
      abstract_domain const &domain() const;
      /** @brief Variable domain
       * @tparam Ty Expected type of the domain
       *
       * This method tries to return the domain of the Variable as
       * a const reference to @a Ty
       * @pre Ty should inherit from DomainBase
       * @pre Ty should be a base class of the domain associted to this
       * Variable
       * @pre The variable is complete
       * @throw VariableException the domain of this variable is undefined
       * @throw std::bad_cast Ty is not the correct domain
       *
       * @note We use boost utilities to check that Ty inherits from
       * DomainBase. This allows to detect a mistke on compilation time.
       * @sa bool isComplete() const
       * @sa DomainBase &domain() const
       */
      template<class Ty>
      Ty const &typed_domain() const {
	BOOST_STATIC_ASSERT((boost::is_convertible<Ty const &, 
			     abstract_domain const &>::value));
	return dynamic_cast<Ty const &>(domain());
      }
      
      void accept(domain_visitor &visitor) const {
	domain().accept(visitor);
      }

      /** @brief Restrict Variable domain
       * @param dom a domain
       *
       * This method attempts to restrict current variable domain with
       * @e dom. The resulting domain should be the intersection of
       * @e dom with the current variable domain or simpluy a copy of
       * @e dom if this varaible had no associated domain yet
       *
       * @throw EmptyDomain the resulting domain is empty
       * @return current instance after the operation
       *
       * @sa DomainBase &DomainBase::restrictWith(DomainBase const &)
       */
      var &restrict_with(abstract_domain const &dom);
      /** @brief Restrict Variable domain
       * @param dom a domain
       *
       * This method attempts to restrict current variable domain with
       * @e dom. The resulting domain should be the intersection of
       * @e dom with the current variable domain or simpluy a copy of
       * @e dom if this varaible had no associated domain yet
       *
       * @throw EmptyDomain the resulting domain is empty
       * @return current instance after the operation
       *
       * @sa DomainBase &DomainBase::restrictWith(DomainBase const &)
       */
      var &operator*=(abstract_domain const &dom) {
	return restrict_with(dom);
      }
      /** @brief Merge variable
       * @param var another variable
       *
       * This method tries to merge current insatnce with @e var
       * @pre var should have the same name and domain type as current
       * instance or one of them should not be complete
       * @throw VariableException the 2 variables does not have
       * the same name
       * @throw EmptyDomain the intersection of current instance domain
       * with var domain is empty
       * @return current insatnce after being merged with @e var
       * @sa DomainBase &DomainBase::restrictWith(DomainBase const &)
       */
      var &restrict_with(var const &v);
      /** @brief Merge variable
       * @param var another variable
       *
       * This method tries to merge current insatnce with @e var
       * @pre var should have the same name and domain type as current
       * instance or one of them should not be complete
       * @throw VariableException the 2 variables does not have
       * the same name
       * @throw EmptyDomain the intersection of current instance domain
       * with var domain is empty
       * @return current insatnce after being merged with @e var
       * @sa DomainBase &DomainBase::restrictWith(DomainBase const &)
       */
      var &operator*=(var const &v) {
	return restrict_with(v);
      }

      boost::property_tree::ptree as_tree() const;

	
      friend class Predicate;
      
   }; // class TREX::transaction::var
      
  std::ostream &operator<<(std::ostream &out, var const &v);
        
  } // TREX::transaction
} // TREX 

#endif // H_Variable
