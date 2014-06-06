/* -*- C++ -*- */
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
/** @file "DomainBase.hh"
 * @brief TREX domain basic utilities
 *
 * This file defines the bases for implementing domains in TREX.
 * Domains are used for representing attributes' values for goals
 * and observations between reactors.
 *
 * @note Domains representation used to rely on Europa implementation.
 * In an effort to make TREX independent from Europa framework this
 * version of TREX provides this new implementation. Note that it can
 * evolve in the future on users request or performances requirements.
 *
 * @author Frederic Py <fpy@mbari.org>
 * @ingroup domains
 */
#ifndef H_DomainBase 
# define H_DomainBase

# include <boost/any.hpp>

# include <trex/utils/xml_factory.hh>
# include <trex/utils/ptree_io.hh>
# include <trex/utils/platform/cpp11_deleted.hh>

# include "domain_visitor_fwd.hh"

namespace TREX {
  /** @brief TREX transaction framework
   *
   * This namespace embeds all objects and utilities used
   * for messaging between TREX reactors. It especially
   * defines basic data structures to describe goals and
   * observations between reactors. 
   *
   * People who want to implement a new reactor (based on
   * their own planning framework or whatever) should use
   * classes defined here for providing messages to other
   * reactors.
   *
   * @author Frederic Py <fpy@mbari.org>
   * @note This namespace is shared between @ref domains and
   * @ref transaction This should change in the future
   */
  namespace transaction {      
    
    class abstract_domain;
    
    namespace domain_error {
      enum domain_error_t {
        ok                = 0,
        empty_domain      = 1,
        not_a_singleton,
        incompatible_types,
        domain_access,
        unnamed_var
      }; // TREX::transaction::domain_error::domain_error_t
      
      ERROR_CODE make_error(domain_error_t e);
      
    }
    
    /** @brief Abstract domain definition
     *
     * This class is an abstract representation of a domain as manipulated
     * by TREX. A domain specifies a set of possible values generally for a
     * Variable
     *
     * @author Frederic Py <fpy@mbari.org>
     * @ingroup domains 
     */
    class abstract_domain :public TREX::utils::ptree_convertible {
    public:
      /** @brief Destructor */
      virtual ~abstract_domain() {}
      /** @brief Duplicate method
       *
       * Create a new instance which is identical to current instance value.
       * This instance is usually created using the @c new operator and it
       * is up to the caller to de-allocate it when not needed.
       *
       * @return The newly created instance
       */
      virtual abstract_domain *copy() const=0;

      /** @brief Visiting method
       *
       * @param visitor A domain visitor
       *
       * This method implement the Visitor design pattern for TREX domains.
       * It will call @e visitor methods depending on the actual type of this
       * instance.
       *
       * There's 3 possible cases supported :
       * @li this class is a BasicInterval instance
       * @li this class is a BasicEnumerated instance
       * @li this class is none of the above
       *
       * in each case the corresponding method of @e visitor will be called
       *
       * @sa DomainVisitor
       */
      virtual void accept(domain_visitor &visitor) const;

      /** @brief Check for interval domain
       *
       * This method is used to identify if this domain is an interval
       * based domain. In which case it is represented by a lower and
       * upper bound.
       *
       * @retval true if the domain is interval based
       * @retval false otherwise
       *
       * @sa bool isEnumerated() const
       */
      virtual bool is_interval() const =0;
      /** @brief Check for enumerated domain
       *
       * This method check if the domain is based on an
       * enumerated representation. In which case its representation
       * is based on the list of all the possible values.
       *
       * @retval true if the domain is enumeration based
       * @retval false otherwise
       *
       * @sa bool isInterval() const
       */
      virtual bool is_enumerated() const =0;
      
      /** @brief type of the domain
       *
       * This method indicates the type of this domain.
       * The type is a symbol that describes what is the type
       * manipulated by this domain.
       *
       *
       * @return the type associated to this domain
       */
      TREX::utils::symbol const &type_name() const {
	return m_type;
      }

      /** @brief check for common values
       * 
       * @param other another instance
       *
       * This methods indicates if current instance share values with
       * @e other. This generally means that both have the same type
       * and that one can safely restrict the domain of the other.
       *
       * @retval true if current instance shares values with @e other
       * @retval false otherwise
       *
       * @sa bool equals(DomainBase const &other) const
       * @sa DomainBase &restrictWith(DomainBase const &other)
       */
      virtual bool intersect(abstract_domain const &other) const =0;
      /** @brief Equality test
       * @param other Another instance
       *
       * Checks if current instance is identical to @e other. This generally
       * means that both have the same type and exactly the same possible
       * values. In that sense it is more restrictive than the intersect
       * method.
       *
       * @retval true if current instance is identical to other
       * @retval false otherwise
       *
       * @sa bool intersect(DomainBase const &other) const
       */
      virtual bool equals(abstract_domain const &other) const =0;
      
      /** @brief Restrict domain possible values
       * @param other Another domain
       *
       * This method restrict the domain of current instance to the
       * values that are shared by this domain and @e other.
       *
       * @pre this domain intersect @e other
       * @retval true if the domain changed
       * @retval false otherwise
       * @throw EmptyDomain resulting domain is empty
       * @sa bool intersect(DomainBase const &) const
       */
      virtual bool restrict_with(abstract_domain const &other) =0;
      /** @brief Restrict domain possible values
       * @param other Another domain
       *
       * This method restrict the domain of current instance to the
       * values that are shared by this domain and @e other.
       *
       * @pre this domain intersect @e other
       * @return this domain after the operation
       * @throw EmptyDomain resulting domain is empty
       * @sa bool intersect(DomainBase const &) const
       */
      abstract_domain &operator*=(abstract_domain const &other) {
	restrict_with(other);
        return *this;
      }

      /** @brief check for full domain
       * 
       * This method identifies if current domain is full. A full domain
       * is adomain that includes al the possibles values for its type.
       *
       * @retval true if current instance is full
       * @retval false otherwise
       */
      virtual bool is_full() const =0;

      /** @brief check for singleton value
       * 
       * This method identifies if current domain is a singleton. A singleton
       * domain is defined by one and only one value.
       * 
       * @retval true if current instance is a singleton 
       * @retval false otherwise
       */
      virtual bool is_singleton() const =0;

      /** @brief singleton value
       *
       * This methods give acces to the value of a singleton instance.
       *
       * @pre the instance is a singleton
       * @throw DomainAccess the insatnce was not a singleton
       * @return The value of the domain encapsulated in a boost::any
       *
       * @note except for the control of the instance being a singleton the
       * actual code of this method will be provided by the virtual method
       * singleton()
       *
       * @sa boost::any singleton() const
       * @sa getStringSingleton() const
       * @sa getTypedSingleton() const
       */
      boost::any get_singleton() const;
      /** @brief singleton value
       *
       * This methods give acces to the value of a singleton instance
       * represented as a string
       *
       * @pre the instance is a singleton
       * @throw DomainAccess the insatnce was not a singleton
       * @return The text representation of the singleton value
       *
       * @note except for the control of the instance being a singleton the
       * actual code of this method will be provided by the virtual method
       * stringSingleton()
       *
       * @sa std::string stringSingleton() const
       * @sa getStringSingleton() const
       * @sa getTypedSingleton() const
       */
      std::string get_singleton_as_string() const;
      /** @brief singleton value
       *
       * @tparam Ty output type
       * @tparam Safe robust conversion
       *
       * This methods give acces to the value of a singleton instance
       * The output will be of type @e Ty
       *
       * The @e Safe parameter indicates which method should be used.
       * @li If the real type of the underneath domain type is guaranteed to
       * be @a Ty the @a Safe can be @c false for performance
       * @li If the type is not guaranteed to be @a Ty  but its text
       * representation is compatible with @a Ty the @e Safe should be @c true
       *
       * @pre the instance is a singleton
       * @pre the type of the value manip[ulated by this domain should be
       * convertible to @e Ty or @i exactly @e Ty is @e Safe is @c false 
       * @throw DomainAccess the insatnce was not a singleton
       * @throw boost::bad_string_cast @a Safe is @a true and the value of
       * the singleton cannot be parse into @a Ty
       * @throw boost::bad_string_cast @a Safe is @a false and the type of
       * the singleton is not  @a Ty  
       * @return The singleton value of this domain
       *
       *
       * @sa getStringSingleton() const
       * @sa getSingleton() const
       */
      template<class Ty, bool Safe>
      Ty get_typed_singleton() const {
        boost::any val = get_singleton();
	if( Safe ) {
          Ty *ret = boost::any_cast<Ty>(&val);
          if( NULL!=ret )
            return *ret;
          else
            return boost::lexical_cast<Ty>(get_singleton_as_string());
	} else 
	  return boost::any_cast<Ty>(val);
      }

      boost::property_tree::ptree as_tree() const;
      
      
      /** @brief XML parsing factory for domains
       *
       * This type defines the factory that should be manipulated
       * for parsing domains from xml.
       */
      typedef TREX::utils::xml_factory<abstract_domain,
                                       SHARED_PTR<abstract_domain> > factory;
      virtual boost::property_tree::ptree build_tree() const=0;

    protected:
      
      /** @brief Constructor
       *
       * @param type A domain type descriptor
       *
       * Create a new instance with associated domain type @e type
       */
      explicit abstract_domain(TREX::utils::symbol const &type)
	:m_type(type) {}
      /** @brief XML parsing constructor
       *
       * @param node An XML node 
       *
       * Create a new instance from parsing @e node. This constructor
       * is used by xml_factory to generate domains from XML. At this abstract 
       * level. The only operation donae is to extract the type of the domain
       * from the node tag.
       *
       * @note to ensure compatibility this constructor should parse a XML
       * code which is consistent with what is produced by the toXml method of
       * your class
       *
       * @sa std::ostream &toXml(std::ostream &out, size_t tabs) const
       * @sa xml_factory
       */
      explicit abstract_domain(boost::property_tree::ptree::value_type &node)
	:m_type(node.first) {}   
      /** @brief Copy constructor
       * @param other Another instance
       *
       * Create a new instance similar to @e other
       */
      abstract_domain(abstract_domain const &other)
	:m_type(other.m_type) {}

      /** @brief Print domain
       *
       * @param out An output stream
       *
       * Print a non full domain value to @e out
       *
       * This method is called internally by the @c print_to method when
       * the domain to print is not full. Indeed Abstract domain represents
       * full domains as '*'
       *
       * @return The output stream after the operation
       *
       * @sa std::ostream &print_to(std::ostream &out) const
       */
      virtual std::ostream &print_domain(std::ostream &out) const =0;

      /** @brief singleton value
       * This method is called by getSingleton() to get the actual value of
       * a singleton domain
       *
       * @return the value of the singleton
       *
       * @note as this method is called by getSingleton we have the guarantee
       * that it will be called only if the domain is a singleton
       *
       * @sa boost::any getSingleton() const
       */
      virtual boost::any singleton() const =0;
      /** @brief singleton text representation
       * This method is called by getStringSingleton() to get the actual value of
       * a singleton domain as text
       *
       * @return the textual value of the singleton
       *
       * @note as this method is called by getStringSingleton we
       * have the guarantee
       * that it will be called only if the domain is a singleton
       *
       * @sa std::string getStringSingleton() const
       */
      virtual std::string string_singleton() const =0;

    private:
      /** @brief type of the domain */
      TREX::utils::symbol const m_type;


# ifndef DOXYGEN
      // Following method have no code
      abstract_domain() DELETED;
# endif

      friend std::ostream &operator<<(std::ostream &out,
                                      abstract_domain const &d) {
        if( d.is_full() )
          return out.put('*');
        else
          return d.print_domain(out);
      }

    }; // TREX::transaction::abstract_domain

    
  } // TREX::transaction
} // TREX

#endif // H_DomainBase
