/* -*- C++ -*- */
/** @file "Predicate.hh"
 * @brief abstract TREX predicate for transactions
 *
 * This header defines the abstract Predicate class which is a
 * basic representation of a predicate. Both Goals and Observations
 * derive from this class.
 *
 * @author Frederic Py <fpy@mbari.org>
 * @ingroup transaction
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
#ifndef H_Predicate
# define H_Predicate

# include <map>
# include <list>

// put it this way to avoid conflict with Europa
# include <trex/domain/Variable.hh>

namespace TREX {
  namespace transaction {

    /** @brief Predicate manipulation error
     *
     * This exception is thrown while an error was detected during
     * Predicate manipulation
     *
     * @author Frederic Py <fpy@mbari.org>
     * @relates Predicate
     * @ingroup transaction
     */
    class PredicateException :public TREX::utils::Exception {
    public:
      /** @brief Constructor
       * @param msg A message
       *
       * Create a new instance with associated message @a msg
       */
      PredicateException(std::string const &msg) throw()
	:TREX::utils::Exception(msg) {}
      /** @brief Destructor */
      ~PredicateException() throw() {}
    }; // TREX::transaction::PredicateException
    

    /** @brief TREX Predicate
     *
     * This class defines predicate as manipulated by TREX. A predicate is
     * an abstract representaion of Goal and/or Observation exchanged
     * between reactors.
     * It is a named @e predicate attached to an @e object with different
     * attributes represented as Variable.
     *
     * The @e object is the name of the timeline supporting this observation
     * while the @c predicate is the value of the state for @e object
     *
     * A goal has just the specificity to also define the @c start,
     * @c duration and @c end temporal attributes.
     *
     * @author Frederic Py <fpy@mbari.org>
     * @ingroup transaction
     */
    class Predicate :public TREX::utils::ostreamable, public TREX::utils::ptree_convertible {
    protected:
      /** @brief Type used to store predicate attributes */
      typedef std::map<TREX::utils::Symbol, Variable> attr_set;

    public:
      /** @brief Predicate attributes' iterator */
      typedef attr_set::iterator iterator; 
      /** @brief Predicate attributes' const iterator */
      typedef attr_set::const_iterator const_iterator; 

      /** @brief Destructor */
      virtual ~Predicate() =0;

      /** @brief Predicate object
       *
       * This method indicates which object this predicate is associated
       * to. In TREX this object has to be the name of a timeline shared
       * between reactors.
       *
       * @return the name of the associated object
       */
      TREX::utils::Symbol const &object() const {
	return m_object;
      }
      /** @brief predicate type
       *
       * This method indicates the type of the predicate. This type
       * reflects the state of the associated object
       *
       * @return the type of the predicate
       */
      TREX::utils::Symbol const &predicate() const {
	return m_type;
      }

      /** @brief beginning of the attributes set
       *
       * This method allows to iterate through the attributes of this
       * predicate
       *
       * @return An iterator pointing to the beginning of the attributes'
       * set
       * @sa iterator end()
       */
      iterator begin() {
	return m_vars.begin();
      }
      /** @brief end of the attributes set
       *
       * This method allows to iterate through the attributes of this
       * predicate
       *
       * @return An iterator pointing to the end of the attributes'
       * set
       * @sa iterator begin()
       */
      iterator end() {
	return m_vars.end();
      }
      /** @brief beginning of the attributes set
       *
       * This method allows to iterate through the attributes of this
       * predicate
       *
       * @return An iterator pointing to the beginning of the attributes'
       * set
       * @sa const_iterator end() const
       */
      const_iterator begin() const {
	return m_vars.begin();
      }
      /** @brief end of the attributes set
       *
       * This method allows to iterate through the attributes of this
       * predicate
       *
       * @return An iterator pointing to the end of the attributes'
       * set
       * @sa const_iterator begin() const
       */
      const_iterator end() const {
	return m_vars.end();
      }

      /** @brief attribute domain restriction
       * @param name A attribute name
       * @param domain A domain
       *
       * This method restrict the domain of the attribute @e name with
       * the domain @e domain
       *
       * @throw EmptyDomain The resulting domain for @e name is empty
       * @sa void restrictAttribute(Variable const &)
       * @sa Variable &restrict(Variable const &)
       */
      void restrictAttribute(TREX::utils::Symbol const &name, 
			     DomainBase const &domain) {
	return restrictAttribute(Variable(name, domain));
      }
      /** @brief attribute domain restriction
       * @param var A varaible describing the attribute
       *
       * This method restrict the domain of the attribute corresponding
       * to @e var with the domain of @e var
       *
       * @throw EmptyDomain The resulting domain for @e name is empty
       * @sa void restrictAttribute(TREX::utils::Symbol const &, DomainBase const &)
       * @sa Variable &restrict(Variable const &)
       */
      virtual void restrictAttribute(Variable const &var);
      /** @brief Check for constrained attribute
       * @param name an attribute
       * This methodsw checks if current instance explicitely defines
       * the attribute @a name
       *
       * @retval true if an attribute @e name is defined
       * @retval false otherwise
       *
       * @note the fact that a Predicate instance does not have an
       * attribute just means that this attribute has not been constrained
       * yet using the restrictAttribute method. It is mostly used to check
       * for attribute existence when one wants to access to the domain
       * associated to this attribute
       *
       * @sa Variable const &getAttribute(TREX::utils::Symbol const &) const
       */
      bool hasAttribute(TREX::utils::Symbol const &name) const {
	return end()!=m_vars.find(name);
      }
      /** @brief Attribute access
       * @param name The name of an attribute
       * @pre an attribute @a name should exist
       * @return The varaible corresponding to the attribute @e name 
       * @throw PredicateException Attribute @e name does not exist
       *
       * @sa template<class Ty> Ty const &getDomain(TREX::utils::Symbol const &) const
       */
      virtual Variable const &getAttribute(TREX::utils::Symbol const &name) 
	const;
      /** @brief Attribute domain access
       * @tparam Ty the type of the domain
       * @param name The name of an attribute
       * @pre Ty should inherit from DomainBase
       * @pre Ty should be a base class of the domain associted to this
       * Variable
       * @pre an attribute @a name should exist
       * @return The domain corresponding to the attribute @e name casted
       * to domain class @e Ty
       * @throw PredicateException Attribute @e name does not exist
       * @throw std::bad_cast Ty is not the correct domain
       *
       * @sa Variable &getDomain(TREX::utils::Symbol const &) const
       * @sa template<class Ty> Ty const &Variable::typedDomain() const
       */
      template<class Ty> 
      Ty const &getDomain(TREX::utils::Symbol const &name) const {
	BOOST_STATIC_ASSERT((boost::is_convertible<Ty const &, 
			     DomainBase const &>::value));
	Variable const &attr = getAttribute(name);
	return attr.typedDomain<Ty>();
      }
      /** @brief Attribute access
       * @param name The name of an attribute
       * @pre an attribute @a name should exist
       * @return The varaible corresponding to the attribute @e name 
       * @throw PredicateException Attribute @e name does not exist
       *
       * @sa Variable const &getAttribute(TREX::utils::Symbol const &) const 
       * @sa template<class Ty> Ty const &getDomain(TREX::utils::Symbol const &)
       */
      Variable const &operator[](TREX::utils::Symbol const &name) const {
	return getAttribute(name);
      }
    
      /** @brief List predicate's attributes
       * @brief attr A varaible to store the attribute names
       * @brief all A flag used to indicate whether it should list all
       * existing attributes or only the ones with restricted domains (ie
       * not full)
       *
       * This method is adding the list of all the attributes names defined
       * for this predicate at the end of @e attr. The @e all flag allows
       * to indicate that we want all the defined attributes even if their
       * domain is full (meaning they are not really constrained)
       *
       * @post attr list contains all the attributes appendded to it end
       */
      virtual void listAttributes(std::list<TREX::utils::Symbol> &attr,
				  bool all=true) const;
      
      boost::property_tree::ptree as_tree() const {
        return as_tree(true);
      }
      
      boost::property_tree::ptree as_tree(bool all) const;
      
      /** @brief XML output
       * @param out An output stream
       * @param tabs desired tags indentation
       *
       * This method writes an XML form of current instance to @e out
       * indented using @e tab.
       *
       * The output excludes all the attributes that are not really
       * constrained to reduce the size of this description.
       *
       * @return @e out after the operation
       */
      std::ostream &toXml(std::ostream &out) const {
        return to_xml(out);
      }
      std::ostream &toJSON(std::ostream &out) const {
        return to_json(out);
      }

      /** @brief XML parsing factory for predicates
       *
       * This defines a factory that allows to parse Predicate classes
       * from XML.
       */
      typedef TREX::utils::XmlFactory< Predicate, 
				       boost::shared_ptr<Predicate> > xml_factory;

      /** @brief Check for predicate compatibility
       *
       * @param[in] other Another predicate 
       *
       * test if @p other can be merged with this instance. Which means that both
       * tokens shares the same domain and predicate and the intersection of all 
       * their domains is not empty.
       *
       

       * @retval true if @p other can be merged 
       * @retval false otherwise
       *
       * @note This method do not check the temporal attributes (start,
       * duration and end).
       */
      bool consistentWith(Predicate const &other) const;
      
            
    protected:
      /** @brief Constructor
       * @param obj An object name
       * @param pred A predicate type
       *
       * Create a new instance referring to object @e obj with the state
       * @e pred.
       *
       * @pre both @e obj and @e pred should not be empty symbols
       * @throw PredicateException One of the argument was empty
       */
      Predicate(TREX::utils::Symbol const &obj, 
		TREX::utils::Symbol const &pred)
	:m_object(obj), m_type(pred) {
	if( obj.empty() )
	  throw PredicateException("Empty predicate's object names are not allowed");
	if( pred.empty() )
	  throw PredicateException("Empty predicate names are not allowed");	
      }
      /** @brief Constructor
       * @param node A XML node
       *
       * Create a new insatnce by parsing the XML node @e node
       *
       * @throw PredicateException object and/or predicate type were not
       * correctly defined in @e node
       * @throw VariableException An XML attribute definition is ill-formed
       * @throw XMLError An occurred while parsing domains definition
       *
       * @sa xml_factory
       * @sa Variable::Variable(rapidxml::xml_node<> const &)
       */
      Predicate(boost::property_tree::ptree::value_type &node);
      /** @brief Constructor
       * @param other another instance
       *
       * Create a copy of @a other
       */
      Predicate(Predicate const &other);

      /** @brief Search for attribute
       * @param name Attribute name
       *
       * This method search for attribute @e name
       * @retval An iterator pointing to the attribute @e name if it exists
       * @retval end() if the attribute @e name is not found
       */
      iterator find(TREX::utils::Symbol const &name) {
	return m_vars.find(name);
      }
      /** @brief removce an attribute
       * @param i An iterator
       *
       * remove the attribute pointed by @e i
       */
      void remove(iterator const &i) {
	m_vars.erase(i);
      }

      /** @brief XML tag name
       *
       * This method is used internally to determine the tag that should
       * be used to describe this attribute in its XML form
       *
       * @return The tag name to use
       *
       * @sa std::ostream &toXml(std::ostream &, size_t) const
       */
      virtual TREX::utils::Symbol const &getPredTag() const = 0;
      
      std::ostream &print_attr(std::ostream &out,
                               utils::Symbol const &name,
                               bool &first) const;
      virtual std::ostream &print_to(std::ostream &out) const;
      
    private:
      /** @brief object name */
      TREX::utils::Symbol m_object;
      /** @brief predicate type */
      TREX::utils::Symbol m_type;
      /** @brief Associated attributes */
      attr_set m_vars;

# ifndef DOXYGEN
      // No code for following methods
      Predicate() DELETED;
# endif

    }; // TREX::transaction::Predicate

  } // TREX::transaction
} // TREX

#endif // H_Predicate
