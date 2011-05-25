/* -*- C++ -*- */
/** @file "Variable.hh"
 * @brief TREX variable representation
 *
 * This files defines a variable as used for TREX transactions
 *
 * @author Frederic Py <fpy@mbari.org>
 * @ingroup domains
 */
#ifndef H_Variable 
# define H_Variable 

# include <functional>
# include <boost/static_assert.hpp>
# include <boost/type_traits.hpp>

# include <trex/utils/Symbol.hh>
# include "DomainBase.hh"

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
    class Variable :public TREX::utils::ostreamable {
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
      Variable();
      /** @brief Constructor
       * @param name A symbolic name
       * @param domain A domain
       *
       * Create a new varaible named @e name with the domain @e domain
       * @pre @e name should not be empty
       * @throw VariableException Tried to create a variable with an
       * empty name
       */
      Variable(TREX::utils::Symbol const &name,
	       DomainBase const &domain);
      /** @brief Constructor
       * @brief var Another instance
       *
       * Create a copy of @e var
       */
      Variable(Variable const &var);
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
      Variable(rapidxml::xml_node<> const &node);
      /** @brief Destructor */
      ~Variable();

      /** @brief Check for complete varaible definition
       *
       * This method checks if current instance is fully defined with a
       * correct name and domain.
       *
       * @retval true if fully defined
       * @retval false else
       */
      bool isComplete() const {
	return !( m_name.empty() || !m_domain );
      }
      
      /** @brief Assignment operator
       *
       * @param other Another instance
       *
       * This method copy the value of @e other in current
       * instance.
       *
       * @return current instance after operation
       */
      Variable &operator= (Variable const &other);
      
      /** @brief Variable name
       * @return the name of this variable
       * @sa bool isComplete() const
       */
      TREX::utils::Symbol const &name() const {
	return m_name;
      }

      /** @brief Variable domain
       *
       * @pre The variable is complete
       * @return the domain of this variable
       * @throw VariableException the domain of this varaible is undefined
       * @sa bool isComplete() const
       * @sa template<class Ty> Ty const &typedDomain() const
       */
      DomainBase const &domain() const;
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
      Ty const &typedDomain() const {
	BOOST_STATIC_ASSERT((boost::is_convertible<Ty const &, 
			     DomainBase const &>::value));
	return dynamic_cast<Ty const &>(domain());
      }
      
      void accept(DomainVisitor &visitor) const {
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
      Variable &restrict(DomainBase const &dom);
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
      Variable &operator*=(DomainBase const &dom) {
	return restrict(dom);
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
      Variable &restrict(Variable const &var);
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
      Variable &operator*=(Variable const &var) {
	return restrict(var);
      }

      /** @brief XML output
       * @param out An output stream
       * @param pad desired tag indentation
       *
       * This method writes in @e out an XML representation of current
       * instance with an indentation of @e pad
       *
       * @return @a out after the operation
       */
      std::ostream &toXml(std::ostream &out, size_t pad=0) const;
	
    private:
      typedef DomainBase::xml_factory::returned_type domain_ptr;
      /** @brief varaible name */
      TREX::utils::Symbol m_name;
      /** @brief Variable domain */
      domain_ptr m_domain;

      std::ostream &print_to(std::ostream &out) const;

      /** @brief Entry point to domain XML parsing */
      static TREX::utils::SingletonUse< DomainBase::xml_factory > s_dom_factory;

      /** @brief Domain duplication helper
       * @param dom A domain
       * This method helps to duplicate a domain pointed by @e dom
       * while managing the case where @e dom is NULL
       * @return a pointer to a copy of @a *dom or a NULL pointer
       */
      static DomainBase *clone(boost::call_traits<Variable::domain_ptr>::param_type dom);
      
      /** @brief Constructor
       * @param name A symbolic name 
       * @param domain A poiunbter to a domain
       *
       * This constructor is used internally to create a variable
       * from a domain that has already been allocated
       */
      explicit Variable(TREX::utils::Symbol const &name,
			DomainBase *domain = NULL);
      friend class Predicate;
   }; // class TREX::transaction::Variable
      
  } // TREX::transaction
} // TREX 

#endif // H_Variable
