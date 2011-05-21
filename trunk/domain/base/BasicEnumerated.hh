/* -*- C++ -*- */
/** @file "BasicEnumerated.hh"
 * @brief Abstract interface for enumerated domains
 *
 * This file defines the bases class for implementing enumerated
 * domains in TREX. It offers multiple utilities to have unified
 * access/input/output for this kind of domain.
 *
 * @author Frederic Py <fpy@mbari.org>
 * @ingroup domains
 */
#ifndef H_BasicEnumerated
# define H_BasicEnumerated

# include "DomainBase.hh"

namespace TREX {
  namespace transaction {

    /** @brief Enumerated Domain base class
     *
     * This class provide a unified abstract interface for implementing
     * enumerated domains. It especially provide a way to ease and unify
     * the access methods and input/output for all the domain of this kind.
     *
     * @author Frederic Py <fpy@mbari.org>
     * @ingroup domains
     */
    class BasicEnumerated :public DomainBase {
    public:
      /** @brief Destructor */
      virtual ~BasicEnumerated() {}

      bool isInterval() const {
	return false;
      }
      bool isEnumerated() const {
	return true;
      }

      bool isFull() const {
	return 0==getSize();
      }
      bool isSingleton() const {
	return 1==getSize();
      }
      std::ostream &toXml(std::ostream &out, size_t tabs) const;

      /** @brief element count
       *
       * This method gives access to the number of elements in this instance.
       * @return the number of values associated to this domain
       *
       * @note by convention if the returned value is 0, TREX will
       * consider this domain as full
       */
      virtual size_t getSize() const =0;
      /** @brief generic element access
       * @param i index of the element
       *
       * This method give access to the element @a i of the domain. 
       *
       * @pre @a i is less than the size of the set
       *
       * @return the element at index @a i encapsulated on the
       * @c boost::any class
       *
       * @sa size_t getSize() const
       */
      virtual boost::any getElement(size_t i) const =0;
      /** @overload boost::any getElement(size_t i) const
       */
      boost::any operator[](size_t i) const {
	return getElement(i);
      }
      /** @brief Typed element access
       * @tparam Ty   expected return type
       * @tparam Safe extraction method
       * @param i index of the element
       *
       * This method give access to the element @a i of the domain converted
       * in the type @a Ty. The @a Safe flag indicate whether the conversion to
       * @a Ty should be done using @c boost::any_cast -- which will fail if
       * the encapsulated type is not @a Ty -- or using the safer string_cast
       * that should be more type robust
       *
       * @pre @a i is less than the size of the set
       * @pre if @a Safe is false, the underlying type of the container has
       * to be @u exactly @a Ty
       * @pre if @a Safe is true the string value of this element should be
       * parseable into @a Ty
       *
       * @return the element at index @a i converted to the type @a Ty
       *
       * @note No matter what this method is heavy in term of computation
       * overhead and should be used wisely (mostly for debugging/instrumentation
       * purpose)
       *
       * @sa size_t getSize() const
       * @sa boost::any getElement(size_t i) const
       * @sa std::string getStringValue(size_t i) const
       */
      template< class Ty, bool Safe >
      Ty getTypedElement(size_t n) const {
	if( Safe ) 
	  return TREX::utils::string_cast<Ty>(getStringValue(n));
	else 
	  return boost::any_cast<Ty>(getElement(n));
      }
           
      /** @brief Get element textual value
       * @param[in] i Index of the element
       *
       * Get the value of the element @p i of the domain in its textual form
       *
       * @pre @p i is a valid index (between 0 and the size of the set-1)
       *
       * @return A string with a human readable for the value of the element @p i
       *
       * @sa getSize() const
       * @sa getElement(size_t) const
       * @sa getTypedElement(size_t) const
       */
      virtual std::string getStringValue(size_t i) const;

    protected:
      explicit BasicEnumerated(TREX::utils::Symbol const &type) 
	:DomainBase(type) {}
      explicit BasicEnumerated(rapidxml::xml_node<> const &node)
	:DomainBase(node) {}

      void completeParsing(rapidxml::xml_node<> const &node);

      virtual void addTextValue(std::string const &val) =0;
      virtual std::ostream &print_value(std::ostream &out, size_t i) const =0;
      
    private:
      void accept(DomainVisitor &visitor) const;
      std::ostream &print_domain(std::ostream &out) const;
      
      boost::any singleton() const {
	return getElement(0);
      }
      std::string stringSingleton() const {
	return getStringValue(0);
      }
    }; // TREX::transaction::BasicEnumerated


  } // TREX::transaction
} // TREX 

#endif // H_BasicEnumerated
