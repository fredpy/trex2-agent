/* -*- C++ -*- */
/** @file "StringDomain.hh"
 * @brief string values enumeration domain
 *
 * This file defines the StringDamain class. 
 *
 * The StringDomain is a specialization in order to declare domains
 * based on string values. 
 *
 * @author Frederic Py <fpy@mbari.org>
 * @ingroup domains
 */
#ifndef H_StringDomain
# define H_StringDomain

# include "EnumeratedDomain.hh"

namespace TREX {
  namespace transaction {

    /** @brief String domain
     *
     * This class implements a simple representation of strings domain.
     * The representation is based on an EnumeratedDomain that enumerates all
     * the possible values for this string.
     *
     * @author Frederic Py <fpy@mbari.org> 
     * @ingroup domains
     */
    class StringDomain 
      :public TREX::transaction::EnumeratedDomain<std::string> {
    public:
      static TREX::utils::Symbol const type_name;
      /** @brief Default Constructor 
       * 
       * Creates a new full string domain.
       */
      StringDomain() 
	:TREX::transaction::EnumeratedDomain<std::string>(type_name) {}
      /** @brief Constructor
       * 
       * @tparam a C++ iterator type
       * @param from an iterator
       * @param to an iterator
       *
       * @pre [@e from,  @e to ) should be a valide iterator sequence
       * @pre @e Iter should points to std::string elements
       *
       * Creates a new instance restricted to all the elements in [@e from, @e to)
       *
       * @throw EmptyDomainthe created domain is empty
       */
      template<class Iter>
      StringDomain(Iter from, Iter to) 
	:TREX::transaction::EnumeratedDomain<std::string>(type_name, from, to) {}
      /** @brief Constructor
       * @param val a value
       *
       * Create a new domain with the single value @e val
       */
      StringDomain(std::string const &val) 
	:TREX::transaction::EnumeratedDomain<std::string>(type_name, val) {}
      /** @brief XML parsing constructor
       *
       * @param node an XML node
       *
       * Create a new domain based on the content of @e node. The type of
       * StringDomain is string.
       * @example
       * A full domain is defined by the empty element :
       * @code
       * <string/>
       * @endcode
       *
       * A string containing the two values "foo" and "bar bar" is described by
       * @code
       * <string>
       *   <elem value="foo"/>
       *   <elem value="bar bar"/>
       * </string>
       *@endcode
       */
      explicit StringDomain(rapidxml::xml_node<> const &node) 
	:TREX::transaction::EnumeratedDomain<std::string>(node) {}

      /** @brief Destructor */
      ~StringDomain() {}

      /** @brief Copy operator
       *
       * Allocates a new copy of current instance
       */
      DomainBase *copy() const {
	return new StringDomain(begin(), end());
      }
    }; // TREX::transaction::StringDomain


  } // TREX::transaction
} // TREX
      
      
#endif  // H_StringDomain
