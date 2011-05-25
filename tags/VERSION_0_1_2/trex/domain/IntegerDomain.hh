/* -*- C++ -*- */
/** @file "IntegerDomain.hh"
 * @brief Integer domain representation
 *
 * This files defines an integer based domain
 *
 * @author Frederic Py <fpy@mbari.org>
 * @ingroup domains
 */
#ifndef H_IntegerDomain 
# define H_IntegerDomain 

# include "IntervalDomain.hh" 

namespace TREX {
  namespace transaction {
    
    /** @brief Integer domain
     *
     * This class implements a a domain used to describe integers.
     * The symbolic type name for this class is "int" and it fully
     * support XML serialization.
     *
     * @author Frederic Py <fpy@mbari.org>
     * @ingroup domains
     */
    class IntegerDomain :public IntervalDomain<long> {
    public:
      /** @brief Symbolic type name for this domain
       *
       * This static attibute gives an access to the symbolic name
       * used for IntegerDomain. As stated in the main class description
       * its value is "int" but programmers who want to use this name
       * would better use this variable in case it changes in the future
       */
      static TREX::utils::Symbol const type_name;
      
      /** @brief Constructor
       *
       * Create a full integer domain
       */
      IntegerDomain() 
	:IntervalDomain<long>(type_name) {}
      /** @brief Constructor
       *
       * @param node A XML node
       *
       * Create a integer domain by parsing @e node.
       * 
       * @throw TREX::utils::bad_string_cast One of the attributes is
       * not correctly formatted
       * @throw EmptyDomain the resulting interval is empty
       */
      IntegerDomain(rapidxml::xml_node<> const &node)
	:IntervalDomain<long>(node) {}
      /** @brief Constructor
       * @param lb A lower bound
       * @param ub An upper bound
       *
       * Create an integer domain represented by the interval [lb, ub]
       * @pre [lb, ub] is a valid interval (ie not empty)
       * @throw EmptyDomain the resulting domain is empty
       */
      IntegerDomain(IntervalDomain<long>::bound const &lb, 
		    IntervalDomain<long>::bound const ub)
	:IntervalDomain<long>(type_name, lb, ub) {}
      /** @brief Constructor
       * @param val A value
       *
       * Create a new integer domain with only the value 
       * @e val. in other terms the domain is represented by the interval
       * [val, val]
       */
      IntegerDomain(long val) 
	:IntervalDomain<long>(type_name, val) {}
      /** @brief Destructor */
      ~IntegerDomain() {}

      DomainBase *copy() const {
	return new IntegerDomain(*this);
      }
      
    }; // IntegerDomain 

  } // TREX::utils
} // TREX

#endif // H_IntegerDomain
