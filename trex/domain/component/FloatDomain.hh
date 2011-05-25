/* -*- C++ -*- */
/** @file "FloatDomain.hh"
 * @brief Float domain representation
 *
 * This files defines a float based domain
 *
 * @author Frederic Py <fpy@mbari.org>
 * @ingroup domains
 */
#ifndef H_FloatDomain 
# define H_FloatDomain 

# include "IntervalDomain.hh" 

namespace TREX {
  namespace transaction {
    
    /** @brief Float domain
     *
     * This class implements a domain used to describe floats.
     * The symbolic type name for this class is "float" and it fully
     * support XML serialization.
     *
     * @author Frederic Py <fpy@mbari.org>
     * @ingroup domains
     */
    class FloatDomain :public IntervalDomain<double> {
    public:
      using typename IntervalDomain<double>::bound;
      
      /** @brief Symbolic type name for this domain
       *
       * This static attibute gives an access to the symbolic name
       * used for FloatDomain. As stated in the main class description
       * its value is "float" but programmers who want to use this name
       * would better use this variable in case it changes in the future
       */
      static TREX::utils::Symbol const type_name;

      /** @brief Constructor
       *
       * Create a full float domain
       */
      FloatDomain()
	:IntervalDomain<double>(type_name) {}
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
      FloatDomain(rapidxml::xml_node<> const &node)
	:IntervalDomain<double>(node) {}
      /** @brief Constructor
       * @param lb A lower bound
       * @param ub An upper bound
       *
       * Create an integer domain represented by the interval [lb, ub]
       * @pre [lb, ub] is a valid interval (ie not empty)
       * @throw EmptyDomain the resulting domain is empty
       */
      FloatDomain(bound const &lb, bound const ub)
	:IntervalDomain<double>(type_name, lb, ub) {}
      /** @brief Constructor
       * @param val A value
       *
       * Create a new integer domain with only the value 
       * @e val. in other terms the domain is represented by the interval
       * [val, val]
       */
      FloatDomain(double val) 
	:IntervalDomain<double>(type_name, val) {}
      /** @brief Destructor */
      ~FloatDomain() {}

      DomainBase *copy() const {
	return new FloatDomain(*this);
      }
      
    }; // FloatDomain 

  } // TREX::utils
} // TREX

#endif // H_FloatDomain
