/* -*- C++ -*- */
/** @file "BooleanDomain.hh"
 * @brief Boolean domain representation
 *
 * This files defines a boolean based domain
 *
 * @author Frederic Py <fpy@mbari.org>
 * @ingroup domains
 */
#ifndef H_BooleanDomain
# define H_BooleanDomain

# include "basic_interval.hh"

namespace TREX {
  namespace transaction {
  
    /** @brief Boolean domain
     *
     * This class implements a domain used to describ a boolean.
     * The symbolic typename for this class is "bool" and it fully
     * support XML serialization.
     *
     * @author Frederic Py <fpy@mbari.org>
     * @ingroup domains
     */
    class boolean_domain :public basic_interval {
    public:
      /** @brief Symbolic type name for this domain
       *
       * This static attibute gives an access to the symbolic name
       * used for BooleanDomain. As stated in the main class description
       * its value is "bool" but programmers who want to use this name
       * would better use this variable in case it changes in the future
       */
      static TREX::utils::symbol const type_str;

      /** @brief Default constructor

       *
       * Creates a full boolean domain
       */
      boolean_domain()
	:basic_interval(type_str), m_full(true) {}
      /** @brief Constructor
       * @param val A booelan value
       *
       * Creates a singleton domain with value @e val
       */
      explicit boolean_domain(bool val)
	:basic_interval(type_str), m_full(false), m_val(val) {}
      /** @brief Copy constructor
       * @param d Another instance
       *
       * Create a copy of @e d
       */
      boolean_domain(boolean_domain const &d)
	:basic_interval(d), m_full(d.m_full), m_val(d.m_val) {}
      /** @brief Constructor
       *
       * @param node A XML node 
       *
       * Create a new instance by parsing the @e node.
       * The expected format for this node is :
       * @code
       * <bool value="<val>"/>
       * @endcode
       *
       * Where @c value attribute is optional and @c @<val@> is an
       * integer representation of a boolean
       *
       * @throw TREX::utils::bad_string_cast Unable to parse value
       * attribute
       */
      explicit boolean_domain(boost::property_tree::ptree::value_type &node);

      /** @brief Destructor */
      ~boolean_domain() {}

      std::ostream &toXml(std::ostream &out, size_t tabs) const;

      abstract_domain *copy() const {
	return new boolean_domain(*this);
      }
      bool has_lower() const {
	return is_singleton();
      }
      bool has_upper() const {
	return is_singleton();
      }
      bool is_full() const {
	return m_full;
      }
      bool is_singleton() const {
	return !m_full;
      }
      
      bool intersect(abstract_domain const &other) const;
      bool equals(abstract_domain const &other) const;
      abstract_domain &restrict_with(abstract_domain const &other);

    private:
      boost::any get_lower() const {
	return m_full?false:m_val;
      }
      boost::any get_upper() const {
	return m_full?true:m_val;
      }
      bool json_protect() const {
        return false;
      }

      void parse_singleton(std::string const &val);
      void parse_lower(std::string const &val);
      void parse_upper(std::string const &val);
      std::ostream &print_lower(std::ostream &out) const;
      std::ostream &print_upper(std::ostream &out) const;


      /** @brief Flag for full domain
       * This flag indicat if the domain is full
       */
      bool m_full;
      /** @brief Domain value
       * This value is releveant only if m_full is false. It then
       * represent the value of the domain
       */
      bool m_val;
      
    }; // boolean_domain

  } // TREX::transaction
} // TREX

#endif // H_BooleanDomain
