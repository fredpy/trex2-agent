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

# include "BasicInterval.hh"

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
    class BooleanDomain :public BasicInterval {
    public:
      /** @brief Symbolic type name for this domain
       *
       * This static attibute gives an access to the symbolic name
       * used for BooleanDomain. As stated in the main class description
       * its value is "bool" but programmers who want to use this name
       * would better use this variable in case it changes in the future
       */
      static TREX::utils::symbol const type_name;

      /** @brief Default constructor

       *
       * Creates a full boolean domain
       */
      BooleanDomain()
	:BasicInterval(type_name), m_full(true) {}
      /** @brief Constructor
       * @param val A booelan value
       *
       * Creates a singleton domain with value @e val
       */
      explicit BooleanDomain(bool val)
	:BasicInterval(type_name), m_full(false), m_val(val) {}
      /** @brief Copy constructor
       * @param d Another instance
       *
       * Create a copy of @e d
       */
      BooleanDomain(BooleanDomain const &d)
	:BasicInterval(d), m_full(d.m_full), m_val(d.m_val) {}
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
      explicit BooleanDomain(boost::property_tree::ptree::value_type &node);

      /** @brief Destructor */
      ~BooleanDomain() {}

      std::ostream &toXml(std::ostream &out, size_t tabs) const;

      DomainBase *copy() const {
	return new BooleanDomain(*this);
      }
      bool hasLower() const {
	return isSingleton();
      }
      bool hasUpper() const {
	return isSingleton();
      }
      bool isFull() const {
	return m_full;
      }
      bool isSingleton() const {
	return !m_full;
      }
      
      bool intersect(DomainBase const &other) const;
      bool equals(DomainBase const &other) const;
      DomainBase &restrictWith(DomainBase const &other);

    private:
      boost::any getLower() const {
	return m_full?false:m_val;
      }
      boost::any getUpper() const {
	return m_full?true:m_val;
      }
      bool json_protect() const {
        return false;
      }

      void parseSingleton(std::string const &val);
      void parseLower(std::string const &val);
      void parseUpper(std::string const &val);
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
      
    }; // BooleanDomain

  } // TREX::transaction
} // TREX

#endif // H_BooleanDomain
