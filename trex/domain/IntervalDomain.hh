/* -*- C++ -*- */
/** @file "IntervalDomain.hh"
 * @brief Interval based domain deifinition
 *
 * This file defines the basic template classe used to define interval
 * based domains.
 *
 * @author Frederic Py <fpy@mbari.org>
 * @ingroup domains
 */
#ifndef H_IntervalDomain 
# define H_IntervalDomain

# include <cctype>

# include "BasicInterval.hh"

namespace TREX {
  namespace transaction {

    /** @brief Interval based domains
     *
     * @param Ty type of the elements
     * @param Comp Ordering function
     *
     * This template class implements a template basis for Interval based
     * domains.
     *
     * An interval domain is a continuous domain in [-inf +inf]. It supports
     * any interval as long as the lower bound is less of equal to the
     * upper bound and the if any of the bound is infinity it is not equal
     * to the other bound (ie we do not allow [-inf -inf] or [+inf +inf]).
     * These intervals are closed on both end.
     *
     * @pre @e Ty is expected to be a continuous type (generally int or float)
     * @pre @e Comp is expected to be a complete order over @e Ty
     *
     * @author Frederic Py <fpy@mbari.org>
     * @ingroup domains
     */
    template< typename Ty, class Comp=std::less<Ty> >
    class IntervalDomain :public BasicInterval {
    public:
      /** @brief subjacent type for interval */
      typedef Ty base_type;

      /** @brief interval bounds extension of @e Ty
       * @relates IntervalDomain
       *
       * This class extend the type @e Ty with the tow specific
       * values @c -inf and @c +inf. It is used to represent interval
       * domains.
       *
       * @sa class IntervalDomain
       * @author Frederic Py <fpy@mbari.org>
       */
      class bound :public TREX::utils::istreamable,
		   public TREX::utils::ostreamable {
      public:
	/** @brief constructor
	 *
	 * @param val A value
	 *
	 * Convert @e val into an interval bound
	 */
	bound(Ty const &val = Ty())
	  :m_inf(false), m_value(val) {}
	/** @brief Copy constructor
	 *
	 * @param other Instance to copy
	 *
	 * Creates a copy of @e other
	 */
	bound(bound const &other) 
	  :m_inf(other.m_inf), m_pos(other.m_pos),
	   m_value(other.m_value) {}
	/** @brief Destructor */
	~bound() {}

	/** @brief Assignment
	 *
	 * @param other another instance
	 *
	 * Assign @e other to the current instance
	 *
	 * @return @c *this after the operation
	 */
	bound &operator= (bound const &other) {
	  m_inf = other.m_inf;
	  m_pos = other.m_pos;
	  m_value = other.m_value;
	  return *this;
	}
	/** @brief Assignment
	 *
	 * @param val A Ty value
	 *
	 * Assign the value @e val to current instance
	 *
	 * @return @c *this after the operation
	 */
	bound &operator= (Ty const &val) {
	  m_inf = false;
	  m_value = val;
	  return *this;
	}

	/** @brief Check for infinity
	 *
	 * Checks if current instance is an infinity bound
	 *
	 * @retval true if the value is +inf or -inf
	 * @retval false if the value is of type @e Ty
	 */
	bool isInfinity() const {
	  return m_inf;
	}
	/** Finite bound value
	 * @pre This bound is not infinite
	 * @return the value associated to this instance
	 *
	 * @sa bool isInfinity() const
	 */
	Ty const &value() const {
	  return m_value;
	}
	/** @brief less than operator
	 *
	 * @param other value to compare
	 *
	 * Test if current instance is less than @e other.
	 * In this test @c -inf is smaller than any other value
	 * and @c +inf greater than any other value.
	 *
	 * @retval true if @c *this<other
	 * @retval false otherwise
	 */
	bool operator< (bound const &other) const {
	  if( m_inf ) 
	    return !m_pos && ( other.m_pos || !other.m_inf );
	  else if( other.m_inf )
	    return other.m_pos;
	  else {
	    return s_cmp(m_value, other.m_value); 
	  }
	}
	/** @brief Equality test
	 *
	 * @param other value to compare
	 *
	 * Test if current instance is equal to @e other.
	 *
	 * @note To ensure genericity this test relies on @e Comp instead
	 * of basic subjacent equality test of @e Ty. This has two side
	 * effects :
	 * @li The test may be inneficient as it relies on the fact that
	 * @c a==b is equivalent to @c !(a<b || b<a). Whivch may require
	 * more computation when used with a simple ordering between basic
	 * C types
	 * @li For the reasonds mentionned above one need to make sure
	 * that @e Comp is a complete order
	 *
	 * @retval true if @c *this==other
	 * @retval false otherwise
	 */
	bool operator==(bound const &other) const {
	  if( m_inf ) 
	    return other.m_inf && (m_pos==other.m_pos);
	  else 
	    return !( other.m_inf || 
		      s_cmp(m_value, other.m_value) ||
		      s_cmp(other.m_value, m_value) );
	}
	/** brief Difference test
	 * @param other Another instance
	 * @retval true if @c *this!=other
	 * @retval false otherwise
	 * @sa bool operator==(bound const &other) const
	 */
	bool operator!=(bound const &other) const {
	  return !operator==(other);
	}
	/** brief Greater than test
	 * @param other Another instance
	 * @retval true if @c *this>other
	 * @retval false otherwise
	 * @sa bool operator<(bound const &other) const
	 */
	bool operator> (bound const &other) const {
	  return other.operator< (*this);
	}
	/** brief Less or equal to test
	 * @param other Another instance
	 * @retval true if @c *this<=other
	 * @retval false otherwise
	 * @sa bool operator<(bound const &other) const
	 */
	bool operator<=(bound const &other) const {
	  return !operator> (other);
	}
	/** brief Greater or equal to test
	 * @param other Another instance
	 * @retval true if @c *this>=other
	 * @retval false otherwise
	 * @sa bool operator<(bound const &other) const
	 */
	bool operator>=(bound const &other) const {
	  return !operator< (other);
	}
	
	/** @brief Minimum value
	 *
	 * @param other another instance
	 *
	 * @return the lowest value between @c *this and @e other
	 * @sa bool operator<(bound const &other) const
	 * @sa bound const &max(bound const &other) const
	 */
	bound const &min(bound const &other) const {
	  if( operator< (other) )
	    return *this;
	  else
	    return other;
	}
	/** @brief Maximum value
	 *
	 * @param other another instance
	 *
	 * @return the highest value between @c *this and @e other
	 * @sa bool operator<(bound const &other) const
	 * @sa bound const &min(bound const &other) const
	 */
	bound const &max(bound const &other) const {
	  if( operator< (other) )
	    return other;
	  else
	    return *this;
	}

	/** @brief addition
	 *
	 * @param other another instance
	 * @param onInf value to return if one is infinite
	 *
	 * Compute the sum of current instance with @e other.
	 *
	 * @retval onInf if this instance or @a other is an infinite bound
	 * @retval the sum of the value of these two instances otherwise
	 *
	 * @sa bound minus(bound const &other, bound const &) const
	 */
	bound plus(bound const &other, bound const &onInf) const;
	/** @brief substraction
	 *
	 * @param other another instance
	 * @param onInf value if infinity
	 *
	 * Compute the differenc between current instance and @e other.
	 *
	 * @param onInf value to return if one is infinite
	 * @retval the sum of the value of these two instances otherwise
	 *
	 * @sa bound plus(bound const &other, bound const &) const
	 */
	bound minus(bound const &other, bound const &onInf) const;
	  
      private:
	/** @brief Comparator between Ty values */
	static Comp s_cmp;
	/** @brief Infinity flag
	 *
	 * This flag indicates whether this instance is an infinite value
	 * or not. Depending on its value only one of m_pos and m_value is
	 * relevant.
	 */
	bool m_inf;
	/** @brief Sign flag
	 *
	 * This flag is relevant only if m_inf is true and indicates
	 * whether the bound is plus infinity of minus infinity.
	 */
	bool m_pos;
	/** @brief Numeric value
	 *
	 * This attribute is relevant only if m_inf is false and gives
	 * the value of the bound.
	 */
	Ty  m_value;

	/** @brief Infinity bound constructor
	 * @param pos a sign flag
	 *
	 * Create a new infinity bound with the associated sign reflected by
	 * @e pos. The extra parameters is here to help compiler identify
	 * that we call this constructor and avoid conflict with other
	 * constructors.
	 */
	bound(bool pos, bool) 
	  :m_inf(true), m_pos(pos) {}

	std::istream &read_from(std::istream &in);
	std::ostream &print_to(std::ostream &out) const;
	  		
	friend class IntervalDomain<Ty, Comp>;
      }; // IntervalDomain<>::bound

      /** @brief Plus infinity bound
       *
       * This static attribute gives access to the plus infinity value
       * for a bound.
       */
      static bound const plus_inf;
      /** @brief Minus infinity bound
       *
       * This static attribute gives access to the minus infinity value
       * for a bound.
       */
      static bound const minus_inf;
      
      /** @brief Constructor
       *
       * @param type A symbolic type name
       *
       * Create a new instance withe as a full interval of type @e type
       */
      explicit IntervalDomain(TREX::utils::Symbol const &type)
	:BasicInterval(type), m_lower(minus_inf), m_upper(plus_inf) {}
      /** @brief Constructor
       * @param type A symbolic type name
       * @param lb A bound
       * @param ub A bound
       *
       * Create a new instance of type @e type covering the interval
       * [lb, ub]
       *
       * @pre [lb, ub] is a valid interval (ie not empty)
       * @throw EmptyDomain the resulting domain is empty
       */
      IntervalDomain(TREX::utils::Symbol const &type, 
		     bound const &lb, bound const &ub)
	:BasicInterval(type), m_lower(lb), m_upper(ub) {
	if( m_upper<m_lower || ( m_lower.isInfinity() 
				 && m_upper==m_lower ) ) {
	  throw EmptyDomain(*this, "interval domain is empty");
	}
      }
      /** @brief Constructor
       * @param type A symbolic type name
       * @param val A value
       *
       * Create a new instance of type @e type including only the value
       * @e val. in other terms the domain is represented by the interval
       * [val, val]
       */
      IntervalDomain(TREX::utils::Symbol const &type, Ty const &val) 
	:BasicInterval(type), m_lower(val), m_upper(val) {}
      /** @brief Destructor */
      virtual ~IntervalDomain() {}
      
      /** @brief Check for inclusion
       * @param val A value
       *
       * This method is used to check if @a val is part of the domain
       * @retval true if @val is member of te domain
       * @retval false otherwise
       */
      bool contains(Ty const &val) const {
	return val==closestTo(val);
      }

      /** @brief Closest value
       * @param val A value
       *
       * This method is a simplae utility that gives the value of the domain
       * which is the closest to @a val.
       *
       * It is generally iused when one want to get a singleton value even
       * if the domain is not a singleton
       *
       * @retval lowerBound() if @a val @c< @c lowerBound()
       * @retval upperBound() if @a val @c> @c upperBound()
       * @retval val otherwise
       */
      Ty const &closestTo(Ty const &val) const {
	bound me(val);
	
	if( m_lower>me )
	  return m_lower.value();
	if( m_upper<me ) 
	  return m_upper.value();
	return val;
      }

      bool intersect(DomainBase const &other) const;
      bool equals(DomainBase const &other) const;
      /** @brief Restrict domain possible values
       * @param lo minimum value
       * @param hi maximum value
       *
       * This method restrict the domain of current instance to tits subset
       * included in [lo, hi]
       * 
       * @pre [lo, hi] is a valid interval
       * @pre this domain intersect [lo, hi]
       * @return this domain after the operation
       * @throw EmptyDomain resulting domain is empty
       */
      DomainBase &restrictWith(bound const &lo, bound const &hi);
      DomainBase &restrictWith(DomainBase const &other);

      /** @brief interval lower bound
       *
       * @return the lower bound of the interval for this domain
       * @sa bound const &upperBound() const
       * @sa void getBounds(bound &, bound&) vonst
       */
      bound const &lowerBound() const {
	return m_lower;
      }
      /** @brief interval upper bound
       *
       * @return the upper bound of the interval for this domain
       * @sa bound const &lowerBound() const
       * @sa void getBounds(bound &, bound&) vonst
       */
      bound const &upperBound() const {
	return m_upper;
      }

      bool isSingleton() const {
	return m_lower==m_upper;
      }

      bool hasLower() const {
	return !m_lower.isInfinity();
      }
      bool hasUpper() const {
	return !m_upper.isInfinity();
      }

      boost::any getLower() const {
	return m_lower.value();
      }
      boost::any getUpper() const {
	return m_upper.value();
      }
      
      /** @brief Get interval bounds
       * @param lo a reference to a bound variable
       * @param hi a reference to a bound variable
       *
       * Store the bounds of this interval in lo and hi.
       *
       * @pre lo and hi do not refer to the same varaiable
       * @post lo stores the lower bound value of this interval
       * @post hi stores the upper bound value of this interval
       *
       * @sa bound const &lowerBound() const
       * @sa bound const &upperBound() const
       */
      void getBounds(bound &lo, bound &hi) const {
	lo = m_lower;
	hi = m_upper;
      }

    protected:
      /** @brief Copy constructor
       * @param other another instance
       *
       * Create a copy of @a other
       */
      IntervalDomain(IntervalDomain const &other)
	:BasicInterval(other), m_lower(other.m_lower), m_upper(other.m_upper) {}
      /** @brief XML parsing constructor
       *
       * @param node An XML node
       * 
       * Parse an XML node to extract the interval domain.
       * The expected format is :
       * @code
       * <type min="<val>" max="<val>"/>
       * @endcode
       * @c min and @c max are optional and are set by default to either
       * @c -inf or @c +inf. The format of @c @<val@> is the classic @a Ty
       * text format or @c [-+]?inf.
       *
       * @throw TREX::utils::bad_string_cast One of the attributes is
       * not correctly formatted
       * @throw TREX::transaction::EmptyDomain the resulting interval is empty
       */
      explicit IntervalDomain(rapidxml::xml_node<> const &node)
	:BasicInterval(node), m_lower(minus_inf), m_upper(plus_inf) {
	completeParsing(node);
      }
 
    private:
      /** @brief interval lower bound */
      bound m_lower;
      /** @brief interval upper bound */
      bound m_upper;

      void parseLower(std::string const &val);
      void parseUpper(std::string const &val);
      std::ostream &print_lower(std::ostream &out) const {
	return out<<m_lower;
      }
      std::ostream &print_upper(std::ostream &out) const {
	return out<<m_upper;
      }
    }; // IntervalDomain<>

# define In_H_IntervalDomain
#  include "bits/IntervalDomain.tcc"
# undef In_H_IntervalDomain

  }
}
#endif // H_IntervalDomain
