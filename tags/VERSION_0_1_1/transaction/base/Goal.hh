/** @file "Goal.hh"
 * @brief TREX goal definition
 *
 * This file provides interfaces and utilities to create and manipulate
 * goals exchanged between reactors.
 *
 * @deprecated the current implementation of goals is not as flexible
 * and transparent as desired and may change drastically in the future
 *
 * @author Frederic Py <fpy@mbari.org>
 * @ingroup transaction
 */ 
#ifndef H_Goal
# define H_Goal

#include "Predicate.hh"
#include "Tick.hh"

namespace TREX {
  namespace transaction {

    /** @brief TREX goal
     *
     * This class represent a goal as exchanged between reactors a goal
     * is a predicate with the 3 temporal varaiables @c start, @c duration
     * and @c end. It is in substance what is called a Token inside the
     * EUROPA planning framework.
     *
     * This class embeds also suficient code to ensure that the @c duration
     * is at least 1 TICK and that any modification of one of the temporal
     * attributes is propagated to the others so @c start+duration=end is
     * maintained
     *
     * @author Frederic Py <fpy@mbari.org>
     * @ingroup transaction
     *
     * @depreacted need to be extended to embed representation from 3rd
     * party planners without copy so it improve efficiency.
     */
    class Goal :public Predicate {
    public:
      /** @brief Copy constructor
       * @param[in] other Another instance
       *
       * Create a copy of @a other
       */
      Goal(Goal const &others);
      /** @brief Constructor
       * @param[in] object A timeline identifier
       * @param[in] pred   A predicate name
       *
       * Create a new instance connected to timeline @a object with the
       * state @a pred and all its attributes not constrained
       */
      Goal(TREX::utils::Symbol const &object,
	   TREX::utils::Symbol const &pred);
      /** @brief XML parsing constructor
       * @param[in] node An XML node
       *
       * Create a new instance by parsing the content of @a node. The
       * expected format of node is
       * @code
       * <Goal on="<name>" pred="<name>">
       *   <!-- attribute definition -->
       * </Goal>
       * @endcode
       * If @c start, @c duration or @c end varaiables are defined it
       * checks that  their domains are properly defined and propageated.
       *
       * @throw PredicateException Error while trying to parse
       * @throw EmptyDomain A domain became empty during parsing
       * @throw XmlError Another error while parsing
       * @sa Variable::Variable(rapidxml::xml_node<> const &)
       * @sa Predicate::Predicate(rapidxml::xml_node<> const &)
       */
      Goal(rapidxml::xml_node<> const &node);
      /** @brief destructor */
      ~Goal() {}

      /** @brief Dispatchability test
       * @param[in] date  current date
       * @param[in] delay A delay
       *
       * Thsi method is used by TeleoReactor class to check if a goal on
       * an external timeline needs to be dispatched or should be kept
       * for now. It can also constraint the start time of this goaal to
       * be after @a date if possible
       *
       * @retval true if the domain of @c start intersects
       * @c [start+delay,@c +inf). In this case the @c start time is
       * constrained to be necessarily greater or equal to @a date
       * @retval false otherwise 
       */
      bool startsAfter(TICK date, TICK delay =0);
      /** @brief Check for possible early start
       * @param[in] date A date
       *
       * @retval true if @c start can be less of equal to @a date
       * @retval false otherwise
       */
      bool startsBefore(IntegerDomain::bound const &date) const {
	return getStart().lowerBound()<=date;
      }

      /** @brief Get start interval
       *
       * Identifies the domain of possible start TICK.
       *
       * @note This function is strictly equivalent to :
       * @code
       * getAttribute(s_startName).typedDomain<IntegerDomain>()
       * @endcode
       *
       * @return the domain for the start domain of this goal
       *
       * @sa getDuration() const
       * @sa getEnd() const
       * @sa getAttribute() const 
       */
      IntegerDomain const &getStart() const;
      /** @brief Get duration interval
       *
       * Identifies the domain of possible furations 
       *
       * @note This function is strictly equivalent to :
       * @code
       * getAttribute(s_durationName).typedDomain<IntegerDomain>()
       * @endcode
       * @note As of today the duration of a token in trex is at least of 1 TICK
       *
       * @return the domain for the start domain of this goal
       *
       * @sa getStart() const
       * @sa getEnd() const
       * @sa getAttribute() const 
       */
      IntegerDomain const &getDuration() const;
      /** @brief Get end  interval
       *
       * Identifies the domain of possible end TICK.
       *
       * @note This function is strictly equivalent to :
       * @code
       * getAttribute(s_endName).typedDomain<IntegerDomain>()
       * @endcode
       *
       * @return the domain for the start domain of this goal
       *
       * @sa getDuration() const
       * @sa getEnd() const
       * @sa getAttribute() const 
       */
      IntegerDomain const &getEnd() const;
      
      /** @brief restrict one of the Goal attributes
       *
       * @param[in] var A variable
       *
       * Add restrict the attribute @p var.name() to the domain @p var.domain()
       *
       * @throw PredicateException Restricting the Goal attribute using @p var result
       *        on an empty domain
       * @note If an exception is thrown the domain is not changed
       * @note if var refers to one temporal varaibale (@c start, @c duration or
       *        @c end) the method restrictTime will be called instead
       *
       * @sa restrictTime(IntegerDomain const &, IntegerDomain const &, 
       *                  IntegerDomain const &)
       */
      void restrictAttribute(Variable const &var);
      Variable const &getAttribute(TREX::utils::Symbol const &name) const;
      void listAttributes(std::list<TREX::utils::Symbol> &attr,
			  bool all) const;
      
      /** @brief name of start attribute */
      static TREX::utils::Symbol const s_startName;
      /** @brief name of duration attribute */
      static TREX::utils::Symbol const s_durationName;
      /** @brief name of end attribute */
      static TREX::utils::Symbol const s_endName; 

      /** @brief Restrict time variables
       *
       * @param[in] s A start domain
       * @param[in] d A duration domain
       * @param[in] e A end domain
       *
       * This method restrict the token temporals variables (@c start restricted
       * by @p s, @c duration restricted by @p d and @c end restricted by @p e) and
       * also progate the constraint such as @c start+duration=end.
       *
       * @throw PredicateException One of the temporal variables domain would
       *        become empty if the operation is applied.
       *
       * @note If an exception is thrown the domain is not changed
       *
       * @post the temporal variables have been restrictefd accordingly
       *
       * @sa restrictAttribute(Variable const &)
       */
      void restrictTime(IntegerDomain const &s,
			IntegerDomain const &d,
			IntegerDomain const &e);
      /** @brief Restrict start time domain
       *
       * @param[in] s A domain
       *
       * This method overloads restrictTime by requiring only a domain for
       * the @c start domain as argument
       *
       * @throw PredicateException applying this constraint would result on an
       *        empty domain
       *
       * @sa restrictTime(IntegerDomain const &, IntegerDomain const &, 
       *                  IntegerDomain const &)
       */
      void restrictStart(IntegerDomain const &s) {
	restrictTime(s, s_durationDomain, s_dateDomain);
      }
      /** @brief Restrict duration domain
       *
       * @param[in] d A domain
       *
       * This method overloads restrictTime by requiring only a domain for
       * the @c duration domain as argument
       *
       * @throw PredicateException applying this constraint would result on an
       *        empty domain
       *
       * @sa restrictTime(IntegerDomain const &, IntegerDomain const &, 
       *                  IntegerDomain const &)
       */
      void restrictDuration(IntegerDomain const &d) {
	restrictTime(s_dateDomain, d, s_dateDomain);
      }
      /** @brief Restrict end domain
       *
       * @param[in] e A domain
       *
       * This method overloads restrictTime by requiring only a domain for
       * the @c end domain as argument
       *
       * @throw PredicateException applying this constraint would result on an
       *        empty domain
       *
       * @sa restrictTime(IntegerDomain const &, IntegerDomain const &, 
       *                  IntegerDomain const &)
       */
      void restrictEnd(IntegerDomain const &e) {
	restrictTime(s_dateDomain, s_durationDomain, e);
      }
    private:
      Variable m_start, m_duration, m_end;
      static IntegerDomain const s_dateDomain;
      static IntegerDomain const s_durationDomain;
      
      TREX::utils::Symbol const &getPredTag() const;
    };

    /** @brief A goal identifier
     *
     * This is the type used by TREX to identify and manipulate goals
     *
     * @ingroup transaction
     * @relates class Goal
     */
    typedef boost::shared_ptr<Goal> goal_id;

  } // TREX::transaction
} // TREX

#endif // H_Goal