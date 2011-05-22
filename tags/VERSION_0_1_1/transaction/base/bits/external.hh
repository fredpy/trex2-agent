/** @file transaction/base/bits/external.hh
 * @brief external timeline external utilities
 * 
 * This file provide definition of diverse internal REX utilities in order to 
 * manipulate an represent external timelines dependencies of a reactor.
 * 
 * @ingroup transaction
 * @author Frederic Py <fpy@mbari.org>
 */
#ifndef H_BITS_external
# define H_BITS_external

# include "Goal.hh"
# include "timeline.hh"

# include <boost/iterator_adaptors.hpp>

namespace TREX {
  namespace transaction {
    namespace details {
      
      /** @brief A goal queue
       *
       * The basic container used by external class to maintain and
       * process the set of goals being posted by a reactor and yet to
       * be dipatched to the owner of this timeline
       *
       * @ingroup transaction
       * @relates class external
       */
      typedef std::list<goal_id>                           goal_queue;
      /** @brief A external timeline proxy
       *
       * This type by a external class to represent an external timeline
       * relation and its corresponding set of pending goals
       *
       * @ingroup transaction
       * @relates class external
       */
      typedef utils::map_id_traits< Relation, goal_queue > pending_traits;
      /** @brief external timelines
       *
       * The type used by a TeleoReactor to maintin and manage the set of all its
       * @p External relation
       *
       * Most of this management is done by the external class that acts as
       * an iterator other this type with extra functionalities
       *
       * @ingroup transaction
       * @relates TeleoReactor
       * @sa class external
       */
      typedef utils::list_set<pending_traits>              external_set;
      
      /** @brief An external timeline
       *
       * This class is represent a reactor external timeline declaration
       * It refers to the corresponding relation and also offer some
       * management on the goals that have been posted to this timeline
       * and when they should be dispatched to the ownere of this timeline
       *
       * @ingroup transaction
       * @relates class TeleoReactor
       * @sa class Relation
       *
       * @author Frederic Py <fpy@mbari.org>
       */
      class external 
	:public boost::iterator_adaptor<external, external_set::iterator>{
      public:
	typedef Relation::value_type   value_type;
	typedef Relation::reference    reference;
	typedef Relation::pointer      pointer;

	/** @brief Default constructor
	 *
	 * Create a new instance 
	 *
	 * @post the instance is not valid
	 *
	 * @sa valid() const
	 */
	external();
	/** @brief Copy constructor
	 *
	 * @param[in] other Another instance
	 *
	 * Create a copy of @p other
	 */
	external(external const &other);
	/** @brief Destructor */
	~external() {}

	/** @brief Equality test
	 *
	 * @param[in] other Another instance
	 *
	 * @retval true if @p other is identical to this instance
	 * @retval false otherwise
	 *
	 * @note invalid instances are considered as identical
	 *
	 * @sa operator!=(external const &) const
	 * @sa valid() const
	 */
	bool operator==(external const &other) const;
	/** @brief Difference test
	 *
	 * @param[in] other Another instance
	 *
	 * @retval false if @p other is identical to this instance
	 * @retval true otherwise
	 * 
	 * @note invalid instances are considered as identical
	 *
	 * @sa operator==(external const &) const
	 * @sa valid() const
	 */
	bool operator!=(external const &other) const {
	  return !operator==(other);
	}
	/** @brief Check for validity
	 *
	 * Check if this instance referes to an exisiting relation
	 *
	 * @retval true if this instance point to an exisiting relation
	 * @retval false otherwise
	 */
	bool valid() const {
	  return m_last!=m_pos;
	}

	/** @brief Goal posting
	 *
	 * @param[in] g A goal
	 *
	 * @pre This instance is valid
	 * @pre the object of @p g is the same as the timeline referred by
	 *       this instance
	 *
	 * Add the goal to this instance pending goal queue. This goal will then
	 * be processed during subsequent calls of the dispatch method
	 *
	 * @retval true @p g has been added to the goal queue
	 * @retval false @p g was already present in the goal queue
	 *
	 * @warning the preconditions described above are not checked here (as they
	 *       should be checked by TeleoReactor class). Caling this method
	 *       without controlling these will result on an unknown behavior of
	 *       the system
	 *
	 * @sa valid() const
	 * @sa operator->() const
	 * @sa Relation::name() const
	 * 
	 * @sa dispatch(TICK, goal_queue &)
	 * @sa recall(goal_id const &)
	 */
	bool post_goal(goal_id const &g);
	/** @brief Goal dispatching management
	 *
	 * @param[in] current The current tick
	 * @param[out] sent   List of goals being dispatched
	 *
	 * @pre This instance is valid
	 *
	 * This method is called when a the tick value has just been updated to
	 * @p current. It will update the pending goals queue of this timeline
	 * and duipatch to the owner of this timeline all the goals that can be
	 * dispatched at @p current. Conversely it will remove the goals on the
	 * pending queue that necessarily startes before @p current and indicate
	 * such operation in TREX.log
	 *
	 * All the goals dispatched during this call will be added to @p sent and
	 * removed from the pending queue
	 *
	 * @sa valid() const
	 */
	void dispatch(TICK current, goal_queue &sent);
	/** @brief Recall a goal
	 *
	 * @param[in] g A goal
	 *
	 * @pre This instance is valid
	 *
	 * recall the goal @p g. If this goal is not in the pending queue a
	 * recall notification will be sent to the oewner of this timeline
	 *
	 * @sa valid() const
	 * @sa post_goal(goal_id const &)
	 */
	void recall(goal_id const &g);
	
	/** @brief Relation access
	 *
	 * @pre This instance is valid
	 *
	 * Gives access to the timeline relation this instance refers to
	 *
	 * @sa class Relation
	 * @sa operator* () const
	 * @sa valid() const
	 */
	pointer operator->() const {
	  return &operator* ();
	}
	/** @brief Relation access
	 *
	 * @pre This instance is valid
	 *
	 * Gives access to the timeline relation this instance refers to
	 *
	 * @retval A reference to the Relation referred by this instance 
	 *
	 * @sa class Relation
	 * @sa operator* () const
	 * @sa valid() const
	 */
	reference operator* () const {
	  return m_pos->first;
	}

	/** @brief Next external
	 *
	 * Advance this instance to the next Relation
	 *
	 * @return This instance after the operation
	 */
	external &operator++();

      private:
	TREX::utils::internals::LogEntry syslog();

	external_set::iterator m_pos, m_last;

	external(external_set::iterator const &pos,
		 external_set::iterator const &last);


	static bool cmp_goals(IntegerDomain const &a, IntegerDomain const &b);
	
	void next_active();

	goal_queue::iterator lower_bound(IntegerDomain const &dom);

	friend class TREX::transaction::TeleoReactor;
      }; // TREX::transaction::details::external
          
    } // TREX::transaction::details
  } // TREX::transaction
} // TREX

#endif // H_BITS_external
