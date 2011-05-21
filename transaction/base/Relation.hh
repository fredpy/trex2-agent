#ifndef H_Relation
# define H_Relation

# include "TeleoReactor_fwd.hh"

# include <boost/iterator/iterator_facade.hpp>

namespace TREX {
  namespace transaction {

    /** @brief Exception for timeline ownership conflict
     *
     * This exception is udsed when more than one reactor attyempt to declare
     * a timeline as internal
     *
     * @relates class details::timeline
     *
     * @author Frederic Py <fpy@mbari.org>
     */
    class MultipleInternals :public ReactorException {
    public:
      /** @brief Constructor
       *
       * @param[in] faulty   The reactor that attempted to steal the timeline
       * @param[in] timeline The name of the timeline
       * @param[in] owner    The reactor that currenlty owns the timeline
       *
       * Create a new instance that indicates that @p faulty attempted to declare the
       * timeline named @p timeline while it is alread oewned by @p owner
       */
      MultipleInternals(TeleoReactor const &faulty, utils::Symbol const &timeline,
			TeleoReactor const &owner) throw();
      /** @brief Destructor */
      ~MultipleInternals() throw() {}
    }; // TREX::transaction::MultipleInternals

    namespace details {

      class timeline;

    }; // TREX::transaction::details

    /** @brief Timeline relation
     *
     * This class represent a single timline dependency link between 2
     * reactors. Such relation can be described as follow:
     * @li @c name() A timeline name
     * @li @c client() A reactor that declares this timeline as @e External
     * @li @c server() The reactor that declares this timeline as @e Internal
     *
     * When the relation is @c active() both the @c client() and @c server()
     * are defined. Otherwise one of the 2 -- usually the server -- is yet to
     * be assigned.
     *
     * This class plays a central role in TREX transaction management. It is
     * used as an edge of the graph that gives the full dependencies between
     * reactors and is strongly used by the agent in order to  properly
     * propagate observations and goals between reactors.
     *
     * It also provide a basic management on observation and goals dispatching
     * for each reactor that ensures basic properties required for TREX such as:
     * @li There can be only one Observation per timeline aper Tick
     * @li A goal can should be send to the server only when its start overlaps
     *     the reactor planning window
     *
     * Finally this class allows to flexibly change the server internally in order
     * to easily change reactors dependencies by the Agent (for example killing a
     * reactor, or creating a new reactor that provides missing timlines in  the
     * middle of agent lifetime).
     *
     * @author Frederic Py <fpy@mbari.org>
     * @ingroup transaction
     */
    class Relation {
    public:
      typedef Relation        value_type;
      typedef Relation const &reference;
      typedef Relation const *pointer;

      typedef Relation      base_type;
      typedef utils::Symbol id_type;

      /** @brief Get relation identifier
       *
       * @param[in] rel A relation
       *
       * This methods give the name of the timeline associated
       * to the relation @p rel
       *
       * @pre @p ral is valid
       *
       * @return the symbolic timeline name of @p rel
       *
       * @sa Relation::name() const
       * @sa relation::valid() const
       */
      static id_type const &get_id(base_type const &rel) {
	return rel.name();
      }
      
      /** @brief Default constructor
       *
       * Create a new empty relation
       *
       * @post this relation is not valid
       */
      Relation()
	:m_timeline(NULL) {}
      /** @brief Copy constructor 
       *
       * @param[in] other A relation
       *
       * Create a new relation identical to @p other
       */
      Relation(Relation const &other)
	:m_timeline(other.m_timeline), m_pos(other.m_pos) {}
      /** @brief Destructor */
      ~Relation() {}

      /** @brief Check if accept goal
       *
       * Indicates whether this relation is accepting goals or not.
       * This method is used by a reactor to identifies if it can
       * dispatch goals or not.
       *
       * Potential reasons a relation do not accept goal are:
       * @li the relation is not active
       * @li the look-ahead of the reactor owning the timeline is 0
       * @li the client for this relation declared it as an observation
       *     relation
       *
       * @retval true if the relation accept goals
       * @retval false else
       *
       * @sa Relation::client() const
       * @sa Relation::active() const 
       * @sa Relation::server() const
       */
      bool accept_goals() const;

      void recall(goal_id const &g);
      void request(goal_id const &g);
      
      /** @brief Relation latency
       * 
       * This methosd identifie s the timelien latency associated to
       * this relation. It is mostly a proxy to details::timeline::latency(). 
       *
       * If the timeline is not valid the latency is then set to 0
       *
       * @return the execution latency of this relation
       *
       * @sa details::timeline::latency() const
       * @sa valid() const
       * @sa look_ahead() const
       */
      TICK latency() const;
      /** @brief Timeline look ahead
       *
       * Ideintifies the look ahead associated to this relation. If the relation
       * does not accept goals this value will be 0 otherwise it is just a proxy
       * to timeline::look_ahead()
       *
       * @return the look ahead of this relation
       *
       * @sa accept_goals() const
       * @sa details::timeline::look_ahead() const
       */
      TICK look_ahead() const;
      
      /** @brief Dispatch window 
       *
       * @param[in] current A tick date
       *
       * Gives the time interval for which this elation will allow dispatch
       * goals at tick @p current. Any goal that has its start time potantially
       * starting in this window can be safely posted to this relation.
       *
       * @pre The relation accepts goals
       *
       * @throw EmptyDomain the relation does not accept goals
       *
       * @return the dispatch interval at @p current
       *
       * @sa accept_goals() const
       * @sa latency() const
       * @sa look_ahead() const
       */
      IntegerDomain dispatch_window(TICK current) const {
	TICK lb = current+latency();
	
	return IntegerDomain(lb, lb+look_ahead());
      }

      /** @brief Check for relation validity
       *
       * Checks if this relation is valid. A valid relation is
       * a relation which is associated to a timeline and have a
       * client.
       *
       * @retval true if the relation is valid
       * @retval false otherwise
       */
      bool valid() const;
      /** @brief Check if relation active
       *
       * An active relation is a relation that is both valid and is
       * associated to a timeline that have a server. In that case
       * this relation is an edge of the reactor relation graph.
       * 
       * @retval true if the relation is active
       * @retval false otherwise
       *
       * @sa Relation::valid() const
       * @sa details::timeline::active() const
       */
      bool active() const;
      
      /** @brief Date of last observation
       *
       * This method indicates at what tick was produced the
       * last observation on the timeline associated to this
       * relation
       *
       * @pre the relation is valid
       * @pre at least one observation has been produce on this timeline
       *
       * @return the tick during which was produced the last observation 
       *
       * @sa Relation::valid() const
       * @sa details::timeline::lastObsDate() const
       * @sa Relation::lastObservation() const
       */
      TICK               lastObsDate() const;
      /** @brief Last observation
       *
       * Gives the last observation produced for this timeline
       *
       * @pre the relation is valid
       * @pre at least one observation has been produce on this timeline
       *
       * @return the observation for this timeline
       *
       * @sa Relation::valid() const
       * @sa Relation::lastObsDate() const
       * @sa details::timeline::lastObservation() const
       */
      Observation const &lastObservation() const; 

      /** @brief client for this relation
       *
       * Indicates which reactor created this relation by declaring
       * the associated timeline  as external.
       *
       * @pre the relation is valid
       *
       * @return the client for this relation
       *
       * @sa Relation::valid() const
       */
      TeleoReactor &client() const {
	return *(m_pos->first);
      }
      /** @brief Timeline name
       *
       * Gives the name of the timeline associated to this relation.
       *
       * @pre the relation is valid
       *
       * @return the name of the timline for this relation
       *
       * @sa Relation::valid() const
       */
      utils::Symbol const &name() const;
      /** @brief Timeline server
       *
       * Gives the reactor that declared the timline associated to
       * this relation as internal.
       *
       * @pre the relation is active
       * 
       * @return a reference to the reactor owning this timeline
       *
       * @sa Relation::active() const
       * @sa details::timeline::owner() const
       */
      TeleoReactor &server() const;
      
      /** @brief Equality test
       *
       * @param[in] other another relation
       *
       * Check if this relation is identical to @p other.
       *
       * @retval true if this relation is identical to @p other
       * @retval false otherwise
       *
       * @sa operator!=(Relation const &) const
       */
      bool operator==(Relation const &other) const {
	return m_pos==other.m_pos;
      }
      /** @brief Difference test
       *
       * @param[in] other another relation
       *
       * Check if this relation is not identical to @p other.
       *
       * @retval false if this relation is identical to @p other
       * @retval true otherwise
       *
       * @sa operator==(Relation const &) const
       */
      bool operator!=(Relation const &other) const {
	return !operator==(other);
      }

      /** @{ 
       * @brief Access operator 
       *
       * These operators are implemented so the Relation clas can
       * be used as an iterator
       */
      pointer operator->() const {
	return this;
      }
      reference operator*() const {
	return *operator->();
      }
      /** @} */
      /** @brief advance to next Relation
       *
       * Change this instance to point to the next relation.
       * Indeed the relation class is just an iterator of all
       * the relations for a given timeline in disguise. 
       *
       * @pre the relation is valid
       *
       * @note this method is used internally by TREX in order to traverse
       * the relation graph between reactors. It should not be used outside
       * of details::timeline class and other core compoenents
       *
       * @return this instance after the operation
       *
       * @sa details::timeline::begin() 
       * @sa details::timeline::end() 
       */
      Relation &operator++() {
	++m_pos;
	return *this;
      }

    private:
      /** @brief Constructor
       *
       * @param[in] tl  A timeline
       * @param[in] pos An iterator position
       *
       * This constructor is called by details::timeline class when
       * a reactor subscribe to this timeline. 
       *
       * @sa details::timeline::subscribe() const
       */
      Relation(details::timeline *tl,
	       details::client_set::iterator const &pos)
	:m_timeline(tl), m_pos(pos) {}

      void unsubscribe() const;

      details::timeline            *m_timeline;
      details::client_set::iterator m_pos;

      friend class details::timeline;
      friend class TeleoReactor;
    }; // TREX::transaction::Relation     

  } // TREX::transaction
} // TREX 

#endif // H_Relation
