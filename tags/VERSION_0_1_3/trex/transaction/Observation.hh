/** @file "Observation.hh"
 * @brief TREX observation definition
 *
 * This file provides interfaces and utilities to create and manipulate
 * observations exchanged between reactors.
 *
 * @deprecated the current implementation of observations is not as flexible
 * and transparent as desired and may change drastically in the future
 *
 * @author Frederic Py <fpy@mbari.org>
 * @ingroup transaction
 */ 
#ifndef H_Observation
# define H_Observation

# include "Predicate.hh"

namespace TREX {
  namespace transaction {

    /** @brief Reactor observation
     *
     * This class encapsulate the latest observation on a timeline. Each time a timeline
     * value changes/is updated the reactor owning this timeline is producing tis observation
     * which in turn is dispatched to all the reactors tha observe the corresponding timeline.
     *
     * @sa void TeleoReactor::postObservation(Observation const &obs)
     * @sa void TeleoReactor::notify(Observation const &obs)
     *
     * @deprecated the current implementation of observations is not as flexible
     * and transparent as desired and may change drastically in the future
     *
     * @author Frederic Py <fpy@mbari.org>
     * @ingroup transaction
     */
    class Observation :public Predicate {
    public:
      /** @brief Copy constructor
       * @param pred A predicate
       *
       * Create a copy of pred as an observation
       */
      Observation(Predicate const &pred)
	:Predicate(pred) {}
      /** @brief Constructor
       * @param obj A timline identifier
       * @param pred   A predicate name
       *
       * Create a new instance connected to timeline @a object with the
       * state @a pred and all its attributes not constrained.
       */
      Observation(TREX::utils::Symbol const &obj,
		  TREX::utils::Symbol const &pred)
	:Predicate(obj, pred) {}
      /** @brief XML parsing constructor
       * @param node An XML node
       *
       * Create a new instance by parsing the content of @a node. The
       * expected format of node is
       * @code
       * <Observation on="<name>" pred="<name>">
       *   <!-- attribute definition -->
       * </Observation>
       * @endcode
       *
       * @throw PredicateException Error while trying to parse
       * @throw EmptyDomain A domain became empty during parsing
       * @throw XmlError Another error while parsing
       * @sa Variable::Variable(rapidxml::xml_node<> const &)
       * @sa Predicate::Predicate(rapidxml::xml_node<> const &)
       */
      Observation(rapidxml::xml_node<> const &node) 
	:Predicate(node) {}
      /** brief Destructor */
      ~Observation() {}
    private:
      TREX::utils::Symbol const &getPredTag() const;

    }; // TREX::transaction::Observation

    typedef boost::shared_ptr<Observation> observation_id;

  } // TREX::transaction
} // TREX

#endif // H_Observation
