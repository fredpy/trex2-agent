/** @file agent/base/agent_graph.hh
 *
 * @brief utilities for TREX agen reactor graph management
 *
 * This files provides simple utilities for manipulating a transaction graph
 * inside an agent.
 *
 * @sa class TREX::transaction::graph
 *
 * @author Frederic Py <fpy@mbari.org>
 * @ingroup agent
 */
#ifndef H_agent_graph
# define H_agent_graph

# include "Agent_fwd.hh"
# include "TeleoReactor.hh"

# include <boost/graph/depth_first_search.hpp>

namespace boost {

  /** @brief Boost bgraph traits for the Agent
   *
   * This traits justr specify tto  the boost graph library that
   * a TREX::agent::Agent is also graph like which is manipulated like its mother
   * class TREX::transaction::graph
   *
   * @relates class TREX::agent::Agent
   * @ingroup agent
   */
  template<>
  struct graph_traits<TREX::agent::Agent> 
    :public graph_traits<TREX::transaction::graph> {};

} // boost

namespace TREX {
  namespace agent {

    /** @brief Cyclic dependency exception
     *
     * EXcveption thrown when the agent dtects a cycli dependcy between
     * two reactors of its graph
     *
     * Such cyclic dependency cannot be allowed in an agent
     * as it implies that both reactors require the other to complete
     * its synchronization in order to complete their. Resulting on
     * a chicken and egg problem during xynchronization which can only
     * be reolved by computing a fixed point for synchronization of both
     * @p a and @p b which is not even guraranteed to converge.
     * 
     * @author Frederic Py
     * @relates class Agent
     * @ingroup agent
     */
    class CycleDetected :public AgentException {
    public:
      /** @brief Constructor
       * @param[in] g A graph
       * @param[in] a A reactor
       * @param[in] b A reatcor
       *
       * Create a new instance stating that a dependency cycle has
       * been detected between the tow reactors @p a and @p b in
       * the graph @p g
       */
      CycleDetected(TREX::transaction::graph const &g,
		    TREX::transaction::graph::reactor_id a,
		    TREX::transaction::graph::reactor_id b) throw() 
	:AgentException(g, "Cyclic dependency between reactor \""+
			a->getName().str()+"\" and \""+
			b->getName().str()+"\".") {}
      /** @brief Destructor */
      ~CycleDetected() throw() {}
    };

    /** @brief Cycle detector
     *
     * This class is the base class used by a TREX agent to traverse all
     * its reactors.
     *
     * It ensures that a CyclicDependency extception will be thrown if
     * a cyclic dependency is identified during the reactors traversal
     *
     * @author Frederic Py <fpy@mbari.org>
     * @ingroup agent
     */
    class CycleDetector :public boost::dfs_visitor<> {
    public:
      /** @brief Default constructor */
      CycleDetector() {}
      /** @brief Copy constructor
       *
       * @param[in] other Another instance
       */
      CycleDetector(CycleDetector const &other) 
	:boost::dfs_visitor<>(other) {}
      /** @brief Destructor */
      virtual ~CycleDetector() {}

      /** @brief back edge callback
       *
       * This dfs callback is called only when the dfs loops back to a vertex
       * that is still being explored. This clearly idenitfy a loop in the
       * graph wwhich is a proof of cyclic dependncy betwee two reactors
       *
       * @throw CycleDetected always thrown as this call indicates a cyclic
       *        dependency in the egent graph
       */
      void back_edge(TREX::transaction::graph::relation_type const &rel,
		     TREX::transaction::graph const &g) {
	throw CycleDetected(g, boost::source(rel, g), boost::target(rel, g));
      }
      
			   
    protected:
      bool is_valid(TREX::transaction::graph::reactor_id r) const {
	return TREX::transaction::graph::null_reactor()!=r;
      }
      
    private:

    }; // TREX::agent::CycleDetector

  } // TREX::agent
} // TREX

#endif // H_agent_graph
