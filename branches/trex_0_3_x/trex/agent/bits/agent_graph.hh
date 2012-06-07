/** @file trex/agent/agent_graph.hh
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
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, MBARI.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the TREX Project nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef H_agent_graph
# define H_agent_graph

# include "../Agent_fwd.hh"
# include <trex/transaction/TeleoReactor.hh>

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
			boost::get(boost::get(boost::vertex_name_t(), g), a).str()+"\" and \""+
			boost::get(boost::get(boost::vertex_name_t(), g), b).str()+"\".") {}
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
        if( is_valid(boost::source(rel, g), g) &&
            is_valid(boost::target(rel, g), g) )
          throw CycleDetected(g, boost::source(rel, g), boost::target(rel, g));
      }


    protected:
      /** @brief Check for node validity
       * @param[in] r A reactor reference
       * @param[in] g The graph supporting @p r
       *
       * Check that @p r referes to an exisiting reactor. This method is
       * used during graph traversal to check that the an edge is not pointing
       * to nothing or to a node that has been isolated
       *
       * @retval true if @p r refers to a reactor
       * @retval false otherwise
       *
       * @sa TREX::transaction::garph::null_reactor()
       */
      bool is_valid(TREX::transaction::graph::reactor_id r,
                    TREX::transaction::graph const &g) const {
	return TREX::transaction::graph::null_reactor()!=r &&  !g.is_isolated(r);
      }

    private:

    }; // TREX::agent::CycleDetector

  } // TREX::agent
} // TREX

#endif // H_agent_graph
