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
#ifndef H_Observation
# define H_Observation

# include "Predicate.hh"

namespace TREX {
  namespace transaction {

    /** @brief Reactor observation
     *
     * This class encapsulate the latest observation on a timeline. Each time a timeline
     * value changes/is updated the reactor owning this timeline is producing this observation
     * which in turn is dispatched to all the reactors that observe the corresponding timeline.
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
      Observation(boost::property_tree::ptree::value_type &node) 
	:Predicate(node) {}
      /** brief Destructor */
      ~Observation() {}
    private:
      TREX::utils::Symbol const &getPredTag() const;

    }; // TREX::transaction::Observation

    /** @brief An observation id
     * 
     * A type (not yet) used to refer to a specific observation. 
     * This class is part of a long term plan t orefactor observations
     * in a similar way goals are manipulated in ucrrent trex version. 
     * While there is not plan yet on when this will be done, we do 
     * believe  that this refactoring would be benefitial for further 
     * evolutions of the framework such as allowing to post/refine 
     * observations in the past.
     *
     * @relates class Observation
     * @sa goal_id
     */
    typedef boost::shared_ptr<Observation> observation_id;

  } // TREX::transaction
} // TREX

#endif // H_Observation
