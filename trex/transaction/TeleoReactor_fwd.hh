/** @file TeleoReactor_fwd.hh
 * @brief Forward decalrions for TeleoReactor
 *
 * This file declares some basic lasses that are used for TeleoReactor
 * manipulation.
 *
 * Many of these declarations are incomplete and just used in order to refer
 * to certain objects without manipulating them. For a more complete definition
 * use TeleoReactor.hh
 *
 * @author  Frederic Py <fpy@mbari.org>
 * @ingroup transaction
 * @sa TeleoReactor.hh
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
#ifndef FWD_TeleoReactor
# define FWD_TeleoReactor

# include "bits/transaction_fwd.hh"

# include "Observation.hh"
# include "Goal.hh"
# include <trex/utils/id_mapper.hh>

namespace TREX {
  namespace transaction {

    class TeleoReactor;
    class graph;

    /** @brief Graph Exception
     *
     * An exception due to a graph related issue
     *
     * @relates graph
     * @author Frederic Py <fpy@mbari.org>
     * @ingroup transaction
     */
    class GraphException :public TREX::utils::Exception {
    public:
      GraphException(graph const &g, std::string const &msg) throw();
      virtual ~GraphException() throw() {}

    protected:
      GraphException(graph const &g, std::string const &who,
		     std::string const &msg) throw();
    }; // TREX::transaction::GraphException


    /** @brief TeleoReactor related exception
     *
     * An exception produced while manipulating a reactor.
     *
     * @relates TeleoReactor
     * @author  Frederic Py <fpy@mbari.org>
     * @ingroup transaction
     */
    class ReactorException :public GraphException {
    public:
      /** @brief Constructor
       *
       * @param[in] r    The source of the exception
       * @param[in] msg  The associated message
       *
       * Create a new instance with an ossociated message that indicates that
       * this exception is due to the TeleoReactor @p r and that the error can
       * be described as @p msg.
       */
      ReactorException(TeleoReactor const &r, std::string const &msg) throw();
      /** @brief Destructor */
      virtual ~ReactorException() throw() {}

    }; // TREX::transaction::ReactorException

    /** @brief Implementation details for transaction
     *
     * This namespace embed all the components that support the implementation
     * of TREX transaction between reactors.
     *
     * @warn The components in this namespace should be considered as a private
     *       part of the framework. They are documented mostly for "educational"
     *       purpose. Any 3rd part developper that uses one of these exposes
     *       himself to have his code broken in future version.
     *
     * @author  Frederic Py <fpy@mbari.org>
     * @ingroup transaction
     */
    namespace details {

      /** @brief ID traits for TeleoReactor
       *
       * Gives a way to acces to the name of a reactor
       * in order to store them in a TREX::utils::list_set
       *
       * @author Frederic Py <fpy@mbari.org>
       * @ingroup transaction
       */
      template<class PtrType = TeleoReactor *>
      struct reactor_id_traits {
	typedef PtrType base_type;
	typedef utils::Symbol id_type;

	/** @brief ID extraction function
	 *
	 * @param[in] A reference to a TeleoReactor
	 *
	 * Extract the name of the reactor @p r to be used as a
	 * sorting identifier
	 *
	 * @pre @p r is not @c NULL
	 *
	 * @sa class TREX::utils::list_set
	 *
	 * @return The name of @p r
	 * @relates reactor_set
	 */
	static id_type const &get_id(base_type r) {
	  return r->getName();
	}
      }; // TREX::transaction::details::reactor_id_traits

      typedef reactor_id_traits<>::base_type reactor_id;


      /** @brief A set of reactors
       *
       * This type stores reactors in an list ordered by reactor names
       *
       * @sa class TREX::utils::list_set
       */
      typedef utils::list_set<
	reactor_id_traits< boost::shared_ptr<TeleoReactor> > > reactor_set;


      /** @brief timeline client helper
       *
       * An helper used to define a set of timeline clients.
       * A timeline client defintion is a std::pair where the
       * @c first element is a pointer to the TeleoReactor that declare
       * the timeline as external and the @c second element is a boolean
       * flag indicating if this reactor expects to post goals to this
       * timeline
       *
       * This is used by the relation class in order to identifies
       * if this relation will be used to pass goals to the owner of
       * the timeline
       *
       * @note Having the boolean flag to @c true do not guaranttee that
       *       this relation will accept goals. Indeed if the owner of
       *       the timeline have a look-ahed of @c 0 or there's no owner
       *       for this timeline, the relation will not accept goals
       *       either
       *
       * @sa class TREX::utils::relation
       * @sa TREX::utils::relation::accept_goals() const
       */
      typedef utils::map_id_traits<reactor_id_traits<>, transaction_flags> client_id_traits;

      /** @brief Set of clients
       *
       * This type is usded to store and maintin the set of client to a timeline.
       * The clients of a timline are the reactors that declared it as external
       *
       * @ingroup transaction
       * @relates class timeline
       */
      typedef utils::list_set<client_id_traits> client_set;

    } // TREX::transaction::details

  } // TREX::transaction
} // TREX

#endif // FWD_TeleoReactor
