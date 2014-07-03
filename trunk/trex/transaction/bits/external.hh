/** @file trex/transaction/bits/external.hh
 * @brief external timeline external utilities
 * 
 * This file provide definition of diverse internal REX utilities in order to 
 * manipulate an represent external timelines dependencies of a reactor.
 * 
 * @ingroup transaction
 * @author Frederic Py <fpy@mbari.org>
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
#ifndef H_BITS_external
# define H_BITS_external

# include <trex/domain/token.hh>
# include "timeline.hh"

# include <boost/iterator_adaptors.hpp>
# include <boost/iterator/filter_iterator.hpp>

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
      typedef std::list< std::pair<token_ref, bool> >  goal_queue;
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
	:public boost::iterator_facade<external, Relation const, 
				       boost::forward_traversal_tag>{
      public:        
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

	/** @brief Check if active
	 * 
	 * Checks that this external iterator is both valid and points to an active relation
	 * 
	 * @sa external::valid() const
	 * @sa Relation::active() const
	 */
	bool active() const {
	  return valid() && dereference().active();
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
        bool post_goal(token_id const &g);
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
        void recall(token_id const &g);

	/** @brief releas all blocked goals 
	 *
	 * Mark all ther bgoal that wer blocked by an exception as free 
	 * for dispatch. This is done whenever the timeline is owned by 
	 * a new reactor
	 */
	void unblock();
                 
      private:
        utils::log::stream syslog(utils::log::id_type const &kind);
        
        external_set::iterator m_pos, m_last;
        
        external(external_set::iterator const &pos,
                 external_set::iterator const &last);
        
        
        static bool cmp_goals(int_domain const &a, int_domain const &b);
        
        goal_queue::iterator lower_bound(int_domain const &dom);

	void increment();
	bool equal(external const &other) const;
	Relation const &dereference() const;
        
        friend class TREX::transaction::reactor;
	friend class boost::iterator_core_access;
      }; // TREX::transaction::details::external
      
      /** @brief Active external predicate
       *
       * This class is a functor acting as a predicate that checks if
       * a @ external instance refers to an active timeline relation.
       *
       * It is used to implement the @c active_external class, which 
       * using a boost filter_iterator on the @c external class in order
       * to reproduce the past behavior of this same @c external which used
       * to only iterate through active relations.
       *
       * @sa external::active() const
       * @sa Relation::active() const
       * @relates active_external
       *
       * @author Frederic Py <fpy@mbari.org>
       */
      struct is_active_external {
	/** @brief check if active
	 *
	 * @param r An external relation
	 *
	 * @retval true if @p e is active
	 * @retval false otherwise
	 */
	bool operator()(TREX::transaction::Relation const &r) const {
	  return r.active();
	}
      }; // struct TREX::transaction::details::is_active_external
      
      /** @brief Iterator though active external relations 
       *
       * This class provides a way to iterate though @c external while filtering
       * out all the non active ones.
       *
       * It is provided mostly as the @ external class used to behave this way
       * allowing user to easily reproduce this deprecated behavior.
       *
       * @relates class external
       * @author Frederic Py <fpy@mbari.org>
       */
      typedef boost::filter_iterator<TREX::transaction::details::is_active_external, 
				     TREX::transaction::details::external> active_external; 

    } // TREX::transaction::details
  } // TREX::transaction
} // TREX

#endif // H_BITS_external
