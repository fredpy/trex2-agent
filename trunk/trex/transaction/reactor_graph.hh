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
#ifndef H_reactor_graph
# define H_reactor_graph

# include "TeleoReactor_fwd.hh"
# include "bits/timeline.hh"

# include <trex/utils/TimeUtils.hh>

# include <boost/graph/graph_traits.hpp>
# include <boost/graph/adjacency_iterator.hpp>
# include <boost/graph/properties.hpp>
# include <boost/graph/reverse_graph.hpp>



namespace TREX {
  namespace transaction {

    using utils::log::null;
    using utils::log::info;
    using utils::log::warn;
    using utils::log::error;

    /** @brief Conflicting reactor names
     *
     * This mthod is throwned when multiple reactors in a graph have the
     * same name.
     *
     * Indeed, while reactors are identified internally using their memory
     * address, we also did enforce that it is not possible to have
     * multiples reactors with the same name in a transaction graph. This choice
     * was made to avoid confusion on the different logs produced by TREX.
     *
     * @author Frederic Py <fpy@mbari.org>
     * @relates class graph
     * @ingroup transaction
     */
    class MultipleReactors :public GraphException {
    public:
      /** @brief Constructor
       *
       * @param[in] g A graph
       * @param[in] r A reactor
       *

       * Create a new instance indicating that multiple reactors with the same
       * name as @p r were declared in the graph @p g
       */
      explicit MultipleReactors(graph const &g, TeleoReactor const &r) throw();
      /** @brief Destructor */
      ~MultipleReactors() throw() {}
    }; // TREX::transaction::MultipleReactors



    /** @brief Transactions graph
     *
     * This class provides and maintain the transactions between reactors
     * and timelines it maintins the relations of the reactors based on the
     * timelines they declare as external and allow to traverse reactors based
     * on these dependencies.
     *
     * It is heavily based on the boost graph library leveraging the ability to
     * uses the diverse graph traversal algorithms using diverse alghorithms
     * (depth first search, ...) that are provided by this framework
     *
     * In the past reactors and their realtion were amintined directly byt the
     * Agent class. The order in which these reactors were executed was determined
     * at the beginnign by storing them in a list using an ad-hoc topological sort.
     * This approach was simple end efficient but very limitating for future
     * evolutions/tweaking fo the agent. By porting this to a graph we can now
     * have more flexibility on the way the reactors can be schedulled. The first
     * positive impact on TREX is to allow to add new reactors on the egent after
     * its initialisation while maintaining the requirements of the synchronization
     * phase. Other potential arguments are :
     * @li allow to apply heurisitics to the way this graph is traversed. which
     *     could allow the agent to identify cynamic choices in how to schedulle
     *     reactors based on their criticity at a given time
     * @li potetnially allow parallel synchronization of reactors that are not
     *     dependents. Provided of course thata such algorithm has been implemented
     *     under BGL.
     * @li take benefit of any new search algorithm that has been implemented
     *     under BGL. This library effort is to provide a generic framework for
     *     implementing graph traversal algorithms. Having it as a basis of TREX
     *     allow then to take advantage of advances made in  the field. It also
     *     allow a TREX developper to test an algorithm inside BGL before
     *     transferring it completely inside TREX.
     *
     * @author Frederic Py <fpy@mbari.org>
     * @ingroup transactions
     */
    class graph :boost::noncopyable {
    public:
      typedef CHRONO::nanoseconds duration_type;
      typedef boost::posix_time::ptime   date_type;
          
      /** @brief reactor ID type
       *
       * The type used to represent a reactor in the graph
       */
      typedef details::reactor_id reactor_id;

      /** @brief Timeline relation creation failure
       *
       * This class is used to indicate when a timeline operation -- such as
       * declaring an Internal or External timeline --  in the
       * @c graph failed. It is used to indicate the nature of the
       * error and will be reported to the reactor that made the request
       * through failed_internal and failed_external callbacks
       *
       * @relates graph
       * @sa TeleoReactor::failed_external
       * @sa TeleoReactor::failed_internal
       *
       * @author Frederic Py <fpy@mbari.org>
       */
      struct timeline_failure: public ReactorException {
      public:
        /** @brief Destructor */
        virtual ~timeline_failure() throw() {}
        /** @brief Constructor
         * @param[in] r The reactor that requested the operation
         * @param[in] msg The associated error message
         */
        timeline_failure(graph::reactor_id r,
                         std::string const &msg) throw()
        :ReactorException(*r, msg) {}
      }; // TREX::transaction::graph::timeline_failure

      /** @brief reactor relation type
       *
       * The type used to represent the dependencies between reactors in
       * the graph.
       */
      typedef Relation                              relation_type;
      /** @brief Reactor factory
       *
       * A factory used ot produce new reactor in a graph using XML
       * input.
       */
      typedef TREX::utils::XmlFactory<TeleoReactor, boost::shared_ptr<TeleoReactor>,
				      graph *> xml_factory;

      /** @brief Reverse of a reactor graph
       * 
       * This type is the one used in order to refer to this graph with 
       * reversed relations
       */
      typedef boost::reverse_graph<graph, graph const &> reverse_graph;

      typedef boost::directed_tag            directed_category;
      typedef boost::allow_parallel_edge_tag edge_parallel_category;

      /** @brief Graph traversal category
       *
       * This pseudo-type is used by the boost graph library in order to
       * know the nature of a reactor graph and how it can be manipulated.
       *
       * So far we specified that a transaction graph repect the following
       * interfaces/concepts as specified in BGL:
       * @li Incidence Graph
       *     (http://www.boost.org/doc/libs/1_43_0/libs/graph/doc/IncidenceGraph.html)
       * @li Vertex List Graph
       *     (http://www.boost.org/doc/libs/1_43_0/libs/graph/doc/VertexListGraph.html)
       * @li Adjacency List Graph
       *     (http://www.boost.org/doc/libs/1_43_0/libs/graph/doc/AdjacencyGraph.html)
       *
       * The number of concepts supported may grow in the future as the implementation
       * of the transaction graph between reactors is improved.
       *
       * @author Frederic Py <fpy@mbari.org>
       * @relates graph
       */
      struct traversal_category:
	public virtual boost::bidirectional_graph_tag,
	public virtual boost::adjacency_graph_tag,
	public virtual boost::vertex_list_graph_tag,
	public virtual boost::edge_list_graph_tag {};
      
      /** @brief timelines and relations listener
       *
       * This abstarct class allows user to implement a listener to all the 
       * timeline connection events within a reactor graph. The operations are:
       * @li creation of a new internal timeline
       * @li undeclaration of an internal timeline
       * @li decalration of connection through an external timeline
       * @li undeclaration of connection througn an external timeline
       *
       * All of these operation trelates to the alteration of the agent graph 
       * structure and give user the ability to tack when the reactor is altering 
       * itself.
       *
       * @author Frederic Py <fpy@mbari.org>
       * @ingroup transaction
       * @relates graph
       */
      class timelines_listener {
      public:
        typedef timelines_listener const *id_type;
        typedef timelines_listener base_type;

        virtual ~timelines_listener();

        static id_type get_id(base_type const &me) {
          return &me;
        }

      protected:
        timelines_listener(graph &g);
        timelines_listener(graph::xml_factory::argument_type const &arg);
        void initialize();

        virtual void declared(details::timeline const &timeline) {}
        virtual void undeclared(details::timeline const &timeline) {}
        
        virtual void connected(Relation const &r) {}
        virtual void disconnected(Relation const &r) {}

      private:
        graph &m_graph;

        friend class TeleoReactor;
      }; // TREX::transaction::graph::timeline_listener


      /** @brief reactors iterator
       *
       * The class used to iterate through the reactors managed by this graph
       */
      class reactor_iterator
	:public boost::iterator_facade< reactor_iterator,
					reactor_id,
					boost::forward_traversal_tag,
					reactor_id > {
      public:
	/** @brief Constructor
	 *
	 * Create a new iterator referring to no reactor
	 */
	reactor_iterator() {}
	/** @brief Copy constructor
	 *
	 * @param[in] other Another itinstance
	 *
	 * Create a copy of @p other
	 */
	reactor_iterator(reactor_iterator const &other)
	  :m_iter(other.m_iter) {}
	/** @brief Conversion constructor
	 *
	 * @param[in] i An iterator to a reactor_set
	 *
	 * Create a new instance referring to the same element referred by @p i
	 */
	explicit reactor_iterator(details::reactor_set::const_iterator const &i)
	  :m_iter(i) {}
	/** @brief Destructor */
	~reactor_iterator() {}


      private:
	friend class boost::iterator_core_access;

	/** @brief advance iterator
	 *
	 * Advance the iterator to the next element
	 *
	 * @pre The iterator is valid (ie it refers to an existing element)
	 */
	void increment() {
	  ++m_iter;
	}
	/** @brief Equality test
	 *
	 * @param[in] other Another instance
	 *
	 * @retval true if this instance refers to the same element as @p other
	 * @retval false otherwise
	 */
	bool equal(reactor_iterator const &other) const {
	  return m_iter==other.m_iter;
	}
	/** @brief Dereference utility
	 *
	 * @pre The iterator is valid
	 * @return A reactor_id to the reactor referred by this iterator
	 */
	reactor_id dereference() const {
	  return m_iter->get();
	}

	details::reactor_set::const_iterator m_iter;
      }; // TREX::transaction::graph::reactor_iterator

      typedef details::reactor_set::size_type       size_type;
      typedef details::reactor_set::difference_type difference_type;

      typedef boost::edge_name_t edge_property_type;
      typedef boost::vertex_name_t vertex_property_type;

      /** @brief Graph name
       *
       * @return The name of the graph
       */
      inline TREX::utils::Symbol const &getName() const {
	return m_name;
      }
      /** @brief Constructor
       *
       * @param[in] name A symbolic name
       * @param[in] init An initial tick value
       *
       * Create a new graph named @p name with an initial tick @p init
       *
       * @post the graph is empty
       */
      explicit graph(TREX::utils::Symbol const &name, TICK init =0, bool verbose=false);
      /** @brief Constructor
       *
       * @param[in] name A symbolic name
       * @param[in] conf A XML structure iterator
       * @param[in] init An initial tick value
       *
       * Create a new graph named @p name with an initial tick @p init. It will
       * also create all the reactors as defined in the XML iterator @p conf.
       *
       * @throw ReactorException An error ocured while trying to create
       *        the reactors
       *
       * @sa add_reactors(ext_iterator)
       */
      graph(TREX::utils::Symbol const &name,
	    boost::property_tree::ptree &conf,
	    TICK init =0, bool verbose=false);
      /** @brief Destructor
       *
       * Destroy the graph along with all the reactors attached to it.
       *
       * @sa clear()
       */
      virtual ~graph();
      /** @brief Clear the graph
       *
       * Disconnect all the reactors from this graph
       *
       * @post The graph is empty
       */
      void clear();
      /** @brief Check if empty
       *
       * Checks if the graph is empty. Meanining that it has no reactor
       * associated to it.
       *
       * @retval true if the graph is empty
       * @retval false otherwise
       */
      bool empty() const {
	return m_reactors.empty();
      }

      /** @brief Create new reactors for this graph
       * @tparam Iter An iterator type
       * @param[in] from Initial iterator
       * @param[in] to end iterator
       *
       * @pre The value_type of @p Iter is boost::property_tree::ptree::value_type
       *
       * Add all the reactor that can be parsed from thre properties referred by
       * [@p from, @p to) to this graph
       */
      template<class Iter>
      size_t add_reactors(Iter from, Iter to);

      /** @brief Create new reactors for this graph
       *
       * @param[in] conf A xml configuration tree
       *
       * Create all the reactors defined in @p conf and attach them to this graph
       */
      size_t add_reactors(boost::property_tree::ptree &conf) {
	return add_reactors(conf.begin(), conf.end());
      }
      /** @brief Create new reactor for this graph
       *
       * @param[in] definition A xml configuration tree node
       *
       * Create the reactor defined by @p description and attch it to this graph
       */
      reactor_id add_reactor(boost::property_tree::ptree::value_type &description);
      static reactor_id null_reactor() {
	return reactor_id();
      }
      
      /** @brief Check if part of the graph 
       * @param[in] r A reactor
       * 
       * Checks if @p r is currently part of this graph or not. 
       * A reactor is not part of the graph if it has been isolated or killed
       *
       * @retval true if the reactor is still part of this graph
       * @retvals false otherwise
       *
       * @sa kill_reactor(reactor_id)
       * @sa isolate(reactor_id)
       */
      bool is_member(reactor_id r) const;

      /** @brief Kill a reactor
       *
       * @pram [in] r A reactor
       *
       * Disconnect and remove @p r from this graph
       *
       * @retval true the reactor was successfully removed
       * @retval false @p r did not belong to this graph
       *
       * @post @p r does not belong to this graph
       *
       * @sa isolate()
       */
      bool kill_reactor(reactor_id r);
      /** @brief Isolate a reactor
       *
       * @param[in] r A reactor
       *
       * Disconnect the ractor @p r in this graph. Disconnecting a reactor
       * correspond to disable all timeline declarations made by this reactor
       * inside this graph.
       *
       * @note After this operation the graph still belongs to the graph, it is
       *       just not connected to any other reactor anymore.
       * @retval @p r or null_reactor() if @p r did not belong to this graph
       *
       * @post if @p r did belong to this graph it is added to the set of reactor
       *       to be killed during the next cleanup operation
       *
       * @sa cleanup()
       */
      reactor_id isolate(reactor_id r) const;
      bool is_isolated(reactor_id r) const;
      /** @brief Kill all isolated reactors
       *
       * Kill all the reactors which have been explicitedly isolated in this
       * graph
       *
       * @return the numeof reactor that have been killed
       *
       * @sa isolate(reactor_id) const
       * @sa kill_reactor(reactor_id)
       */
      size_t cleanup();
      
      TICK as_date(std::string const &str) const;
      TICK as_duration(std::string const &str, bool up=false) const;

      /** @brief get current tick
       *
       * This method provides the current tick being executed in the graph.
       *
       * @return The tick value
       *
       * @note This method is not usefull at the graph level and is more used by
       *       the Agent. It may be refactored in roder to keep the graph class
       *       as simple as possible.
       */
      TICK getCurrentTick() const {
	return m_currentTick;
      }
      virtual TICK finalTick() const {
        return std::numeric_limits<TICK>::max();
      }
      
      /** @brief get tick duration
       *
       * This method provides the tick duration in real-time in order to
       * allow to convert tick varelated values into a real-time value
       * @return The tick duration
       *
       * @note This method is not usefull at the graph level and is more used by
       *       the Agent. It may be refactored in roder to keep the graph class
       *       as simple as possible.
       * @sa tickToTime(TICK) const
       * @sa timeToTick(time_t, suseconds_t) const
       */
      virtual duration_type tickDuration() const {
	return CHRONO::seconds(1);
      }
      /** @brief convert real-time into a TICK
       *
       * @param secs Number of seconds
       * @param usecs Micro seconds
       *
       * This method provides an utility to convert a real-time date into a
       * TICK value
       *
       * @return the TICK equalent to the time sepresented by @p secs seconds and
       *        @p usecs microseconds
       *
       * @note This method is not usefull at the graph level and is more used by
       *       the Agent. It may be refactored in roder to keep the graph class
       *       as simple as possible.
       * @sa tickDuration() const
       * @sa tickToTime(TICK) const
       */
      virtual TICK timeToTick(date_type const &date) const {
        typedef utils::chrono_posix_convert<duration_type> convert;
	return convert::to_chrono(date-boost::posix_time::from_time_t(0)).count()/tickDuration().count();
      }
      /** @brief convert a TICK into its real-time equivalent
       *
       * @param cur A TICK date
       *
       * This method provides an utility to convert a TICK value into its real-time
       * equivalent
       *
       * @return a float representing the date (usually in seconds)
       *        corresponding to the tTICK @p cur
       *
       * @note This method is not usefull at the graph level and is more used by
       *       the Agent. It may be refactored in roder to keep the graph class
       *       as simple as possible.
       * @sa tickDuration() const
       * @sa timeToTick(time_t, suseconds_t) const
       */
      virtual date_type tickToTime(TICK cur) const {
        typedef utils::chrono_posix_convert<duration_type> convert;
        return boost::posix_time::from_time_t(0)+convert::to_posix(tickDuration()*cur);
      }
  
      virtual std::string date_str(TICK cur) const;
      virtual std::string duration_str(TICK dur) const;
      
      /** @brief Number of reactors
       *
       * @return the number of reactors in this graph
       */
      size_type count_reactors() const {
	return m_reactors.size();
      }
      /** @brief Reactor index
       *
       * @param[in] r A reactor
       *
       * @return An arbitrary index value for @p r or count_reactors() if this
       *       reactor doea not belong to this graph.
       * @note For reactor tat belong to the graph the returned value is between
       *       0 and count_reactors()-1
       * @warning The index of a reactor can change whenver the graph is changed
       *          (by either adding new reactors or removing ones)
       *
       * @sa count_reactors() const
       */
      long index(reactor_id r) const;


      /** @brief Beginning reactor iterator
       *
       * @return a reactor_iterator pointing to the beginning of the reactors
       *        set maintained by this graph
       * @sa reactor_end() const
       */
      reactor_iterator reactor_begin() const {
	return reactor_iterator(m_reactors.begin());
      }
      /** @brief End reactor iterator
       *
       * @return a reactor_iterator pointing to the end of the reactors
       *        set maintained by this graph
       * @sa reactor_begin() const
       */
      reactor_iterator reactor_end() const {
	return reactor_iterator(m_reactors.end());
      }
      /** @brief find a reactor
       *
       * @param[in] name A reactor symbolic name
       *
       * @return An iterator referring to the reactor with the name @p name
       *         or reactor_end() if no such reactor exists
       *
       * @note  As the graph does not allow to have multiple reactors with the
       *        same name the solution is always unique.
       *
       * @sa reactor_end() const
       * @sa TeleoReactor::getName() const
       */
      reactor_iterator find_reactor(TREX::utils::Symbol const &name) const {
	return reactor_iterator(m_reactors.find(name));
      }

      typedef details::relation_iter edge_iterator;

      size_type count_relations() const;
      edge_iterator relation_begin() const {
	return details::relation_iter(m_timelines.begin(), m_timelines.end());
      }
      edge_iterator relation_end() const {
	return details::relation_iter(m_timelines.end(), m_timelines.end());
      }

      graph &me() {
	return *this;
      }
      graph const &me() const {
	return *this;
      }

      /** @brief Check for verbosity level
       *
       * Checks if this graph is set as verbose or not.
       * This value will be the default verbosity leve lof reactors handled by 
       * this graph
       *
       * @sa set_verbose(bool)
       * @sa TeleoReactor::reset_verbose()
       */
      bool is_verbose() const {
        return m_verbose;
      }
      /** @brief Set default verbosity level
       * @param[in] flag verbosity flag
       * @sa is_verbose() const
       */
      void set_verbose(bool flag=true) {
        m_verbose = flag;
      }
      
      goal_id parse_goal(boost::property_tree::ptree::value_type goal) const;
      boost::property_tree::ptree export_goal(goal_id const &g) const;
      
      
      boost::asio::strand &strand() {
        return *m_strand;
      }
      
    protected:
      reactor_id add_reactor(reactor_id r);

      graph();

      bool hasTick() const {
	return m_tick_valid;
      }

      void updateTick(TICK value, bool started=true) {
	m_tick_valid = started;
	m_currentTick = value;
      }
      void set_name(TREX::utils::Symbol const &name) {
	m_name = name;
      }

      utils::log::stream syslog(utils::log::id_type const &context, 
                                utils::log::id_type const &kind) const;
      utils::log::stream syslog(utils::log::id_type const &kind=null) const {
	return syslog(null, kind);
      }
      
      TREX::utils::LogManager &manager() const {
	return *m_log;
      }

      bool has_timeline(TREX::utils::Symbol const &tl) const {
	return m_timelines.find(tl)!=m_timelines.end();
      }

      typedef details::timeline_set::const_iterator timeline_iterator;

      timeline_iterator timeline_begin() const {
	return m_timelines.begin();
      }
      timeline_iterator timeline_end() const {
	return m_timelines.end();
      }

      /** @brief Check for internal timeline creation
       *
       * @param[in] r A reactor
       * @param[in] tl A timeline
       *
       * This method is used by the graph to check if the reactor @p r can
       * declare the timeline @p tl as internal.
       *
       * It can be used by derived classes to add extra control on the creation
       * of internal timelines. For example this is where the Agent could check
       * if such internal timeline creation won't generate a dependency cycle.
       *
       * @throw timeline_failure indicates that @p tl cannot be declared as
       * internal by @p r
       *
       * @sa TeleoReactor::failed_internal(Symbol const &, timeline_failure const &)
       * @sa external_check(reactor_id r, details::timeline const &)
       */
      virtual void internal_check(reactor_id r, details::timeline const &tl) {}
      /** @brief Check for external timeline creation
       *
       * @param[in] r A reactor
       * @param[in] tl A timeline
       *
       * This method is used by the graph to check if the reactor @p r can
       * declare the timeline @p tl as external.
       *
       * It can be used by derived classes to add extra control on the creation
       * of internal timelines. For example this is where the Agent could check
       * if such external timeline creation won't generate a dependency cycle.
       *
       * @throw timeline_failure indicates that @p tl cannot be declared as
       * external by @p r
       *
       * @sa TeleoReactor::failed_external(Symbol const &, timeline_failure const &)
       * @sa internal_check(reactor_id r, details::timeline const &)
       */
      virtual void external_check(reactor_id r, details::timeline const &tl) {}

    private:
      
      bool assign(reactor_id r, TREX::utils::Symbol const &timeline,
                  details::transaction_flags const &flags);
      bool subscribe(reactor_id r, TREX::utils::Symbol const &timeline,
		     details::transaction_flags const &flags);
      
      details::timeline_set::iterator get_timeline(TREX::utils::Symbol const &tl);

      utils::Symbol            m_name;
      details::reactor_set     m_reactors;
      details::timeline_set    m_timelines;
      bool                     m_tick_valid;
      TICK                     m_currentTick;
      
      UNIQ_PTR<boost::asio::strand> m_strand;

      typedef TREX::utils::list_set< TREX::utils::pointer_id_traits<graph::timelines_listener> > listen_set;
      listen_set m_listeners;

      bool m_verbose;
      TREX::utils::SingletonUse<TREX::utils::LogManager> m_log;
      TREX::utils::SingletonUse<xml_factory>             m_factory;

      mutable details::reactor_set m_quarantined;

      friend class TeleoReactor;
      friend class timelines_listener;
    };

    std::string date_export(graph const &g, IntegerDomain::bound const &val);
    std::string duration_export(graph const &g, IntegerDomain::bound const &val);
  }
}

#endif
