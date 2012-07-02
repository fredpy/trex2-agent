#ifndef H_reactor_graph
# define H_reactor_graph

# include "TeleoReactor_fwd.hh"
# include "bits/timeline.hh"

# include <boost/graph/graph_traits.hpp>
# include <boost/graph/adjacency_iterator.hpp>
# include <boost/graph/properties.hpp>

namespace TREX {
  namespace transaction {

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
      /** @brief reactor ID type
       *
       * The type used to represent a reactor in the graph
       */
      typedef details::reactor_id reactor_id;
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

      typedef boost::directed_tag            directed_category;
      typedef boost::allow_parallel_edge_tag edge_parallel_category;

      /** @brief Graph traversal category
       *
       * This pseudo-type is used by the boost graph libraray in order to
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
       * The number of concepts supported may grow in the future as the implementation of
       * the transaction graph between reactors is improved.
       *
       * @author Frederic Py <fpy@mbari.org>
       * @relates graph
       */
      struct traversal_category:
	public virtual boost::incidence_graph_tag,
	public virtual boost::adjacency_graph_tag,
	public virtual boost::vertex_list_graph_tag,
	public virtual boost::edge_list_graph_tag {};

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
	reactor_iterator() {}
	reactor_iterator(reactor_iterator const &other)
	  :m_iter(other.m_iter) {}
	explicit reactor_iterator(details::reactor_set::const_iterator const &i)
	  :m_iter(i) {}
	~reactor_iterator() {}
	
      private:
	friend class boost::iterator_core_access;

	void increment() {
	  ++m_iter;
	}
	bool equal(reactor_iterator const &other) const {
	  return m_iter==other.m_iter;
	}
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
      explicit graph(TREX::utils::Symbol const &name, TICK init =0);
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
	    TREX::utils::ext_iterator const &conf, 
	    TICK init =0);
      /** @brief Destructor
       *
       * Destroy the graph along with all the reactors attached to it.
       *
       * @sa clear()
       */
      virtual ~graph();

      void clear();
      
      bool empty() const {
	return m_reactors.empty();
      }

      size_t add_reactors(TREX::utils::ext_iterator iter);
      reactor_id add_reactor(rapidxml::xml_node<> &description); 
      static reactor_id null_reactor() {
	return reactor_id();
      }

      bool kill_reactor(reactor_id r);
      reactor_id isolate(reactor_id r) const;

      TICK getCurrentTick() const {
	return m_currentTick;
      }
      virtual double tickDuration() const {
	return 1.0;
      }

      size_type count_reactors() const {
	return m_reactors.size();
      }
      long index(reactor_id r) const;
   
      reactor_iterator reactor_begin() const {
	return reactor_iterator(m_reactors.begin());
      }
      reactor_iterator reactor_end() const {
	return reactor_iterator(m_reactors.end());
      }  
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
      
    protected:
      reactor_id add_reactor(reactor_id r);

      graph() {}

      void updateTick(TICK value) {
	m_currentTick = value;
      }
      void set_name(TREX::utils::Symbol const &name) {
	m_name = name;
      }

      TREX::utils::internals::LogEntry syslog(std::string const &context=std::string()) const;
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

    private:
      bool assign(reactor_id r, TREX::utils::Symbol const &timeline);
      bool subscribe(reactor_id r, TREX::utils::Symbol const &timeline,
		     bool control);

      details::timeline_set::iterator get_timeline(TREX::utils::Symbol const &tl);

      utils::Symbol            m_name;
      details::reactor_set     m_reactors;
      details::timeline_set    m_timelines;
      TICK                     m_currentTick;

      TREX::utils::SingletonUse<TREX::utils::LogManager> m_log;
      TREX::utils::SingletonUse<xml_factory>             m_factory;

      friend class TeleoReactor;
    };

  }
}

#endif 