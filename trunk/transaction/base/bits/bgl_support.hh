/** @file transaction/base/bits/bgl_support.hh"
 * 
 * @brief Boost Graph Library support code 
 *
 * This file provides all the classes and definitions require by the boost graph
 * library in order to manipulate a TREX::transaction::graph
 *
 * @author Frederic Py <fpy@mbari.org>
 * @ingroup transaction
 */
#ifndef H_bgl_support
# define H_bgl_support

namespace boost {

  /** @brief Transaction graph traits
   *
   * Gives several informations on the nature of a transaction graph that
   * helps boost graph library (BGL) know how to extract information about nodes
   * and edges of this graph.
   *
   * A transaction graph is a directed graph with the vertex corresponding to
   * the reactors and the edges being Relation from @e External timelines to
   * their @e Internal counterpart.
   *
   * For furher details about this traits please refer to boost documentation on
   * the BGL :
   * @li http://www.boost.org/doc/libs/1_46_1/libs/graph/doc/table_of_contents.html
   *
   * @author Frederic Py <fpy@mbari.org>
   * @ingroup transaction
   * @relates class TREX::transaction::graph
   */
  template<>
  struct graph_traits<TREX::transaction::graph> {
  private:
    // Helpers :
    typedef TREX::transaction::graph graph_type;

  public:
    // Graph concept
    typedef graph_type::reactor_id        vertex_descriptor;
    typedef graph_type::relation_type     edge_descriptor;

    typedef graph_type::directed_category      directed_category;
    typedef graph_type::edge_parallel_category edge_parallel_category;
    typedef graph_type::traversal_category     traversal_category;

    static inline vertex_descriptor null_vertex() {
      return graph_type::null_reactor();
    }

    // Incidence Graph Concept
    typedef TREX::transaction::TeleoReactor::external_iterator  out_edge_iterator;
    typedef TREX::transaction::TeleoReactor::size_type          degree_size_type;

    // Adjacency Graph Concetp
    typedef boost::adjacency_iterator<graph_type, vertex_descriptor,
				      out_edge_iterator, 
				      graph_type::difference_type> adjacency_iterator;
    // vertex List graph Concept
    typedef graph_type::reactor_iterator     vertex_iterator;
    typedef graph_type::size_type            vertices_size_type;

    // edge List graph Concept
    typedef graph_type::edge_iterator     edge_iterator;
    typedef graph_type::size_type         edges_size_type;
  }; // boost::graph_traits<TREX::transaction::graph>

  /*
   * Incidence Graph Concept
   */
  /** @brief Out edges of a reactor
   * 
   * @param[in] r A reactor
   * @param[in] g A graph
   *
   * Gives all the relations reflecting the external timelines of the reactor @p r
   *
   * @ingroup transaction
   * @relates TREX::transaction::graph
   *
   * @sa TREX::transaction::TeleoReactor::ext_begin()
   * @sa TREX::transaction::TeleoReactor::ext_end()
   * @sa out_degree(TREX::transaction::graph::reactor_id, TREX::transaction::graph const &)
   * @sa adjacent_vertices(TREX::transaction::graph::reactor_id, TREX::transaction::graph const &)
   */
  inline std::pair< TREX::transaction::TeleoReactor::external_iterator, 
		    TREX::transaction::TeleoReactor::external_iterator >
  out_edges(TREX::transaction::graph::reactor_id r,
	    TREX::transaction::graph const &g) {
    return std::make_pair(r->ext_begin(), r->ext_end());
  }
  
  /** @brief source of a relation
   *
   * @param[in] rel A relation
   * @param[in] g   A graph
   *
   * Gives the reactors that created this relation by declaring the corresponding
   * timeline as @e External
   *
   * @return The reactor which is the client of this relation
   *
   * @sa TREX::transaction::Relation::client() const
   * @sa target(TREX::transaction::graph::relation_type const &, TREX::transaction::graph cons &)
   *
   * @ingroup transaction
   * @relates TREX::transaction::graph
   */
  inline TREX::transaction::graph::reactor_id
  source(TREX::transaction::graph::relation_type const &rel,
	 TREX::transaction::graph const &g) {
    return &(rel.client());
  }

  /** @brief target of a relation
   *
   * @param[in] rel A relation
   * @param[in] g   A graph
   *
   * Gives the reactors that this relation connect to. This reactor is the one that declared
   * the corresponding timeline as @e Internal
   *
   * @return The reactor which is the client of this relation or
   *         TREX::transaction::graph::null_reactor() if the relation is not active
   *
   * @sa TREX::transaction::Relation::active() const
   * @sa TREX::transaction::Relation::server() const
   * @sa TREX::transaction::graph::null_reactor()
   * @sa source(TREX::transaction::graph::relation_type const &, TREX::transaction::graph cons &)
   *
   * @ingroup transaction
   * @relates TREX::transaction::graph
   */
  inline TREX::transaction::graph::reactor_id
  target(TREX::transaction::graph::relation_type const &rel,
	 TREX::transaction::graph const &g) {
    if( rel.active() )
      return &(rel.server());
    return TREX::transaction::graph::null_reactor();
  }

  /** @brief Number of external relations for a reactor
   *
   * @param[in] r A reactor
   * @param[in] g A graph
   *
   * Indicates the number of @p External timelines the reactor @p r
   * have declared
   *
   * @return The number of @p External timelines of @p r
   *
   * @sa TREX::transaction::TeleoReactor::count_externals() const
   * @sa out_edges(TREX::transaction::graph::relation_id,TREX::transaction::graph const &)
   *
   * @ingroup transaction
   * @relates TREX::transaction::graph
   */
  inline TREX::transaction::TeleoReactor::size_type 
  out_degree(TREX::transaction::graph::reactor_id r,
	     TREX::transaction::graph const &g) {
    return TREX::transaction::graph::null_reactor()==r?0:r->count_externals();
  }
  
  /*
   * Adjacency graph concept 
   */

  /** @brief Reactor adjacency vertices of a reactor
   * 
   * @param[in] r A reactor
   * @param[in] g A graph
   *
   * Gives all the relations reflecting the external timelines of the reactor @p r. This
   * method is very similar to out_edges except that the relation are represented
   * differently.
   *
   * @ingroup transaction
   * @relates TREX::transaction::graph
   * @sa out_degree(TREX::transaction::graph::reactor_id, TREX::transaction::graph const &)
   * @sa out_edges(TREX::transaction::graph::reactor_id, TREX::transaction::graph const &)
   */
  inline std::pair< graph_traits<TREX::transaction::graph>::adjacency_iterator,
		    graph_traits<TREX::transaction::graph>::adjacency_iterator >
  adjacent_vertices(TREX::transaction::graph::reactor_id r,
		    TREX::transaction::graph const &g) {
    std::pair< TREX::transaction::TeleoReactor::external_iterator, 
	       TREX::transaction::TeleoReactor::external_iterator >
      tmp = out_edges(r, g);
    return std::make_pair
      (graph_traits<TREX::transaction::graph>::adjacency_iterator(tmp.first, &g),
       graph_traits<TREX::transaction::graph>::adjacency_iterator(tmp.second, &g));
  }

  /*
   * Vertex List Graph Concept
   */
  /** @brief Graph reactors
   *
   * @param[in] g A graph
   *
   * Gives access to all the reactors maintined by the graph @p g
   *
   * @relates TREX::transaction::graph
   * @ingroup transaction
   *
   * @sa TREX::transaction::graph::reactor_begin() const
   * @sa TREX::transaction::graph::reactor_end() const
   * @sa num_vertices(TREX::transaction::graph const &)
   */
  inline std::pair<TREX::transaction::graph::reactor_iterator,
		   TREX::transaction::graph::reactor_iterator>
  vertices(TREX::transaction::graph const &g) {
    return std::make_pair(g.reactor_begin(), g.reactor_end());
  }

  /** @brief Number of reactors
   *
   * @param[in] g A graph
   *
   * Indicates the number of reactors mainteind by the graph @p g
   *
   * @return the number of reactors in @p g
   * 
   * @relates TREX::transaction::graph
   * @ingroup transaction
   *
   * @sa TREX::transaction::graph::count_reactors() const
   * @sa vertices(TREX::transaction::graph const &)
   */
  inline TREX::transaction::graph::size_type
  num_vertices(TREX::transaction::graph const &g) {
    return g.count_reactors();
  }

  /*
   * Edge List Graph Concept
   */
  inline std::pair< TREX::transaction::graph::edge_iterator,
		    TREX::transaction::graph::edge_iterator >
  edges(TREX::transaction::graph const &g) {
    return std::make_pair(g.relation_begin(), g.relation_end());
  }

  inline TREX::transaction::graph::size_type
  num_edges(TREX::transaction::graph const &g) {
    return g.count_relations();
  }

  
} // boost

namespace TREX {
  namespace transaction {
    namespace details {
      
      /** @brief relation name mapping
       *
       * This class helps boost graph library to map a relation
       * to the name of the timeline it represents.
       *
       * This is particularly helpfull to serialize a transaction graph
       * in different file formats such as graphviz.
       *
       * @ingroup transaction
       * @relates TREX::transaction::graph
       *
       * @author Frederic Py <fpy@mbari.org>
       */
      struct graph_edge_name_map 
	:public boost::put_get_helper<TREX::utils::Symbol, 
				      graph_edge_name_map> {

	typedef boost::readable_property_map_tag category;
	typedef TREX::utils::Symbol              value_type;
	typedef TREX::utils::Symbol const &      reference;
	typedef graph::relation_type             key_type;

	/** @brief Relation name accessor
	 *
	 * @param[in] rel A relation
	 *
	 * @return the name of the timelin associated to @p rel
	 *
	 * @sa TREX::transaction::Relation::name() const
	 */
	reference operator[](key_type const &rel) const {
	  return rel.name();
	}
      }; // TREX::transaction::details::graph_edge_name_map

      /** @brief reactor name mapping
       *
       * This class helps boost graph library to map a reactor
       * to its name
       *
       * This is particularly helpfull to serialize a transaction graph
       * in different file formats such as graphviz.
       *
       * @ingroup transaction
       * @relates TREX::transaction::graph
       *
       * @author Frederic Py <fpy@mbari.org>
       */
      struct graph_vertex_name_map 
	:public boost::put_get_helper<TREX::utils::Symbol, 
				      graph_vertex_name_map> {

	typedef boost::readable_property_map_tag category;
	typedef TREX::utils::Symbol              value_type;
	typedef TREX::utils::Symbol const &      reference;
	typedef graph::reactor_id                key_type;

	/** @brief Reactor name accessor
	 *
	 * @param[in] r A reactor
	 *
	 * @return the name of the reactor @p r or @c "" if the reactor
	 *         is null
	 *
	 * @sa TREX::transaction::graph::null_reactor()
	 * @sa TREX::transaction::TeleoReactor::getName() const
	 */
	reference operator[](key_type r) const {
	  static TREX::utils::Symbol const null_name;

	  if( graph::null_reactor()==r )
	    return null_name;
	  else
	    return r->getName();
	}
      }; // TREX::transaction::details::graph_vertex_name_map
      
      /** @brief reactor ID mapping
       *
       * This class helps boost graph library to map a reactor
       * to a unique id. 
       *
       * This is particularly helpfull to serialize a transaction graph
       * in different file formats such as graphviz.
       *
       * @ingroup transaction
       * @relates TREX::transaction::graph
       *
       * @author Frederic Py <fpy@mbari.org>
       */
      struct graph_vertex_id_map
	:public boost::put_get_helper<long, 
				      graph_vertex_id_map> {
      public:
	typedef boost::readable_property_map_tag category;
	typedef long                             value_type;
	typedef long                             reference;
	typedef graph::reactor_id                key_type;

	/** @brief Constructor
	 *
	 * @param[in] g A graph
	 *
	 * Create a new instance related to the graph @p g
	 */
	graph_vertex_id_map(graph const &g)
	  :m_g(&g) {}
	
	/** @brief Reactor ID accessor
	 *
	 * @param[in] r A reactor
	 *
	 * @return the ID of the reactor @p r 
	 *
	 * @sa TREX::transaction::graph::index(TREX::transaction::graph::reactor_id) const
	 */
	value_type operator[](key_type r) const {
	  return m_g->index(r);
	}
      private:
	graph const *m_g;

      }; // TREX::transaction::details::graph_vertex_id_map
      
    } // TREX::transaction::details    
    
    /** @brief graphviz display helper
     * 
     * This class is an helper for bgl graphviz export utility it allows
     * to add labels to both the nodes and the edges of the graph
     * reflecting the reactors and timelines names.
     *
     * @author Frederic Py <fpy@mbari.org>
     * @ingroup transactions 
     */
    class graph_names_writer {
    public:
      /** @brief Defualt constructor */
      graph_names_writer() {}
      
      /** @brief label for a reactor
       *
       * @param[in,out] out An output stream
       * @param[in] A reactor
       *
       * This method will write intoo @p out the label graphviz attribute of the node
       * with the name of the reactor @p r
       */
      void operator()(std::ostream &out, graph::reactor_id r) const {
	if( graph::null_reactor()!=r )
	  label(out, r->getName());	
      }
      /** @brief label for a timeline relation
       *
       * @param[in,out] out An output stream
       * @param[in] A timeline relation
       *
       * This method will write intoo @p out the label graphviz attribute of the node
       * with the name of the timeline assicaited to the relation @p r
       */
      void operator()(std::ostream &out, graph::relation_type const &rel) const {
	if( rel->active() ) {
	  out<<"[label=\""<<rel->name();
	  if( rel->accept_goals() )
	    out<<'['<<rel->latency()<<':'<<rel->look_ahead()<<']';
	  out<<"\"]";
	} else 
	  // display inactive relation in red
	  out<<"[label=\""<<rel->name()<<"\" color=\"red\"]";
      }

    private:
      /** @brief graphviz label writing helper
       *
       * @param[in,out] out An outtput stream
       * @param[in] name    A symbol
       *
       * This method will wirte into @p out a graphviz formtatted optional information
       * that indicates that labe should  be the value of @p name
       */
      void label(std::ostream &out, TREX::utils::Symbol const &name) const {
	out<<"[label=\""<<name<<"\"]";
      }	
    };

  } // TREX::transaction
} // TREX

namespace boost {
  /*
   * Properties
   */

  /** @brief Get relation name map
   *
   * Gets the relation name map associated to a graph
   *
   * @ingroup transaction
   * @relates TREX::transaction::graph
   */
  inline TREX::transaction::details::graph_edge_name_map
  get(edge_name_t, TREX::transaction::graph const &) {
    return TREX::transaction::details::graph_edge_name_map();
  }
  
  /** @brief Get relation name
   *
   * @param[in] m A relation name map
   * @param[in] rel A relation
   *
   *  Get the name of the relation @p rel as provided by the map @p m
   *
   * @ingroup transaction
   * @relates TREX::transaction::graph
   *
   * @sa get(edge_name_t, TREX::transaction::graph const &)
   * @sa TREX::transaction::details::graph_edge_name_map
   */
  inline TREX::utils::Symbol const &
  get(TREX::transaction::details::graph_edge_name_map const &m,
      TREX::transaction::graph::relation_type const &rel) {
    return m[rel];
  }

  /** @brief Graphs property map for edge name 
   *
   * This class is used by boost graph library to 
   * know the type of maps that asociates a transaction graph edges
   * to their name
   *
   * @ingroup transaction
   * @relates TREX::transaction::graph
   *
   * @author Frederic Py
   */
  template<>
  struct property_map<TREX::transaction::graph, edge_name_t> {
    typedef TREX::transaction::details::graph_edge_name_map type;
    typedef TREX::transaction::details::graph_edge_name_map const_type;
  };

  /** @brief Edge name property map 
   *
   * This class is used by boost graph library to know the type of an
   * edge name in a transaction graph
   *
   * @ingroup transaction
   * @relates TREX::transaction::graph
   *
   * @author Frederic Py
   */
  template<>
  struct property_map<TREX::transaction::graph::relation_type,
		      TREX::transaction::details::graph_edge_name_map> {
    typedef TREX::transaction::graph::relation_type       type;
    typedef TREX::transaction::graph::relation_type const const_type;
  };

  /** @brief Get reactor name map
   *
   * Gets the reactor name map associated to a graph
   *
   * @ingroup transaction
   * @relates TREX::transaction::graph
   */
  inline TREX::transaction::details::graph_vertex_name_map
  get(vertex_name_t, TREX::transaction::graph const &) {
    return TREX::transaction::details::graph_vertex_name_map();
  }
  
  /** @brief Get reactor name
   *
   * @param[in] m A reactor name map
   * @param[in] r A reactor
   *
   *  Get the name of the relctor @p r as provided by the map @p m
   *
   * @ingroup transaction
   * @relates TREX::transaction::graph
   *
   * @sa get(vertex_name_t, TREX::transaction::graph const &)
   * @sa TREX::transaction::details::graph_vertex_name_map
   */
  inline TREX::utils::Symbol const &
  get(TREX::transaction::details::graph_vertex_name_map const &m,
      TREX::transaction::graph::reactor_id r) {
    return m[r];
  }

  /** @brief Graphs property map for reactor name 
   *
   * This class is used by boost graph library to 
   * know the type of maps that asociates a transaction graph reactors
   * to their name
   *
   * @ingroup transaction
   * @relates TREX::transaction::graph
   *
   * @author Frederic Py
   */
  template<>
  struct property_map<TREX::transaction::graph, vertex_name_t> {
    typedef TREX::transaction::details::graph_vertex_name_map type;
    typedef TREX::transaction::details::graph_vertex_name_map const_type;
  };

  /** @brief Reactor name property map 
   *
   * This class is used by boost graph library to know the type of a
   * reactor name in a transaction graph
   *
   * @ingroup transaction
   * @relates TREX::transaction::graph
   *
   * @author Frederic Py
   */
  template<>
  struct property_map<TREX::transaction::graph::reactor_id,
		      TREX::transaction::details::graph_vertex_name_map> {
    typedef TREX::transaction::graph::reactor_id       type;
    typedef TREX::transaction::graph::reactor_id const const_type;
  };
  
  /** @brief Get reactor ID map
   *
   * @param[in] g A transaction graph
   *
   * Gets the reactor ID map associated to the graph @p g
   *
   * @ingroup transaction
   * @relates TREX::transaction::graph
   */
  inline TREX::transaction::details::graph_vertex_id_map
  get(vertex_index_t, TREX::transaction::graph const &g) {
    return TREX::transaction::details::graph_vertex_id_map(g);
  }
  

  /** @brief Graphs property map for reactor ID 
   *
   * This class is used by boost graph library to 
   * know the type of maps that asociates a transaction graph reactors
   * to their ID
   *
   * @ingroup transaction
   * @relates TREX::transaction::graph
   *
   * @author Frederic Py
   */
  template<>
  struct property_map<TREX::transaction::graph, vertex_index_t> {
    typedef TREX::transaction::details::graph_vertex_id_map type;
    typedef TREX::transaction::details::graph_vertex_id_map const_type;
  };

} // boost 

#endif // H_bgl_support
