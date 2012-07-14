/**  @file "Agent.cc"
 * @brief Agent implementation.
 *
 * @author Conor McGann
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

#include "Agent.hh"

#include <iostream>
#include <iterator>

// #define below is needed in bosst 1.47 under xcode 4.2
// overwise it fails to compile and get confused ???
// #define BOOST_EXCEPTION_DISABLE
#include <boost/graph/graphviz.hpp>
#include <boost/graph/topological_sort.hpp>
#include <boost/graph/depth_first_search.hpp>
#include <boost/graph/reverse_graph.hpp>

#include <boost/thread.hpp>


using namespace TREX::agent;
using namespace TREX::transaction;
using namespace TREX::utils;
namespace xml = boost::property_tree::xml_parser;

namespace TREX {
  namespace agent {
    namespace details {

      /** @brief Reactors initialization visitor
       *
       * This class is a depth first search visitor that will initialize all
       * the reactors from the least dependent to the most dependent in the
       * graph.
       *
       * The initialization is done through the @c initialize call on each
       * reactor that will in turn call handleInit callback insisde this
       * reactor
       *
       * @sa TREX::transaction::TeleoReactor::initialize(TREX::transaction::TICK)
       * @sa TREX::transaction::TeleoReactor::handleInit()
       *
       * @relates class Agent
       * @ingroup agent
       * @author Frederic Py <fpy@mbari.org>
       */
      class init_visitor :public CycleDetector {
      public:
	/** @brief Constructor
	 * @param[in] final the final tick of the mission
	 */
	explicit init_visitor(TICK final)
	  :m_final(final) {}
	/** @brief Copy constructor
	 *
	 * @param[in] other the instance to copy
	 */
	init_visitor(init_visitor const &other)
	  :CycleDetector(other), m_final(other.m_final) {}
	/** @brief Destructor */
	virtual ~init_visitor() {}

	/** @brief Finish vertex callback
	 *
	 * @param[in] r A reactor
	 * @param[in] g The graph @p r is attached to
	 *
	 * This method will be called during a depth first search when all
	 * the reactors @p r depends on have been already explored.
	 *
	 * It will call the initialize method of @p r with a final tick value as
	 * provided by the m_finalTick attribute in order to allow this reactor
	 * to complete its initialization.
	 *
	 * @sa Agent::initComplete()
	 * @sa TREX::transaction::TeleoReactor::initialize(TREX::transaction::TICK)
	 */
	void finish_vertex(graph::reactor_id r, graph const &g) {
	  if( is_valid(r,g) ) {
	    if( !r->initialize(m_final) ) {
	      g.isolate(r);
	    }
	  }
	}
      protected:
	/** @brief Final tick value
	 *
	 * Stores the final tick of the mission that will be proovided to
	 * the reactors during the depth first search execution of this visitor.
	 */
	TICK m_final;

      };

      /** @brief reactor synchronization visitor
       *
       * This class is used during a depth first in order to:
       * @li call the method @c newTick() on each of these reactors
       * @li identify a scheduling order for calling the reactors synchronization
       *     from the least dependent to the most dependent.
       *
       * The scheduling order is stored in the list this callback points to and can
       * be used after the depth first search by the Agent to execue the synchronization
       * of each reactors.
       *
       * @note We may in the future embeds the synchronization call of the reactors
       *       directly into this search. Even though it may be more efficient -- as we
       *       would avoid to have to construct this list and then iterate though it. We
       *       have yet to check and verify that such approach would be valid as it would
       *       interleave the calls of @c newTick and @c synchronize of the reactors
       *       which may -- or may not -- be problematic.
       *
       * @ingroup agent
       * @relates clas Agent
       * @author Frederic Py <fpy@mbari.org>
       */
      class sync_scheduller :public CycleDetector {
      public:
	/** @brief scheduling queue type
	 *
	 * The type of the queue which is used to store the
	 * reactors in the scheduling order for synchronization.
	 *
	 * This is just a simple list which can then be iterated
	 * from the beginning in order to go in a valid order for
	 * reactors execution
	 */
	typedef std::list<graph::reactor_id> reactor_queue;

	/** @brief Constructor
	 *
	 * @param[in,out] sync_order A reference to the list where the schduling
	 *                           order should be stored
	 *
	 * Create A new instance that will put the reactor synchronization
	 * scheduling into the list referred by @p sync_order
	 *
	 * @note As boost graph library passes visitors by copy it is necessary
	 *       here to have this list maintained as a reference to the list.
	 *       Otherwise we would loose the information of this scheduling as
	 *       soon as the search complete.
	 */
	explicit sync_scheduller(reactor_queue &sync_order)
	  :m_sync(&sync_order) {}
	/** @briief Copy constructor
	 *
	 * @param[in] other Another instance
	 *
	 * Create new instance referring to the same scheduling list as @p other
	 */
	sync_scheduller(sync_scheduller const &other)
	  :CycleDetector(other), m_sync(other.m_sync) {}
	/** @brief Destructor */
	virtual ~sync_scheduller() {}

	/** @brief First encounter of a reactor
	 *
	 * @param[in] r A reactor
	 * @param[in] g The grah @p r belngs to
	 *
	 * This method is called when the sarch encounter for the first time the
	 * reactor @p r. It calls the method @c newTick() to indicate to @p r
	 * that a new tick has started.
	 */
	void discover_vertex(graph::reactor_id r, graph const &g) {
	  // First encounter with r :
	  //   - inform him that a new tick has started
	  //        this will result on dispatching its goals if possible
	  if( is_valid(r,g) )
	    if( !r->newTick() )
              g.isolate(r);
	}

	/** @brief Finish vertex callback
	 *
	 * @param[in] r A reactor
	 * @param[in] g The graph @p r is attached to
	 *
	 * This method is called when the sarch has explored all the reactor @p r
	 * depends on. It can the insert this reactor at the end of the scheduling
	 * list ensuring that this reactor synchronization won't be executed before
	 * all the reactors it depend on have been synchronized
	 */
	void finish_vertex(graph::reactor_id r, graph const &g) {
	  // Completed all the vertex below :
	  //   - this the next candidate to be synchronized
	  if( is_valid(r,g) )
	    m_sync->push_back(r);
	}

      private:
	/** @brief Reference to scheduling list */
	reactor_queue               *m_sync;

	sync_scheduller(); // no code in purpose
      };

      /** @brief Potential Graph cycle detector
       *
       * This class is a graph visitor that looks for potential dependency cycles between 
       * 2 reactors within an Agent. It is used whenever a reactor attempts to 
       * declare an Internal or External timeline in order to check that this 
       * new connection won't generate a cyclic dependecny between this reactor 
       * and the reactor it would connect to. 
       *
       * @author Philip Cooksey
       * @ingroup agent
       * @relates Agent
       */
      class cycle_checker : public boost::default_dfs_visitor
      {
      public:
        /** @brief Conncetion type
         * THis type is used to describe the type of connection attempted
         */
        enum Cycle_Type { 
          internal, //@< internal timeline creation  
          external  //@< external timleine creation
        };
        /** @brief End of search for cycle exception
         *
         * This class is used in order to complete the cycle detection search.
         * Its boolean @c cycle attribute indicate whether the search did 
         * identify a potential cycle or not.
         *
         * @relates cycle_checker
         * @author Philip Cooksey
         * @ingroup agent
         */
        struct has_cycle {
          /** @brief Constructor
           *
           * @param[in] value Cycle detetced flag
           * @param[in] msg Addition text message
           *
           * Create a new instance indicating if a cycle was detected or not through @p value. 
           * If @p value is @c true, then @m msg should describe the potential cycle detected.
           */
          has_cycle(bool value, std::string msg="")
            :cycle(value), message(msg) {};
          bool cycle; //@< Cycle detection output
          std::string message; //@< Cycle description
        };

        /** @brief Constructor 
         * @param[in] goal The goal reactor
         * @param[in] root The reactor declaring the timeline requested as Internal
         * @param[in] type The type of the connection
         * @param[in] name Name of the timeline supporting the connection
         *
         * Create a new instance that checks if @p root can connect to goal @p goal
         * by declaring the timeline @p name as @p type.
         */
        cycle_checker(graph::reactor_id const& goal, graph::reactor_id const& root, 
                      Cycle_Type type, utils::Symbol name="" )
        :root(root), node(goal), name(name), timelineType(type) {};

        /** @brief New reactor discovered 
         *
         * @param[in] r The reactor discovered
         * @param[in] g The graph discovered
         *
         * The code to be executed when @p r is discovered on the depth 
         * first search of @p g. 
         * If @p r is a valid reactor and if the connection attempt is @c internal, this 
         * method will check that none of the external timelines of @p r are the timeline
         * we try to create.
         *
         * @throw has_cycle A cycle has been detected
         */
        void discover_vertex(graph::reactor_id r, graph const &g) {
          if(r==graph::null_reactor())
            return;

          if(timelineType==internal) {
            std::pair< TREX::transaction::TeleoReactor::external_iterator,
            TREX::transaction::TeleoReactor::external_iterator > edges = boost::out_edges(r, g);
            
            for(TREX::transaction::TeleoReactor::external_iterator temp = edges.first; temp!=edges.second; ++temp) {
              if(temp->name()==name) {
                std::stringstream msg;
                
                msg<<"Found a potential cycle going from ";
                for(std::list<utils::Symbol>::iterator it = path.begin(); it!=path.end(); it++) {
                  msg<<"("<<*it<<")->";
                }
                msg<<"("<<name<<")";
                throw has_cycle(true, msg.str());
              }
            }
          }
        }

        /** @brief Tree edge traversed
         *
         * @param[in] rel A timeline relation
         * @param[in] g The graph
         *
         * The code to be executed when depth first search travers a timeline relation that is not looping.
         * This method update he potential cycle path by adding the timeline @p rel as part of the potential 
         * cycle
         */
        void tree_edge(graph::relation_type const & rel, graph const &g) {
          //Adding reactor name to list to store the path to the potential cycle
          path.push_back(rel->name());
        }

        /** @brief Examinge an edge
         *
         * @param[in] rel A timeline relation
         * @param[in] g The graph
         *
         * The code to be executed when depth first search examine one of the timeline realtions.
         * If @p rel points to our target goal reactor and the connection type is External, this 
         * indicate that a cycle has been detected.
         *
         * @throw has_cycle potential cycle detected. 
         */
        void examine_edge(graph::relation_type const & rel, graph const &g) {
          if(timelineType==external) {
            if(boost::target(rel, g)==node) {
              std::stringstream msg;
              msg<<"Found a potential cycle going from ("<<name<<")->";
              for(std::list<utils::Symbol>::iterator it = path.begin(); it!=path.end(); it++) {
                msg<<"("<<*it<<")->";
              }
              msg<<"("<<rel->name()<<")";
              throw has_cycle(true, msg.str());
            }
          }
        }

        /** @brief End of traversal of a node
         *
         * @param[in] r A reactor
         * @param[in] g The graph traversed
         *
         * This method is called when the depth first search finished the 
         * traversal of the node @p r. If @p r is the root of the relation this 
         * means that there's no potential cycle and we can complete the search 
         * prematurely. Otherwise we remove the last timeline/edge traversed from 
         * the potential cycle description. 
         *
         * @throw has_cycle No potential cycle detected
         */
        void finish_vertex(graph::reactor_id r, graph const &g) {
          switch(timelineType) {
            case external :
              if(r==root) {
                throw has_cycle(false);
              }
              break;
            case internal :
              if(r==root) {
                throw has_cycle(false);
              }
              break;
            default :
              throw has_cycle(false); // Should never happen
          }
          //Taking off the last reactor added to the path
          path.pop_back();
        }

      private:
        /** @brief Timeline owner
         *
         * The owner or future owner of the timeline node attempts to connect to. 
         * If the relation is Internal it should be the same as node.
         */
        graph::reactor_id const& root;
        /** @brief Creator of the connection
         *
         * The reactor that initiales this connection
         */
        graph::reactor_id const& node;
        /** @brief Timeline involved in the connection
         *
         * The name of the timeline the reactor node attempts to declare
         */
        utils::Symbol name;
        /** @brief current timelines path
         *
         * The list of all the timlines currently traversed by the search. 
         * If a cycle is detected this path describes it.
         */
        std::list<utils::Symbol> path;
        /** @brief Type of attempted connection
         */
        Cycle_Type timelineType;
      };


    }
  }
}

AgentException::AgentException(graph const &agent, std::string const &msg) throw()
  :GraphException(agent, msg) {}

/*
 * class TREX::agent::Agent
 */

// static

TICK Agent::initialTick(Clock *clk) {
  return (NULL==clk)?0:clk->initialTick();
}

// structors :

Agent::Agent(Symbol const &name, TICK final, Clock *clk, bool verbose)
  :graph(name, initialTick(clk), verbose),
   m_clock(clk), m_finalTick(final), m_valid(true) {
  m_proxy = new AgentProxy(*this);
  add_reactor(m_proxy);
}

Agent::Agent(std::string const &file_name, Clock *clk, bool verbose)
  :m_clock(clk), m_valid(true) {
  set_verbose(verbose);
  updateTick(initialTick(m_clock), false);
  m_proxy = new AgentProxy(*this);
  add_reactor(m_proxy);
  try {
    loadConf(file_name);
  } catch(TREX::utils::Exception const &e) {
    syslog(null, error)<<"Exception caught while loading "<<file_name<<":\n"
		       <<e;
    throw;
  } catch(std::exception const &se) {
    syslog(null, error)<<"C++ exception caught while loading "
		       <<file_name<<":\n"
		       <<se.what();
    throw;
  } catch(...) {
    syslog(null, error)<<"Unknown exception caught while loading "
		       <<file_name;
    throw;
  }
}

Agent::Agent(boost::property_tree::ptree::value_type &conf, Clock *clk, bool verbose)
  :m_clock(clk), m_valid(true) {
  set_verbose(verbose);
  updateTick(initialTick(m_clock), false);
  m_proxy = new AgentProxy(*this);
  add_reactor(m_proxy);
  try {
    loadConf(conf);
  } catch(TREX::utils::Exception const &e) {
    syslog(null, error)<<"Exception caught while loading XML:\n"
		       <<e;
    throw;
  } catch(std::exception const &se) {
    syslog(null, error)<<"C++ exception caught while loading XML:\n"
		       <<se.what();
    throw;
  } catch(...) {
    syslog(null, error)<<"Unknown exception caught while loading XML";
    throw;
  }
}


Agent::~Agent() {
  m_valid = false;
    
  m_proxy = NULL;
  if( m_stat_log.is_open() )
    m_stat_log.close();
  if( NULL!=m_clock )
    delete m_clock;
  clear();
}

// observers :

bool Agent::missionCompleted() {
  if( !valid() ) {
    syslog(null, null)<<"Agent destroyed.";
    return true;
  }
  if( count_reactors()<=1 ) {
    // If there's only one reactor left it is probably
    // my AgentProxy => no real reactor available
    syslog(null, info)<<"No reactor left.";
    return true;
  }
  return getCurrentTick()>m_finalTick;
}

void Agent::internal_check(reactor_id r,
			   TREX::transaction::details::timeline const &tl) {
    if(index(r)==count_reactors()) return;
    if(tl.owned())
    {
        std::stringstream msg;
        msg <<"Timeline "<<tl.name()<<" already owned by "<<tl.owner().getName();
        throw timeline_failure(r, msg.str());
    }

    details::cycle_checker checkInternal(r, r, details::cycle_checker::internal, tl.name());
    try {
        boost::depth_first_search(me(), boost::visitor(checkInternal).root_vertex(r));
    } catch (details::cycle_checker::has_cycle check) {
        if(check.cycle)
        {
            throw timeline_failure(r, check.message);
            // throw timeline_failure(...) <= indicate to reactor_graph that
            //this connection is not allowed
        }
    }
}

void Agent::external_check(reactor_id r,
			   TREX::transaction::details::timeline const &tl) {
    if(index(r)==count_reactors()) return;
    if(tl.owned())
    {
        details::cycle_checker checkExternal(r, &(tl.owner()), details::cycle_checker::external, tl.name());
        try {
        boost::depth_first_search(me(), boost::visitor(checkExternal).root_vertex(&(tl.owner())));
        } catch (details::cycle_checker::has_cycle check) {
            if(check.cycle)
            {
                throw timeline_failure(r, check.message);
                //throw timeline_failure(...) <= indicate to reactor_graph that
                //this connection is not allowed
            }
        }
    }
}


// manipulators :


bool Agent::setClock(Clock *clock) {
  if( NULL==m_clock ) {
    m_clock = clock;
    updateTick(initialTick(m_clock), false);
    return true;
  } else {
    delete clock;
    return false;
  }
}

void Agent::loadPlugin(boost::property_tree::ptree::value_type &pg,
                       std::string path) {
  std::string name = parse_attr<std::string>(pg, "name");
  boost::optional<boost::property_tree::ptree &> else_tree = pg.second.get_child_optional("Else");
  boost::property_tree::ptree *sub = NULL;
  
  if( m_pg->load(name, !else_tree) ) {
    // Sucessfully loaded the plug-in 
    //   => sub tree is teis tree
    sub = &(pg.second);
  } else {
    // Failed to loacate the plug-in
    //   => sub tree is <Else /> 
    std::string msg = parse_attr<std::string>("", *else_tree, "message");
    if( msg.empty() )
      syslog(path, info)<<"Failed to locate plug-in "<<name;
    else
      syslog(path, info)<<"Failed to locate plug-in "
        <<name<<"\n\t"<<msg;
    
    name = "(!"+name+")";
    sub = &(*else_tree); 
  }
  // Traverse the sub tree 
  if( path.empty() )
    path = name;
  else 
    path += "."+name;
  subConf(*sub, path);
}

void Agent::subConf(boost::property_tree::ptree &conf, 
                    std::string const &path) {
  boost::property_tree::ptree::assoc_iterator i, last;
  SingletonUse<Clock::xml_factory> clk_f;
  
  if( path.empty() ) {
    // On the root we need to load plug-ins before the clock 
    // so we are bacward compatible
    
    // Look for Use directives
    boost::tie(i, last) = conf.equal_range("Use");
    for(; last!=i; ++i) {
      bool found;
      manager().use(parse_attr<std::string>(*i, "file"), found);
    }


    // Looks for plug-ins at this level
    boost::tie(i, last) = conf.equal_range("Plugin");
    if( last!=i ) {
      // Produce new reactors
      syslog(path, info)<<"Loading plug-ins...";
      for(; last!=i; ++i) 
        loadPlugin(*i, path);
    }

    // check if need/can load a new clock
    if( NULL==m_clock ) {
      syslog(path, info)<<"Check for clock definition...";
      boost::property_tree::ptree::iterator c = conf.begin();
      clk_f->iter_produce(c, conf.end(), m_clock);
    }
  } else {
    // check if need/can load a new clock
    if( NULL==m_clock ) {
      syslog(path, info)<<"Check for clock definition...";
      boost::property_tree::ptree::iterator c = conf.begin();
      clk_f->iter_produce(c, conf.end(), m_clock);
    }

    // Looks for plug-ins at this level
    boost::tie(i, last) = conf.equal_range("Plugin");
    if( last!=i ) {
      // Produce new reactors
      syslog(path, info)<<"Loading plug-ins...";
      for(; last!=i; ++i) 
        loadPlugin(*i, path);
    }
  }
    
  // Produce new reactors
  syslog(path, info)<<"Loading reactors...";
  add_reactors(conf);

  // Now load and post the goals
  syslog(path, info)<<"loading goals....";
  sendRequests(conf);
} 

void Agent::loadConf(boost::property_tree::ptree::value_type &config) {
  if( !is_tag(config, "Agent") )
    throw XmlError(config, "Not an Agent node");
  // Extract attributes
  try {
    Symbol name(parse_attr<std::string>(config, "name"));

    if( name.empty() )
      throw XmlError(config, "Agent name is empty.");
    set_name(name);
    
    m_finalTick = parse_attr<TICK>(config, "finalTick");
    if( m_finalTick<=0 )
      throw XmlError(config, "agent life time should be greater than 0");
  } catch(bad_string_cast const &e) {
    throw XmlError(config, e.what());
  }
  // Populate with external configuration
  ext_xml(config.second, "config");

  // Now iterate through the sub-tags
  if( config.second.empty() )
    throw XmlError(config, "Agent node does not have sub nodes.");

  subConf(config.second, "");
  syslog(null, info)<<"End of init.";
}

void Agent::loadConf(std::string const &file_name) {
  bool found;
  std::string name = manager().use(file_name, found);
  if( !found ) {
    name = manager().use(file_name+".cfg", found);
    if( !found )
      throw ErrnoExcept("Unable to locate "+file_name);
  }
  boost::property_tree::ptree agent;
  read_xml(name, agent, xml::no_comments|xml::trim_whitespace);

  if( agent.empty() )
    throw TREX::utils::Exception(std::string("Configuration file : \"")+
				 file_name+"\" is empty.");
  if( agent.size()!=1 )
    throw TREX::utils::Exception(std::string("Configuration file : \"")+
				 file_name+"\" have multiple roots.");
  loadConf(agent.front());
}

void Agent::initComplete() {
  if( getName().empty() )
    throw AgentException(*this, "Agent has no name :"
			 " You probably forgot to initialize it.");
  if( NULL==m_clock )
    throw AgentException(*this, "Agent is not connected to a clock");
  // Complete the init for the reactors and check that there's 
  // no cycle
  details::init_visitor init(m_finalTick);
  boost::depth_first_search(me(), boost::visitor(init));
  size_t n_failed = cleanup();
  if( n_failed>0 )
    syslog(null, warn)<<n_failed<<" reactors failed to initialize.";

  // Check for missing timelines
  for(timeline_iterator it=timeline_begin(); timeline_end()!=it; ++it)
    if( !(*it)->owned() )
      syslog(null, warn)<<"Timeline \""<<(*it)->name()<<"\" has no owner.";


  // Create initial graph file
  LogManager::path_type graph_dot = manager().file_name("reactors.gv");
  std::ofstream dotf(graph_dot.c_str());
  m_stat_log.open(manager().file_name("agent_stats.csv").c_str());
  m_stat_log<<"tick, synch_ns, synch_rt_ns,"
    " delib_ns, delib_rt_ns, delib_steps,"
    " sleep_ns"<<std::endl;

  graph_names_writer gn;
  boost::write_graphviz(dotf, me(), gn, gn);
  syslog(null, info)<<"Initial graph logged in \"reactors.gv\".";


  // start the clock
  m_clock->doStart();
  syslog(null, "START")<<"\t=========================================================";
  updateTick(m_clock->tick());

}

void Agent::run() {
  initComplete();
  while( doNext() );
  syslog(null, info)<<"Mission completed."<<std::endl;
}


void Agent::synchronize() {
  details::sync_scheduller::reactor_queue queue;
  details::sync_scheduller sync(queue);

  m_edf.clear(); // Make sure that there's no one left in the schedulling
  m_idle.clear();
  bool update = false;

  stat_clock::time_point synch_start = stat_clock::now();
  rt_clock::time_point synch_start_rt = rt_clock::now();

  // Identify synchronization order while notifying of the new tick
  boost::depth_first_search(me(), boost::visitor(sync));
  size_t n_failed = cleanup();
  if( n_failed>0 )
    syslog(null, warn)<<n_failed<<" reactors failed to start tick "
		      <<getCurrentTick();
  // Execute synchronization
  //  - could be done with a dfs but we choose for now to do it using the
  //  output list of sync_scheduller to avoid a potentially costfull graph
  //  exploration.
  while( !queue.empty() ) {
    reactor_id r = queue.front();
    queue.pop_front();        
    // synchronization
    if( r->doSynchronize() ) {
      double wr = r->workRatio();

      if( !std::isnan(wr) ) {
	// this reactor has deliberation :
	//   - add it to the edf scheduler
	m_edf.insert(std::make_pair(wr, r));
      } else { 
        m_idle.push_front(r);
      }
    } else {
      // r failed => kill the reactor
      kill_reactor(r);
      update = true;
    }
  }
  m_stat_log<<getCurrentTick()
	    <<", "<<(stat_clock::now()-synch_start).count()	    
	    <<", "<<(rt_clock::now()-synch_start_rt).count()
	    <<std::flush;
  if( update ) {
    // Create new graph file
    std::ostringstream name;
    name<<"reactors."<<getCurrentTick()<<".gv";
    
    LogManager::path_type graph_dot = manager().file_name(name.str());
    std::ofstream dotf(graph_dot.c_str());
    
    graph_names_writer gn;
    boost::write_graphviz(dotf, me(), gn, gn);
    syslog(null, info)<<"New graph logged in \""<<name.str()<<"\".";    
  }
}

bool Agent::executeReactor() {
  if( m_edf.empty() )
    return false;
  else {
    double wr;
    reactor_id r;

    boost::tie(wr, r) = *(m_edf.begin());
    m_edf.erase(m_edf.begin());
    m_idle.push_back(r);
    try {
      r->step();
      
      std::list<reactor_id>::iterator i = m_idle.begin();
      
      while( m_idle.end()!=i ) {
        // Check if the reactor is still valid 
        if( is_member(*i) ) {
          wr = (*i)->workRatio();
          if( !std::isnan(wr) ) {
            m_edf.insert(std::make_pair(wr, *i));
            i = m_idle.erase(i);
          } else 
            ++i;
        } else 
          i = m_idle.erase(i);
      }
    } catch(Exception const &e) {
      syslog(null, warn)<<"Exception caught while executing reactor step:\n"<<e;
    } catch(std::exception const &se) {
      syslog(null, warn)<<"C++ exception caught while executing reactor step:\n"
			<<se.what();      
    } catch(...) {
      syslog(null, warn)<<"Unknown exception caught while executing reactor step.";            
    }
    return !m_edf.empty();
  }
}

bool Agent::doNext() {
  if( missionCompleted() ) {
    if( empty() )
      syslog(null, info)<<"No more reactor active.";
    syslog(null, "END")<<"\t=========================================================";
    return false;
  }
  synchronize();
  size_t count = 0; //slp_count = 0;
  bool completed = false;
  stat_clock::time_point start_delib = stat_clock::now();
  rt_clock::time_point start_delib_rt = rt_clock::now();
  stat_duration delib;
  rt_clock::duration delib_rt, sleep_time;
  bool print_delib = true;

  try {  
    while( m_clock->tick()==getCurrentTick()
	   && m_clock->is_free() && valid() 
	   && executeReactor() ) {
      ++count;
    }
    completed = true;
    delib = stat_clock::now()-start_delib;
    delib_rt = rt_clock::now()-start_delib_rt;
      
    m_stat_log<<", "<<delib.count()<<", "<<delib_rt.count()
	      <<", "<<count<<", "<<std::flush;
    print_delib = false;
    
    
    rt_clock::time_point start_sleep = rt_clock::now();
    while( valid() && m_clock->tick()==getCurrentTick() ) {
      m_clock->sleep();
      // ++slp_count;
    }
    sleep_time = rt_clock::now()-start_sleep;

    if( valid() )
      updateTick(m_clock->tick());
  } catch(Clock::Error const &err) {
    syslog(null, error)<<"error from the clock: "<<err;
    m_valid = false;
    if( !completed )
      delib = stat_clock::now()-start_delib;
  } 
  if( print_delib )
    m_stat_log<<", "<<delib.count()<<", "<<count<<", ";
  m_stat_log<<sleep_time.count()<<std::endl;

  return valid();
}

void Agent::sendRequest(goal_id const &g) {
  if( !has_timeline(g->object()) )
    syslog(null, warn)<<"Posting goal on a unknnown timeline \""
		      <<g->object()<<"\".";
  m_proxy->postRequest(g);
}

size_t Agent::sendRequests(boost::property_tree::ptree &g) {
  size_t ret = 0;
  boost::property_tree::ptree::assoc_iterator i, last;
  boost::tie(i, last) = g.equal_range("Goal");
  for(; last!=i; ++i) {
    sendRequest(*i);
    ++ret;
  }
  syslog(null, info)<<ret<<" goal"<<(ret>1?"s":"")<<" loaded.";
  return ret;
}


