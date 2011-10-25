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
#define BOOST_EXCEPTION_DISABLE 
#include <boost/graph/graphviz.hpp>
#include <boost/graph/topological_sort.hpp>


using namespace TREX::agent;
using namespace TREX::transaction;
using namespace TREX::utils;

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
	  if( is_valid(r) ) {
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
	  if( is_valid(r) )
	    r->newTick();
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
	  if( is_valid(r) )
	    m_sync->push_back(r);
	}

      private:
	/** @brief Reference to scheduling list */
	reactor_queue               *m_sync;

	sync_scheduller(); // no code in purpose
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

Agent::Agent(Symbol const &name, TICK final, Clock *clk) 
  :graph(name, initialTick(clk)), 
   m_clock(clk), m_finalTick(final) {
  m_proxy = new AgentProxy(*this);
  add_reactor(m_proxy);
}

Agent::Agent(std::string const &file_name, Clock *clk) 
  :m_clock(clk) {
  updateTick(initialTick(m_clock));
  m_proxy = new AgentProxy(*this);
  add_reactor(m_proxy);
  try {
    loadConf(file_name);
  } catch(TREX::utils::Exception const &e) {
    syslog("ERROR")<<"Exception caught while loading "<<file_name<<":\n"
		   <<e;
    throw;
  } catch(std::exception const &se) {
    syslog("ERROR")<<"C++ exception caught while loading "
		   <<file_name<<":\n"
		   <<se.what();
    throw;
  } catch(...) {
    syslog("ERROR")<<"Unknown exception caught while loading "
		   <<file_name;
    throw;
  }
}

Agent::Agent(rapidxml::xml_node<> &conf, Clock *clk) 
  :m_clock(clk) {
  updateTick(initialTick(m_clock));
  m_proxy = new AgentProxy(*this);
  add_reactor(m_proxy);
  try {
    loadConf(conf);
  } catch(TREX::utils::Exception const &e) {
    syslog("ERROR")<<"Exception caught while loading XML:\n"
		   <<e;
    throw;
  } catch(std::exception const &se) {
    syslog("ERROR")<<"C++ exception caught while loading XML:\n"
		   <<se.what();
    throw;
  } catch(...) {
    syslog("ERROR")<<"Unknown exception caught while loading XML";
    throw;
  }
}


Agent::~Agent() {
  m_proxy = NULL;
  clear();
  if( NULL!=m_clock )
    delete m_clock;
}

// observers :

bool Agent::missionCompleted() {
  if( count_reactors()<=1 ) {
    // If there's only one reactor left it is probably
    // my AgentProxy => no real reactor available
    syslog()<<"No reactor left.";
    return true;
  }
  return getCurrentTick()>m_finalTick;
}

// manipulators :

bool Agent::setClock(Clock *clock) {
  if( NULL==m_clock ) {
    m_clock = clock;
    updateTick(initialTick(m_clock));
    return true;
  } else {
    delete clock;
    return false;
  }
}

void Agent::loadPlugin(rapidxml::xml_node<> &pg) {
  Symbol name = parse_attr<Symbol>(pg, "name");
  m_pg->load(name);
}

void Agent::loadConf(rapidxml::xml_node<> &config) {
  if( !is_tag(config, "Agent") ) 
    throw XmlError(config, "Not an Agent node");
  // Extract attributes
  try {
    Symbol name = parse_attr<std::string>(config, "name");
    if( name.empty() )
      throw XmlError(config, "Agent name is empty.");
    set_name(name);
    m_finalTick = parse_attr<TICK>(config, "finalTick");
    if( m_finalTick<=0 ) 
      throw XmlError(config, "agent life time should be greater than 0");
  } catch(bad_string_cast const &e) {
    throw XmlError(config, e.what());
  }
  // Now iterate through the sub-tags 
  ext_iterator const c_start(config, "config"); 
  ext_iterator iter; 

  if( !c_start.valid() )
    throw XmlError(config, "Agent node do not have sub nodes.");

  // First load the plugins :
  syslog()<<"Loading plug-ins...";
  iter = c_start;
  if( is_tag(*iter, "Plugin") )
    loadPlugin(*iter);
  for( iter.next("Plugin"); iter.valid(); iter.next("Plugin") )
    loadPlugin(*iter);

  
  // Look for A clock : Note only the first clock found is used
  if( NULL==m_clock ) {
    SingletonUse<Clock::xml_factory> clk_f;
    iter = c_start;
    clk_f->iter_produce(iter, m_clock);
  }
  // List the reactors available for helping the user :
  SingletonUse<xml_factory> reactor_f;
  
  // First look for the reactor types available
  std::list<Symbol> reactors_types;

  reactor_f->getIds(reactors_types);
  if( reactors_types.empty() ) {
    syslog()<<"ERROR : no reactor types available.";
  } else {
    // Display the list of reactor types
    std::ostringstream oss;

    while( !reactors_types.empty() ) {
      oss<<"\n\t- "<<reactors_types.front();
      reactors_types.pop_front();
    }
    syslog()<<"Loading reactors; available reactor types are :"<<oss.str();

    iter = c_start;
    add_reactors(iter);			  
  }
  
  // Now load and post the goals 
  syslog()<<"load initial goals of the agent";
  iter = c_start;
  sendRequests(iter);
}

void Agent::loadConf(std::string const &file_name) {
  bool found;
  std::string name = manager().use(file_name, found);
  if( !found ) {
    name = manager().use(file_name+".cfg", found);
    if( !found )
      throw ErrnoExcept("Unable to locate "+file_name);
  }
  rapidxml::file<> file(name.c_str());
  rapidxml::xml_document<> doc;
  doc.parse<0>(file.data());
  rapidxml::xml_node<> *root = doc.first_node();
  if( NULL==root ) 
    throw TREX::utils::Exception(std::string("Configuration file : \"")+
				 file_name+"\" is empty.");
  loadConf(*root);
}

void Agent::initComplete() {
  if( getName().empty() ) 
    throw AgentException(*this, "Agent has no name :"
			 " You probably forgot to initialize it.");
  if( NULL==m_clock )
    throw AgentException(*this, "Agent is not connected to a clock");
  // Complete the init for the reactors and check that there's no cycle
  details::init_visitor init(m_finalTick);
  boost::depth_first_search(me(), boost::visitor(init));
  size_t n_failed = cleanup();
  if( n_failed>0 ) 
    syslog("WARN")<<n_failed<<" reactors failed to initialize.";
  
  // Check for missing timelines 
  for(timeline_iterator it=timeline_begin(); timeline_end()!=it; ++it) 
    if( !(*it)->owned() ) 
      syslog("WARN")<<"Timeline \""<<(*it)->name()<<"\" has no owner.";
  

  // Create initial graph file
  std::string graph_dot = manager().file_name("reactors.dot");
  std::ofstream dotf(graph_dot.c_str());
  
  graph_names_writer gn;
  boost::write_graphviz(dotf, me(), gn, gn);
  syslog()<<"Initial graph logged in \"reactors.dot\".";


  // start the clock
  m_clock->doStart();
  syslog("START")<<"\t=========================================================";
}

void Agent::run() {
  initComplete();
  while( doNext() );
  syslog()<<"Mission completed."<<std::endl;
}


void Agent::synchronize() {
  details::sync_scheduller::reactor_queue queue;
  details::sync_scheduller sync(queue);

  // Identify synchronization order while notifying of the new tick
  boost::depth_first_search(me(), boost::visitor(sync));
  // Execute synchronization
  //  - could be done with a dfs but we choose for now to do it using the
  //  output list of sync_scheduller to avoid a potentially costfull graph
  //  exploration.
  while( !queue.empty() ) {
    reactor_id r = queue.front();
    queue.pop_front();
    // Update timelines for r
    r->doNotify();

    // synchronization
    if( r->doSynchronize() ) {
      double wr = r->workRatio();

      if( !std::isnan(wr) ) {
	// this reactor has deliberation :
	//   - add it to the edf scheduler
	m_edf.insert(std::make_pair(wr, r));
      }
    } else {
      // r failed => kill the reactor
      kill_reactor(r);
    }
  }
}

bool Agent::executeReactor() {
  if( m_edf.empty() )
    return false;
  else {
    double wr;
    reactor_id r;

    boost::tie(wr, r) = *(m_edf.begin());
    // syslog("step")<<"Executing reactor "<<r->getName()<<" (wr="<<wr<<")";
    m_edf.erase(m_edf.begin());
    r->step();

    wr = r->workRatio();
    if( !std::isnan(wr) ) {
      // syslog("step")<<"New work ratio for "<<r->getName()<<" is "<<wr;
      m_edf.insert(std::make_pair(wr, r));
      return true;
    }
    return !m_edf.empty();
  }
}
    
bool Agent::doNext() {
  if( missionCompleted() ) {
    if( empty() )
      syslog()<<"No more reactor active.";    
    syslog("END")<<"\t=========================================================";
    return false;
  }
  synchronize();
  
  while( m_clock->getNextTick()==getCurrentTick() && executeReactor() );
  while( m_clock->getNextTick()==getCurrentTick() )
    m_clock->sleep();
  
  updateTick(m_clock->getNextTick());

  return true;
}

void Agent::sendRequest(goal_id const &g) {
  if( !has_timeline(g->object()) )
    syslog("WARN")<<"Posting goal on a unknnown timeline \""<<g->object()<<"\".";
  m_proxy->postRequest(g);
}

size_t Agent::sendRequests(ext_iterator &iter) {
  size_t ret = 0;
  if( iter.valid() ) {
    for(iter = iter.find_tag("Goal"); iter.valid(); iter = iter.next("Goal") ) {
      sendRequest(*iter);
      ++ret;
    }
    syslog()<<ret<<" goal"<<(ret>1?"s":"")<<" loaded.";
  } else 
    syslog()<<"There's no tag in this file.";
  return ret;
}


