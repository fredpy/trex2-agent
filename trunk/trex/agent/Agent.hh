/* -*- C++ -*- */
/** @file "Agent.hh"
 * @brief Definition of an Agent
 * 
 * Provides declaration for the Agent class. This is basically a composition
 * of TeleoReactors. The Agent will allocate reactors and connect them
 * together according to their configuratiun requirements.
 *
 * It will also play the role of middle-man to route observations from sender
 * to receiver.
 * 
 * @note The Agent class is not thread safe. If run on a separate thread,
 * it cannot be accessed further.
 * 
 * @author Conor McGann @& Frederic Py <fpy@mbari.org>
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
#ifndef H_Agent
# define H_Agent

# include "Clock.hh"
# include <trex/utils/PluginLoader.hh>
# include <trex/utils/asio_fstream.hh>

namespace TREX {
  namespace agent {
    
    typedef SHARED_PTR<Clock> clock_ref;
    
    /** @brief TREX agent
     *
     * This class implements the TREX agent. It extends the graph concept
     * by:
     * @li enforcing that there's no cyclic dependency between reactors
     * @li calling the different methods of the reactors in order to execute
     *     them
     *
     * The execution of the reactors is made according to a clock which the
     *  agent observes in order to idenitfy a new tick that will gives the rythm
     *  of execution
     *
     * As soon as the reactor graph as been fully loaded the agent will call in
     * each reactors the @c TeleoReactor::initialize method while controlling that
     * there's no cyclic dependencies in the graph and idenitfy if there's some
     * Internal timelines missing.
     *
     * After this ti will start the clock and ate each tick will execute the
     * following callbacks of the reactors int the following order
     * @li TeleoReactor::newTick indicate to each reactor that a new tick
     *     have started
     * @li TeleoReactor::doSynchronize() require to each reactors to identify
     *     their current state based on the observation they did receive and
     *     potentially produce new observation on their intiernal timelines
     * @li TeleoReactor::workRatio() identify each reactor workload for scedulling
     *     their deliberation
     * @li TeleoReactor::step() is called only if a reactor have works (idenitifed by
     *    the fact that is workRatio() was not @c NAN). It allow the reactor to do a
     *    single step in roder to deliberate.
     * 
     * Do note that ass all theses called have to be executed in the duration of a
     * TICK all of these should be executed in a duration that is well below the
     * tick granularity of the clock used by the agent.
     *
     * @ingroup agent
     * @author Frederic Py <fpy@mbari.org>
     * @sa class TREX::transaction::TeleoReactor
     * @sa class Clock
     */
    class Agent : public TREX::transaction::graph {
    public:
      /** @brief reactors deliberation scheduling queue
       *
       * This structure is used to find a good scheduling order for the reactors
       * deliberation. It sorts the reactors that needs to deliberate from the one
       * with the largest @c workRatio to the one with smaller @c workRatio
       */
      typedef std::multimap< double, reactor_id,
			     std::greater<double> > priority_queue;

      /** @brief Constructor
       *
       * @param[in] name The name of the agent
       * @param[in] final The value of the final tick for this mission
       * @param[in] clock A pointer to the clock that this agent will use
       *
       * Create a new Agent named @p name with and intialize both its end and clock
       * according to the optional arguments @p final and @p clock
       *
       * @post the agent has no reactor
       */
      explicit Agent(TREX::utils::symbol const &name,
		     TREX::transaction::TICK final = 0,
		     clock_ref clock = clock_ref(), bool verbose=false);
      /** @brief Constructor
       *
       * @param[in] file_name A configuration file name
       * @param[in] clock A pointer to the clock that this agent will use.
       *
       * Create a new Agent by parsing the onctent of the file @f file_name.
       * If @p clock is not provided, it will take the first clock defined
       * in @p file_name.
       * Create a new Agent by parsing the content of the xml structure @p conf.
       * If the clock has not beeen set yet, it will take the first clock defined
       * in @p file_name.
       *
       * An Agent configuration xml definition can be defined as follow:
       * @code
       * <Agent name="<agent name>" finalTick="<final tick>" config="<extra cfg>" >
       *    <!-- plugin loading information -->
       *    <!-- clocks defintions -->
       *    <!-- reactors definitions -->
       *    <!-- goal definitions -->
       * </Agent>
       * @endcode
       *
       * Where the attributes are as follow
       * @li @c name is the name of the agent
       * @li @c finalTick is a value greater than 0 that indicates
       *     the agent lifetime
       * @li @c config is an optional attribute that points to another XML file.
       *     this file will contains extra tags that will be parse in simlar mananer
       *     to the childs of this root tag.
       *
       * the child tags will be parsed in the following order:
       * @li Plugin information allowing TREX to load external plugins. These
       *     XML tags have the following syntax :
       *     @code
       *     <Plugin name="TREXvitre" />
       *     @endcode
       *     and will result on the  attempting to load the dynamic libray @e TREXvitre
       *     as a TREX plugin. These need to be loaded firast as oner plugin can defines
       *     new clocks or reactors types used by this agent
       * @li Clock definition. These will be parsed only if a clock is not defined and as
       *     a result only the first clock definition may be parsed in this configuration
       *     The tag of the XML depends on the way the clock class declared itself inside
       *     TREX. And you need to look at the documentation of the clock class you want
       *     to use in order to know how to declare it in this file.
       * @li Reactors definition. these will define all the reactors that are declared
       *     initially on this agent. As for clock, the way to specify a reactor depends
       *     on the type of reactor and onwe should look at its documentation to know
       *     further
       * @li The initial goals for this mission. Goals have generic xml format that
       *     is described in TREX::transaction::Goal class documentation
       * 
       *
       * This constructor will try first to locate @p file_name and if does
       * not find it will try to locate @p file_name.cfg
       *
       * @pre @p file_name is a a valid file on current directory or accessible
       *      through TREX_PATH
       * @pre @p file_name is a valid xml file that defines an agent 
       * @pre the root node of @p file_name needs to be an Agent tag
       *
       *
       * @throw TREX::utils::ErrnoExcept failed to locate the file
       * @throw rapidxml::parse_error parse error while loading the file
       * @throw TREX::utils::XmlError invalid XML content for agent configuration
       * @throw TREX::utils::Exception one exception occured while laoding the agent
       *
       *
       * @sa Agent::Agent(rapidxml::xml_node<> &, Clock *)
       * @sa Agent::loadConf(std::string const &)
       */
      explicit Agent(std::string const &file_name, 
		     clock_ref clock = clock_ref(), bool verbose=false);
      
      /** @brief Constructor
       *
       * @param[in] config A xml structure
       * @param[in] clock A pointer to the clock that this agent will use.
       *
       * Create a new Agent by parsing the content of the XML node @p config
       * If @p clock is not provided, it will take the first clock defined
       * in @p file_name.
       *
       * This constructor will try first to locate @p file_name and if does
       * not find it will try to locate @p file_name.cfg
       *
       * @pre @p config root node is Agent
       * @pre @p file_name is a valid xml file that defines an agent 
       *
       * @throw TREX::utils::ErrnoExcept failed to locate the file
       * @throw rapidxml::parse_error parse error while loading the file
       * @throw TREX::utils::XmlError invalid XML content for agent configuration
       * @throw TREX::utils::Exception one exception occured while laoding the agent
       *
       *
       * @sa Agent::Agent(std::string const &, Clock *)
       * @sa Agent::loadConf(rapidxml::xml_node<> &)
       */
      explicit Agent(boost::property_tree::ptree::value_type &config, 
		     clock_ref clock = clock_ref(), bool verbose=false);
      /** @brief Destructor */
      ~Agent(); 
      
      /** @brief Set agent clock
       *
       * @param[in] clock A clock
       *
       * If the agent did not have a clock yet it is then associated to @p clock.
       * Otherwise @p clock will be deleted
       * 
       * @retval true @p clock is now the clock of this agent
       * @retval false this agent had already a clock and @e deleted @p clock
       *
       * @pre clock has been allocated in memory using the @c new operator
       * @post the laifetime of @p clock is now taken coare by this agent. Do note
       *       that this post-condition is true no matter what value has been
       *       returned by this function.
       */
      bool setClock(clock_ref clock);
      
      clock_ref getClock() const {
        return m_clock;
      }
      
      /** @brief run the agent
       *
       * This method runs the agent until its completes. It is just a
       * simple utility method taht will call initComplete and then
       * loop on a doNext() call untils this one returns false
       *
       * @pre The agent have a clock set
       *
       * @sa intiComplete()
       * @sa doNext()
       */
      void run();
      /** @brief Complete agent initialization
       *
       * Thsi function should be called before starting to execute an agent
       * mission. It specifies to the agent that the initialization has been
       * completed. This method makes final checks that :
       * @li the agent name is not empty
       * @li the agent have a clock associated
       * @li there's no cyclic dependencies between reactors
       *
       * It will also use a TREX::agent::details::init_visitor to ensure that all
       * the reactors can complete their initialization
       * Finally it starts the clock alllowing the mission to be executed
       *
       * @sa TREX::agent::details::init_visitor
       * @sa setClock(Clock *)
       *
       * @throw CycleDetected There's a cyclic dependency between the reactors of this agent
       * @throw AgentException The eagent is not correctly initialized
       *
       */
      void initComplete();
      /** @brief Execute for one tick
       *
       * This method execute all the reactors of the graphs for the current
       * tick. This execution can be descibed as follow
       * @li TeleoReactor::newTick indicate to each reactor that a new tick
       *     have started
       * @li TeleoReactor::doSynchronize() require to each reactors to identify
       *     their current state based on the observation they did receive and
       *     potentially produce new observation on their intiernal timelines
       * @li TeleoReactor::workRatio() identify each reactor workload for scedulling
       *     their deliberation
       * @li TeleoReactor::step() is called only if a reactor have works (idenitifed by
       *    the fact that is workRatio() was not @c NAN). It allow the reactor to do a
       *    single step in roder to deliberate.
       *
       * The function will return as soon as we reached the next tick.
       *
       * @pre @c initComplete should have been called before
       *
       * @retavl true the mission is not finished
       * @retval false end of agent mission
       *
       * @post the current tick is larger than it was when we called this method
       *
       * @sa initComplete()
       * @sa missionCompleted()
       */
      bool doNext();

      /** @brief Check if mission is completed
       *
       * This methods indicates whehter tha mission of this agent has finished or not.
       * Reasons for an agent to finish are one of the following:
       * @li the time of the clock is greater or equal to the deadline initially provided
       * for this agent
       * @li the agent does not have anymore reactor in its graph
       *
       * @retval true if the mission is finished
       * @retval false otherwise
       */
      bool missionCompleted();

      /** @brief Post a goal
       *
       * @param[in] g A goal
       * This method will add the goal @p g to the agent. This goal will be posted
       * to whichever reactor owns the timeline associated to @p g
       */
      void sendRequest(TREX::transaction::goal_id const &g);
      /** @brief Post a goal from xml 
       *
       * @param[in] g A goal in xml
       * 
       *  This method will insert the goal defined by xml structure @p g
       *
       *  @sa TREX::transaction::Goal::Goal(rapidxml::xml_node<> &)
       *  @sa sendRequest(TREX::transaction::goal_id const &)
       *  @sa sendRequests(TREX::utils::ext_iterator &)
       */
      void sendRequest(boost::property_tree::ptree::value_type &g) {
	TREX::transaction::goal_id tmp = parse_goal(g);
	sendRequest(tmp);
      }
      /** @brief Post a goals from xml 
       *
       * @param[in] g A xml iterator
       * 
       *  This method will insert all the goals that are acessibles by iterating
       *  through @p g
       *
       *  @sa TREX::transaction::Goal::Goal(rapidxml::xml_node<> &)
       *  @sa sendRequest(rapidxml::xml_node<> &)
       */
      size_t sendRequests(boost::property_tree::ptree &g);

      duration_type tickDuration() const {
	return m_clock->tickDuration();
      }
      
      TREX::transaction::TICK timeToTick(date_type const &date) const {
	return m_clock->timeToTick(date);
      }
      date_type tickToTime(TREX::transaction::TICK cur) const {
	return m_clock->tickToTime(cur);
      }
      std::string date_str(TREX::transaction::TICK cur) const {
        return m_clock->date_str(cur);
      }
      std::string duration_str(TREX::transaction::TICK dur) const {
        return m_clock->duration_str(dur);
      }
      
      transaction::TICK finalTick() const {
        return m_finalTick;
      }


      
      /** @brief Agent interraction proxy
       *
       * This reactor class is used by the agent as a proxy to interract with 
       * its reactors. The main use is for the Agent to transparently post goals
       * or recall them using the same interface as other reactors. It can also 
       * be extended to provide an interface with the other reactors that -- for example 
       * -- observe their timelines in order to display it to an end user.
       *
       * @ingroup agent
       * @author Frederic Py <fpy@mbari.org>
       */
      class AgentProxy :public TREX::transaction::TeleoReactor {
      public:
	AgentProxy(Agent &agent)
          :TREX::transaction::TeleoReactor(&agent, "", 0, 0) {}
	~AgentProxy() {}
	
	bool postRequest(TREX::transaction::goal_id const &g) {
          if( !isExternal(g->object()) )
            use(g->object());
          if( isExternal(g->object()) )
            return postGoal(g);
          else
            syslog(null, error)<<"Unable to subscribe to "<<g->object();
          return false;
	}
        
      protected:
	bool synchronize() {
	  return true;
        }
      };
      
      void set_proxy(AgentProxy *proxy) {
        if( NULL!=m_proxy )
          kill_reactor(m_proxy);
        m_proxy = proxy;
        add_reactor(m_proxy);
      }
      
    private:
      void internal_check(reactor_id r, 
			  TREX::transaction::details::timeline const &tl);
      void external_check(reactor_id r, 
			  TREX::transaction::details::timeline const &tl);
      
      std::list<reactor_id> init_dfs_sync();
      std::list<reactor_id> sort_reactors_sync();

      AgentProxy *m_proxy;
            

      void subConf(boost::property_tree::ptree &conf,
                   std::string const &path);

      /** @brief Load agent configuration 
       *
       * @param[in] conf A xml structure descirbing the agent
       *
       * Create a new Agent by parsing the content of the xml structure @p conf.
       * If the clock has not beeen set yet, it will take the first clock defined
       * in @p file_name.
       *
       * An Agent configuration xml definition can be defined as follow:
       * @code
       * <Agent name="<agent name>" finalTick="<final tick>" config="<extra cfg>" >
       *    <!-- plugin loading information -->
       *    <!-- clocks defintions -->
       *    <!-- reactors definitions -->
       *    <!-- goal definitions -->
       * </Agent>
       * @endcode
       *
       * Where the attributes are as follow
       * @li @c name is the name of the agent
       * @li @c finalTick is a value greater than 0 that indicates
       *     the agent lifetime
       * @li @c config is an optional attribute that points to another XML file.
       *     this file will contains extra tags that will be parse in simlar mananer
       *     to the childs of this root tag.
       *
       * the child tags will be parsed in the following order:
       * @li Plugin information allowing TREX to load external plugins. These
       *     XML tags have the following syntax :
       *     @code
       *     <Plugin name="witre_pg">
       *       <!-- plugin loading -->
       *       <!-- clock definition -->
       *       <!-- reactors definition -->
       *       <!-- goals -->
       *       [ <Else message="on fail"> ... </Else>  ]
       *     </Plugin>
       *     @endcode
       *     and will result on the  attempting to load the dynamic libray @e witre_pg
       *     as a TREX plugin. If it succeed it will parse the content of this node as 
       *     it did for the agent root oteherwise it will either throw an exception or 
       *     parse the Else node  
       * @li Clock definition. These will be parsed only if a clock is not defined and as
       *     a result only the first clock definition may be parsed in this configuration
       *     The tag of the XML depends on the way the clock class declared itself inside
       *     TREX. And you need to look at the documentation of the clock class you want
       *     to use in order to know how to declare it in this file.
       * @li Reactors definition. these will define all the reactors that are declared
       *     initially on this agent. As for clock, the way to specify a reactor depends
       *     on the type of reactor and onwe should look at its documentation to know
       *     further
       * @li The initial goals for this mission. Goals have generic xml format that
       *     is described in TREX::transaction::Goal class documentation
       * 
       *
       * @pre the root node of conf needs to be an Agent tag
       *
       * @throw TREX::utils::XmlError invalid XML content for agent configuration
       * @throw TREX::utils::Exception one exception occured while laoding the agent
       *      
       * @sa Agent::Agent(rapidxml::xml_node<> &, Clock *)
       * @sa Agent::loadPlugin(rapidxml::xml_node<> &)
       * @sa TREX::transaction::graph::add_reactors(TREX::utils::ext_iterator
       */
      void loadConf(boost::property_tree::ptree::value_type &conf);
      /** @brief Load agent configuration 
       *
       * @param[in] file_name A configuration file name
       *
       * Create a new Agent by parsing the onctent of the file @f file_name.
       * If the clock has not beeen set yet, it will take the first clock defined
       * in @p file_name.
       *
       * This method will try first to locate @p file_name and if does
       * not find it will try to locate @p file_name.cfg
       *
       * @pre @p file_name is a a valid file on current directory or accessible
       *      through TREX_PATH
       * @pre @p file_name is a valid xml file that defines an agent 
       *
       * @throw TREX::utils::ErrnoExcept failed to locate the file
       * @throw rapidxml::parse_error parse error while loading the file
       * @throw TREX::utils::XmlError invalid XML content for agent configuration
       * @throw TREX::utils::Exception one exception occured while laoding the agent
       *
       *
       * @sa Agent::Agent(std::string &, Clock *)
       * @sa Agent::loadConf(rapidxml::xml_node<> const &)
       */
      void loadConf(std::string const &file_name);

      static TREX::transaction::TICK initialTick(clock_ref clk);

      typedef TREX::transaction::TeleoReactor::stat_clock stat_clock;
      typedef stat_clock::duration     stat_duration;
      typedef TREX::transaction::TeleoReactor::rt_clock   rt_clock;

      TREX::utils::async_ofstream m_stat_log;

      clock_ref                  m_clock;
      TREX::transaction::TICK    m_finalTick;
      priority_queue             m_edf;
      std::list<reactor_id>      m_idle;
      
      mutable utils::shared_var<bool> m_valid;
      
      bool valid() const {
        utils::shared_var<bool>::scoped_lock lck(m_valid);
        return *m_valid;
      }

      void synchronize();
      
      bool executeReactor();

      void loadPlugin(boost::property_tree::ptree::value_type &pg,     
                      std::string path);

      /** @brief plug-in loader entry point */
      TREX::utils::singleton::use<TREX::utils::PluginLoader> m_pg;

    }; // TREX::agent::Agent

  } // TREX::agent
} // TREX

#endif
