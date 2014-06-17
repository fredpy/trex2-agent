/** @file "TeleoReactor.hh"
 * @brief Declares the main interface for a Teleo-Reactor
 *
 * Defines the basis to implement a Teleo-reactor
 *
 * @author Frederic Py <fpy@mbari.org> (initial version from Conor McGann)
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
#ifndef H_TeleoReactor
# define H_TeleoReactor

# include <cmath>

# include "bits/external.hh"
# include "reactor_graph.hh"

# include <trex/utils/TimeUtils.hh>
# include <trex/utils/chrono_helper.hh>
# include <trex/utils/cpu_clock.hh>
# include <trex/utils/asio_fstream.hh>

# if !defined(CPP11_HAS_CHRONO) && defined(BOOST_CHRONO_HAS_THREAD_CLOCK)
#  include <boost/chrono/thread_clock.hpp>
# endif

namespace TREX {
  namespace transaction {
    
    /** @brief TeleoReactor abstract interface
     *
     * This class provides the basic framework for a Teleo-Reactor as
     * defined for the TREX architecture.
     * A Teleo-reactor (or reactor) is the basic construct managed by a TREX
     * agent.
     *
     * It is the also the node of a transaction graph connected with other
     * reactors through the timeline relations it have with them. These
     * relations can be deduced from how a reactor declares the timelines it
     * manipulate:
     *
     * @li @e External timelines is a state variable provided by another reactor
     *     in the graph. The reactor can observe its evolution and -- even though
     *     it cannot directly control this timelien value -- it can potentially
     *     post goals on this timeline expressing desired future value(s).
     * @li @e Internal timelines is a state varaible managed by this reactor and
     *     made publicly available in the graph. It is the repsonsibility of this
     *     reactor to provide new Observation when the value of this timeline
     *     change. Similarly this reactor will receive goals posted by other
     *     reactors that declared this timline as @e External
     *
     * A reactor provide also informations about its latency and look-ahead which
     * indicates its planning horizon and the maximum time it is exx1pected to
     * deliberate in order to produce a plan for the goals it received.
     
     * All these informations are used by the agent in order to schedule the
     * communication between agents along with their deliberation.
     *
     * @author Conor McGann @& Frederic Py <fpy@mbari.org>
     * @ingroup transaction
     * @sa class graph
     */
    class TeleoReactor :boost::noncopyable {
      typedef details::timeline_set                       internal_set;
      typedef details::external_set                       external_set;
      
    public:
      typedef utils::cpu_clock stat_clock;
      
# if defined(CPP11_HAS_CHRONO)
      typedef CHRONO::high_resolution_clock rt_clock;
# else
      typedef CHRONO::steady_clock     rt_clock;
# endif // CPP11_HAS_CHRONO      
      
      typedef stat_clock::duration stat_duration;

      
      static utils::Symbol const obs;
      static utils::Symbol const plan;
      
      typedef graph::duration_type duration_type;
      typedef graph::date_type     date_type;
      /** @brief Reactor reference type
       *
       * The type used to refer to specific reactor
       */
      typedef graph::reactor_id                   ref_type;
      /** @brief XML based factory
       *
       * The type of the factory used to generate reactors from XML
       */
      typedef graph::xml_factory                  xml_factory;
      /** @brief XML construction info
       *
       * The argument passed by xml_factory while calling reactor classes
       * constructor. This type is not a simple rapidxml::xml_node<> but also
       * embeds a reference to the graph that the newly created instance will
       * be attached to.
       *
       * @sa TREX::utils::XmlFactory
       * @sa TREX::utils::XmlFactory::arg_traits;
       * @sa TREX::transaction::graph
       */
      typedef xml_factory::argument_type          xml_arg_type;
      
      /** @brief XML based constructor
       *
       *
       * @param[in] arg A structure embedding the xml structure and the graph that
       *                called this constructor
       * @param[in] loadTL A flag to allow/prohibit the parsing of Internal
       *                  and External relations gfrom the xml structure
       * @param[in] log_default A flag to indicate what is the default logging
       *                        behavior for this reactor
       *
       * This constructor creates a new reactor by parsing the xml structure @p arg.
       * Ig loadTL is true it will also extract Internal and External timelines
       * definitions from this structure. @p log_default indicates what is the
       * default logging behaivor of the reactor when the @c log attribute is not
       * given in the XML structure.
       *
       * A typical reactor  definition is as follow
       * @code
       * < <RType> name="<name>" lookahead="<lookahead>" latency="<latency>"
       *           config="<config>" log="<logflag>" verbose="<verbflag>" >
       *      <External name="<ename>" goals="<post goal flag>" />
       *      <Internal name="<iname>" />
       * </ <RType> >
       * @endcode
       *
       * With :
       * @li @c @<RType@> being the type of the reactor as declared to the factory
       * @li @c @<name@>  the name of the reactor
       * @li @c @<lookahead@> the reactor's look-ahead
       * @li @c @<latency@> the reactors's latency
       * @li @c @<logflag@> An optional flag used to indicate that observations and commands
       *                   issued from this reactor should be logged or not
       *                   (default is @p log_default)
       * @li @c @<config@> An optional extra file that extends the defintions
       *                    of this tag
       * @li @c @<verbflag@> An optional flag to indicates wheether this reactor
       *                     should be verbose in TREX.log or not. Defulat is
       *                     graph::is_verbose()
       *
       * If @p loadTL is true then that class will also parse the External
       *              and Internal tags in order to declare internal and
       *              external timelines of this reactor.
       *
       * @throw TREX::utils::XmlError An error occured while extracting reactors
       *        information from the XML structure
       */
      explicit TeleoReactor(xml_arg_type &arg, bool loadTL=true,
                            bool log_default=true);
      /** @brief Destructor */
      virtual ~TeleoReactor();
      
      /** @brief Reactor name
       * @return the anme of the reactor
       */
      TREX::utils::Symbol const &getName() const {
        return m_name;
      }
      /** @brief get \"agent\" name
       *
       * Gets the name of the agent that controls this reactor.
       *
       * @deprecated reactors are now manipulated by a graph.
       *             The agent uses this graph to trigger reactor's
       *             callbacks based on graph traversal algorithms.
       *             A graph is a more general concept thatn an agent
       *             (for example a graph could allow to have cyclic
       *             dependencies between reactors ...) to reflect this
       *             change this method has been replaced by getGraphName()
       *             and kept for backward compatibility.
       *
       * @return the name of the "agent"
       * @sa getGraphName() const
       */
      TREX::utils::Symbol const &getAgentName() const {
        return getGraphName();
      }
      /** @brief Get graph name
       *
       * @return the name of the graph that maintins this reactor relations
       *
       * @sa class TREX::transaction::graph
       */
      TREX::utils::Symbol const &getGraphName() const {
        return m_graph.getName();
      }
      /** @brief Get graph
       *
       * This methods gives a reference to the graph that manage this reactor
       *
       * @return the graph
       * @sa getGraphName() const
       */
      graph const &getGraph() const {
        return m_graph;
      }
      
      
      /** @brief reactor initial tick
       *
       * This method indicates the tick during which the reactor has been
       * initialized.
       *
       * @pre the reactor has bein initialized
       *
       * @return the tick during which the reactor got started
       *
       * @sa handleInit()
       * @sa getCurrentTick() const
       * @sa getFinalTick() const
       */
      TICK getInitialTick() const {
        return m_initialTick;
      }
      /** @brief Tick duration
       *
       * Indicates the duration of the tick. This duration is expected to
       * be expressed in seconds even though nothing in the architecture
       * constraint such aspect.
       *
       * @return the duration of a single tick
       */
      duration_type tickDuration() const {
        return m_graph.tickDuration();
      }
      
      /** @brief unix time to tick conversion
       * @param[in] secs Number of seconds
       * @param[in] usecs Number of microseconds
       *
       * @return equivalent tick to the time @p secs.usecs
       *
       * This method is an utility to ease the conversion form unix time to tick
       * it calls the agent clock corresponding method to convert the specified
       * unix time into a tick value
       *
       * @note the result of the method depends on the clock implementation.
       * For example the RealTimeClock fully support it while the StepClock
       * will simply assume that unixtime and tick are perfeclty aligned
       * (as if the mission started on January 1st 1970)
       */
      TICK timeToTick(date_type const &date) const {
        return m_graph.timeToTick(date);
      }
      /** @brief tick to unix time conversion
       * @param[in] cur A tick value
       *
       * @return equivalent unix time to @p cur as a float
       *
       * This method is an utility to ease the conversion form a tick to unix time
       * it calls the agent clock corresponding method to convert the specified
       * tick value into unix time
       *
       * @note the result of the method depends on the clock implementation.
       * For example the RealTimeClock fully support it while the StepClock
       * will simply assume that unixtime and tick are perfeclty aligned
       * (as if the mission started on January 1st 1970)
       */
      date_type tickToTime(TICK cur) const {
        return m_graph.tickToTime(cur);
      }
      std::string date_str(TICK cur) const {
        return m_graph.date_str(cur);
      }
      std::string duration_str(TICK cur) const {
        return m_graph.duration_str(cur);
      }
      
      /** @brief Get current tick
       *
       * @return current tick date
       *
       * @sa getInitialTick() const
       * @sa getFinalTick()
       * @sa graph::getInitialTick() const
       */
      TICK getCurrentTick() const {
        return m_graph.getCurrentTick();
      }
      /** @brief Get final tick
       *
       * Identify the agent final tick. This value correspond to the maximum
       * possible life-time of the agent.
       *
       * @ pre the reactor is initialized
       *
       * @return mission final tick
       *
       * @sa initialize()
       * @sa getInitialTick() const
       * @sa getCurrentTick() const
       */
      TICK getFinalTick() const; 
      /** @brief reactor latency
       *
       * Indicates the reactor @p deliberation latency. Thids vlaue reflects
       * the exstimated maximum delay this reactor needs to produce a new plan
       * and is usually provided by the user during agent definition
       *
       * @return the reactor's latency
       *
       * @sa getExecLatency() const
       */
      TICK getLatency() const {
        return m_latency;
      }
      /** @brief reactor execution latency
       *
       * Indicates the reactor @p execution latency. This value correspond to
       * the delay it take s for thsi reactor to start executing its plan. It
       * is function of the execution latency of the reactors it depends on and
       * is updated automatically by the reactors graph.
       *
       * This value is used by TREX and represent this reactor's dispatch
       * window lowe bound.
       *
       * @return the reactor's execution latency
       *
       * @sa getLatency() const
       */
      TICK getExecLatency() const {
        return m_latency+m_maxDelay;
      }
      /** @brief Get reactor look-ahead
       *
       * Indicates how many ticks the reactors look ahead while planning.
       * A value of 0 indicates that this reactor does not accept goal.
       *
       * @return the reactor look-ahead
       */
      TICK getLookAhead() const {
        return m_lookahead;
      }
      /** @btrief New observation callback
       *
       * @param[in] obs An observation
       *
       * This method is called by the agent when one of the @p External
       * timelines of this reactor changed state. The state is described by
       * @p obs and satrts from the current tick.
       *
       * It is guartanteed that all the notification of the current tick will be
       * notified before the reactor synchronization
       * @warning the duration of this call should  be neglectable in comparison to
       *          the tick duration.
       * @sa synchronize()
       */
      virtual void notify(Observation const &obs) {}
      
      /** @brief Final initialization
       *
       * @param[in] final   The final tick date
       *
       * @pre @c getCurrentTick() @< @p final
       * @pre the reactor has not been inited yet
       *
       * This method is called by the agent to allow the reactor to complete its
       * initialization. At this stage the reactor knows that it is part of the
       * reactor graph and can complete its initialization if needed. During this
       * call, the callback handleInit will be executed allowing to execute reactors
       * specific initializations
       *
       * @throw ReactorException Attempted to call this method twice
       *
       * @post The reactor is ready to be executed in the agent
       *
       * @sa handleInit()
       * @sa getInitialTick() const
       * @sa getCurrentTick() const
       */
      bool initialize(TICK final);
      
      /** @brief New tick notification callback
       *
       * This method is called by the agent to notify the reactor
       * that a new tick has started. It will call the handleTickStart()
       * method
       *
       * @sa handleTickStart()
       */
      bool newTick();
      /** @brief Synchronization callback
       *
       * This method is called by the agent to make the reactor execute
       * its synchronization. It will call synchronize callback and returns
       * then status of the reactor according to the execution of this call.
       *
       * @retval true synchronization successfull
       * @retval false the call to synchronize returned @c false or thew
       *                an exception
       *
       * @warning when this method returns @c false  this reflect a @e critical
       *          reactor failure. Therefre the agent will kill this reactor and
       *          produce the observation @c Failed on each of its @e Internal
       *          timeline.
       * @sa synchronize()
       */
      bool   doSynchronize();
      /** @brief reactor deliberation work load
       *
       * This method calls hasWork method and, depending on the returned value
       * and how close the reactor is to reach is latency, will give a value
       * indicating heurisitically how urgent it is for him to delinerate.
       *
       * @return a value that becomes larger as the reactor is approaching its
       *         latency deadline or @c NAN if hasWork() returned @c false
       *
       * @sa hasWork()
       * @sa step()
       */
      double workRatio();
      /** @brief Deliberation step
       *
       * This method is called by the agent when it allows the reactor to
       * do one deliberation step. It calls resume() and update informations
       * in order to compute the reactors next work ratio
       *
       * @sa workRatio()
       * @sa resume()
       */
      void   step();
      
      /** @brief Iterator other external timelines
       *
       * The type used to iterate other the external timelines of a reactor.
       * This is mostly used by the boost graph library.
       */
      typedef details::external external_iterator;
      /** @brief size type
       *
       * The type used to return a size such the number of @e External timelines
       * of a reactor
       */
      typedef size_t            size_type;
      
      /** @brief Number of @e External timelines
       *
       * @return the number of @e External timelines on this reactor
       */
      size_type         count_externals() const {
        return m_externals.size();
      }
      /** @brief beginning of @e External timelines set
       *
       * @return An iterator pointing to the beginning of the @e External timelines
       * set for this reactor
       * @sa ext_end()
       */
      external_iterator ext_begin();
      /** @brief end of @e External timelines set
       *
       * @return An iterator pointing to the end of the @e External timelines
       * set for this reactor
       * @sa ext_begin()
       */
      external_iterator ext_end();
      
      typedef details::relation_iter internal_iterator;
      
      /** @brief Count the number of internal timelines
       *
       * This methods indicates the number of internal timelines this reactor
       * currently owns
       *
       * @return the number of internal timelines of this reactor
       *
       * @sa count_internal_relations() const
       */
      size_type     count_internals() const {
        return m_internals.size();
      }
      /** @brief Count the number of internal relations
       *
       * This method count the number of relations that this reactor have
       * with other reactors issued from its internal timelines.
       *
       * This number does not count the number of internal timelines but the
       * sum of all the reactors that are connected to this reactor through its internal
       * timelines
       */
      size_type     count_internal_relations() const;
      
      /** @brief First internal relation iterator
       *
       * @sa count_internal_relations() const
       * @sa int_end() const
       */
      internal_iterator int_begin() const {
        return internal_iterator(m_internals.begin(), m_internals.end());
      }
      /** @brief Last internal relation iterator
       *
       * @sa count_internal_relations() const
       * @sa int_begin() const
       */
      internal_iterator int_end() const {
        return internal_iterator(m_internals.end(), m_internals.end());
      }
      
    protected:
      /** @brief Constructor
       *
       * @param[in] owner The graph that creates and will own this instance
       * @param[in] name Name of the reactor
       * @param[in] latency Deliberation latency for this reactor
       * @param[in] lookahead of his reactor
       * @param[in] log A log flag
       *
       * Create a new reactor named @p name with a latency of @p latency and a
       * look-ahead of @p lookahead and associate it to the graph @p owner
       */
      TeleoReactor(graph *owner, TREX::utils::Symbol const &name,
                   TICK latency, TICK lookahead, bool log=false);
      
      /** @brief LogManager access point
       *
       * @return the LogManager instance for this run
       */
      TREX::utils::LogManager &manager() const {
        return m_graph.manager();
      }
      typedef utils::LogManager::path_type path_type;
      
      path_type file_name(std::string const &short_name) const {
        return manager().file_name(getName().str()+"/"+short_name);
      }
      
      /** @brief Check for verbosity level
       *
       * Checks if this reactor is set as verbose or not.
       * As for now a non verbose reactor will not display its
       * observations in TREX.log if it is logging them.
       *
       * @retval true if the reactor is verbose
       * @retval false otherwise
       * @sa set_verbose(bool)
       * @sa reset_verbose()
       */
      bool is_verbose() const {
        return m_verbose;
      }
      /** @brief set verbosity level
       * @param[in] falg verbosity flag
       *
       * Sets verbosity level of this reactor
       * @sa is_verbose() const
       * @sa reset_verbose()
       */
      void set_verbose(bool flag=true) {
        m_verbose = flag;
      }
      /** @brief reset verbosity level
       * Set verbosity leve lof this reactor to its graph verbosity level
       *
       * @sa is_verbose() const
       * @sa set_verbose(bool)
       * @sa graph::is_verbose() const
       */
      void reset_verbose() {
        m_verbose = m_graph.is_verbose();
      }
      
      /** @brief Check if need to deliberate
       *
       * This method is called during workRatio in order to idenitfy if the reactor
       * needs to deliberate.
       *
       * By default it always return 0 and need to be redefined by derived classes if
       * they need to deliberate.
       *
       * @retval true this instance needs to deliberate
       * @retval false this instance does not require deliberation
       *
       * If the methods returns true this will eventually result on the agent calling
       * the step() method
       *
       * @sa workRatio()
       * @sa step()
       */
      virtual bool hasWork() {
        return false;
      }
      
      
      /** @brief Produce an observation
       *
       * @param[in] o An observation
       *
       * This method is a falcility to produce internally an observation @p o on one of
       * the reactors @e Internal timeline.
       *
       * It can be called internally to the reactor when a new state has been identified
       * for this timeline.
       *
       * @pre @c o.object() is internal to this reactor
       * @post Unless postObservation is called in between with an observation on the
       *       same object, the observation @p o will be dispatched to all the
       *       clients of @c o.object() during the next doNotify()
       *
       * @throw SynchronizationError attempt to post an observation in a timeline
       *        which is not @e Internal to this reactor.
       * @sa class Observation
       * @sa isInternal(TREX::utils::Symbol const &) const
       * @sa doNotify()
       */
      void postObservation(Observation const &o, bool verbose=false);
      
      /** @brief Post a goal
       *
       * @param[in] g A goal id
       *
       * Post the goal @p g to an external timeline
       *
       * @pre g is a valid goal_id (i.e. not NULL)
       * @pre g->object() is an @e External timeline of this reactor
       *
       * @throw DispatchError The goal was not valid or was not on an external
       *        timeline of this reactor
       *
       * @return true the goal has been succesfully posted and is queued for
       *              dispatching
       * @return false this goal id is already in the dispatching queue
       *
       * @note sucessfully posting a goal do not gauarantee that this goal
       *       will be duispatched and even less executed. It can be already
       *       too late to dispatch such a goal or the goal is simply impossible
       *       to be executed in current situation. As of today, the only way to
       *       check a goal has been executed is to wait for the corresponding
       *       observation unitl the maximum start time has been passed.
       *
       * @sa isExternal(TREX::utils::Symbol const &) const
       * @sa postRecall(goal_id const &)
       */
      bool postGoal(goal_id const &g);
      /** @brief Post a goal
       *
       * @param[in] g A goal
       *
       * This method is just an overload of postGoal(goal_d const &) for user facility
       *
       * @pre @c g.object() is an @e External timeline
       *
       * @throw DispatchError the goal wasn't on an @e External timeline
       *
       * @return the goal_id for this goal
       *
       * @sa postGoal(goal_id const &)
       * @sa isExternal(TREX::utils::Symbol const &) const
       */
      goal_id postGoal(Goal const &g);
      
      goal_id postGoal(boost::property_tree::ptree::value_type const &g) {
        goal_id gl=parse_goal(g);
        if( postGoal(gl) )
          return gl;
        return goal_id();
      }
      
      goal_id parse_goal(boost::property_tree::ptree::value_type const &g);
      
      /** @brief Goal completion
       *
       * @param[in] g A goal
       *
       * This method can be used from the reactor to inform the goal
       * management system that the goal @p g on one of its external
       * timeline has completed
       *
       * @deprecated This method should become useless as the goal management
       *             system is much more able to identify that a goal did
       *             complete
       */
      bool completedGoal(goal_id g);
      
      /** @brief Post a planned token
       *
       * @param[in] g A goal id
       *
       * Inform that the token @p g is now part of the plan of the reactor
       *
       * @pre g is a valid goal_id (i.e. not NULL)
       * @pre g->object() is an @e Internal timeline of this reactor
       *
       * @throw DispatchError The token was not valid or was not on an internal
       *        timeline of this reactor
       *
       * @return true the token has been correctly integrated.
       * @return false this token id is already broadcasted or its end time is
       *         in the past
       *
       * @sa isInternal(TREX::utils::Symbol const &) const
       * @sa cancelPlanToken(goal_id const &)
       */
      bool postPlanToken(goal_id const &g);
      /** @brief Post a planned token
       *
       * @param[in] g A goal
       *
       * This method is just an overload of postPlanToken(goal_d const &) for user facility
       *
       * @pre g.object() is an @e Internal timeline of this reactor
       *
       * @throw DispatchError The token was not on an internal
       *        timeline of this reactor
       *
       * @returnthe goal_id for the token posted. This id is invalid if the token was
       *           not posted
       *
       * @sa isInternal(TREX::utils::Symbol const &) const
       * @sa postPlanToken(goal_id const &)
       */
      goal_id postPlanToken(Goal const &g);
      void cancelPlanToken(goal_id const &g);
      
      
      /** @brief Recall a goal
       *
       * @param[in] g A goal
       *
       * This method recall the goal with the goal_id of @p g
       *
       * @retval true the goal was sucessfully recalled
       * @retval false the goal was not valid or is not on an @e External timeline
       *
       * @note The goal_id g should be @e execatly the one passed to postGoal (or returned
       *       by it).
       * @note As for postGoal sucessfully recalling a goal does not mean that it will
       *       be removed from the plan of the owner of this timeline. It is more a way
       *       to indicate that this goal is not required anymore.
       *
       * @sa postGoal(goal_id const &)
       * @sa postGoal(Goal const &)
       * @sa isExternal(TREX::utils::Symbol const &) const
       */
      bool postRecall(goal_id const &g);
      
      /** @brief reactor initialiszation callback
       *
       * This callback is called during initialize() is order to allow derived
       * classes to complete their initialization before the reactor goat executed.
       *
       * It is called once at the beginning of the reactor ewxecution.
       *
       * @bug this function is blocking and while it is usuaally called before the clock
       *      of the agent got started if the reactor is created at the beginning of the
       *      mission. it won't be the case for a reactor created on the fly which may be
       *      problematic if the calls takes too long in comparison of the tick duration.
       */
      virtual void handleInit() {}
      /** @brief New tick callback
       *
       * This call back is executed at the beginning of a new tick. and can be used
       * for the reactor to prepare for synchronization. It is also often used by the
       * reactor to post its new goals.
       *
       * @note at this stage the reactor should have received all the new goals for
       *       this tick. but this is not guaranteed.
       * @warning the duration of this call should  be neglectable in comparison to
       *          the tick duration.
       */
      virtual void handleTickStart() {}
      
      /** @brief Synchronization callback
       *
       * This method is called in order to ask for the reactor to
       * @li resolve its current state
       * @li produce new observation on its exeternal timelines if their state
       *     have changed
       * The agent ensures that all new observation on this reactor timelines have
       * been produced -- and received through notify -- to this reactor.
       *
       * This is one of the more critical callback of the reactor and when it fails
       * is  considereed as a critical error resulting on the resactor being killed
       * by the agent
       *
       * @retval true synchronization succeeded
       * @retval false synchronization failure. Will result on the reactor being
       * killed soon after
       * @warning the duration of this call should  be neglectable in comparison to
       *          the tick duration.
       */
      virtual bool synchronize() =0;
      
      /** @brief Deliberation step callback
       *
       * This callback will be executed in response of hasWork returning @c true. It
       * allows the reactor to execute an atomic deliberation step.
       *
       * @warning the duration of this call should  be neglectable in comparison to
       *          the tick duration.
       */
      virtual void resume() {}
      
      /** @brief Goal reception callback
       * @param[in] g a goal
       *
       * This method is called by the agent in roder to notify this reactor that the
       * goal @p g has been requested to him.
       *
       * @warning the duration of this call should  be neglectable in comparison to
       *          the tick duration.
       */
      virtual void handleRequest(goal_id const &g) {}
      /** @brief Goal recall callback
       * @param[in] g a goal
       *
       * This method is called by the agent in roder to notify this reactor that the
       * goal @p g is no logner desired.
       *
       * @warning the duration of this call should  be neglectable in comparison to
       *          the tick duration.
       */
      virtual void handleRecall(goal_id const &g) {}
      
      virtual void newPlanToken(goal_id const &t) {}
      virtual void cancelledPlanToken(goal_id const &t) {}
      
      /** @brief External timeline declaration
       *
       * @param[in] timeline a name
       * @param[in] control  goal dispatching flag
       * @param[in] plan_listen plan listening flag
       *
       *
       * This method is used to declare @p timeline as an @e External timeline.
       * The optional @p control flag prohibits goal dispatching to this timeline when
       * @c false. The @p plan_listen flag aloow the reactor to be notified whenever the
       * owner of this timeline does give the current tokens it has in his future plan
       *
       * @pre The timeline is not already @e Internal
       *
       * @warning The type of a timeline can change during reactor execution and this
       * is the reason why this methods give no feedback. It is always recommend to
       * check the success/failure of this operation using isExternal
       *
       * @sa isExternal(TREX::utils::Symbol const &) const
       * @sa isInternal(TREX::utils::Symbol const &) const
       * @sa graph::subscribe(graph::reactor_id, TREX::utils::Symbol const &, details::transaction_flag const &)
       * @sa provide(TREX::utils::Symbol const &)
       * @sa unuse(TREX::utils::Symbol const &)
       */
      void use(TREX::utils::Symbol const &timeline, bool control=true, bool plan_listen=false);
      
      /** @brief Internal timeline declaration
       *
       * @param[in] timeline a name
       * @param[in] controlable does this timeline accept goals
       * @param[in] publish does this timeline will publish its plan
       *
       * This method is used to declare @p timeline as an @e Internal timeline.
       *
       * @warning The type of a timeline can change during reactor execution and this
       * is the reason why this methods give no feedback. It is always recommend to
       * check the success/failure of this operation using isInternal
       *
       * @note If the timeline was @p External and is not owned yet by any reactor, this
       *       call will promote it as @p Internal
       *
       * @sa isExternal(TREX::utils::Symbol const &) const
       * @sa isInternal(TREX::utils::Symbol const &) const
       * @sa graph::assign(graph::reactor_id, TREX::utils::Symbol const &)
       * @sa use(TREX::utils::Symbol const &, bool)
       * @sa unprovide(TREX::utils::Symbol const &)
       */
      void provide(TREX::utils::Symbol const &timeline, bool controllable=true, bool publish=false);
      
      /** Add comment on transaction log
       * @param[in] msg A message
       *
       * Adds the message @p msg in the transaction log. This message will be
       * produced only if the reactor is logging its transactions.
       */
      void tr_info(std::string const &msg);
      
      /** @brief External timeline unsubscription
       *
       * @param[in] timeline A timeline name
       *
       * Remove the External declaration of @p timeline from this reactor
       *
       * @retval true operation succeeded
       * @retval false @p timeline was probably not External to this reactor
       *
       * @post @p timeline is not External to this reactor
       *
       * @sa isExternal(TREX::utils::Symbol const &) const
       * @sa use(TREX::utils::Symbol const &, bool)
       * @sa unprovide(TREX::utils::Symbol const &)
       * @sa isolate(bool)
       */
      bool unuse(TREX::utils::Symbol const &timeline);
      /** @brief Internal timeline unsubscription
       *
       * @param[in] timeline A timeline name
       *
       * Remove the Internal declaration of @p timeline from this reactor
       *
       * @retval true operation succeeded
       * @retval false @p timeline was probably not Internal to this reactor
       *
       * @post @p timeline is not Internal to this reactor
       *
       * @sa isInternal(TREX::utils::Symbol const &) const
       * @sa unuse(TREX::utils::Symbol const &)
       * @sa provide(TREX::utils::Symbol const &, bool, bool)
       * @sa isolate(bool)
       */
      bool unprovide(TREX::utils::Symbol const &timeline);
      
      /** @brief Request for external failed
       *
       * @param[in] timeline Symbolic name of the timeline
       * @param[in] err Descriptor of the error
       *
       * This callback is executed when an attempt to use the External timeline
       * @p timeline triggered an error. The reason of the failure is described by
       * @p err.
       * It can be derived in order to provide an alternate way to handle this failed
       * request
       *
       * @sa failed_internal(TREX::utils::symbol const &, graph::timeline_failure const &)
       */
      virtual bool failed_external(TREX::utils::Symbol const &timeline,
                                   graph::timeline_failure const &err) {
        throw err;
      }
      /** @brief Request for internal failed
       *
       * @param[in] timeline Symbolic name of the timeline
       * @param[in] err Descriptor of the error
       *
       * This callback is executed when an attempt to prtovide the Internal timeline
       * @p timeline triggered an error. The reason of the failure is described by
       * @p err.
       * It can be derived in order to provide an alternate way to handle this failed
       * request
       *
       * @sa failed_external(TREX::utils::symbol const &, graph::timeline_failure const &)
       */
      virtual bool failed_internal(TREX::utils::Symbol const &timeline,
                                   graph::timeline_failure const &err) {
        throw err;
      }
      /** @brief Check for internal timeline
       *
       * @param[in] timeline A name
       *
       * @retval true if @p timeline is @e Internal to this reactor
       * @retval false otherwise
       *
       * @sa isExternal(TREX::utils::Symbol const &) const
       * @sa provide(TREX::utils::Symbol const &)
       */
      bool isInternal(TREX::utils::Symbol const &timeline) const;
      /** @brief Check for exeternal timeline
       *
       * @param[in] timeline A name
       *
       * @retval true if @p timeline is @e External to this reactor
       * @retval false otherwise
       *
       * @sa isInternal(TREX::utils::Symbol const &) const
       * @sa use(TREX::utils::Symbol const &, bool)
       */
      bool isExternal(TREX::utils::Symbol const &timeline) const;
      
      /** @brief new log entry
       *
       * @param[in] context log entry prefix
       *
       * create a new entry in TREX.log
       *
       * @return a stream that can be used to qwrite in the entry
       *
       * @sa TREX::utils::LogManager
       * @sa TREX::utils::TextLog
       */
      utils::log::stream
      syslog(utils::log::id_type const &context, utils::log::id_type const &kind) const {
        if( context.empty() )
          return m_graph.syslog(getName(), kind);
        else
          return m_graph.syslog(getName().str()+"."+context.str(), kind);
      }
      utils::log::stream
      syslog(utils::log::id_type const &kind=null) const {
        return m_graph.syslog(getName(), kind);
      }
      
      /** @brief Find an external timeline
       *
       * @param[in] name hte name of the timeline
       *
       * @return An iterator refering to the @e external timeline @p name or
       *        @c ext_end() if this reactor does not subscribe to @p name
       */
      external_iterator find_external(TREX::utils::Symbol const &name);
      
    protected:
      graph &get_graph() {
        return m_graph;
      }
      
      void setMaxTick(TICK max); 
      TICK update_latency(TICK new_latency);
      TICK update_horizon(TICK new_horizon);

    private:
      stat_duration m_start_usage, m_synch_usage, m_deliberation_usage;
      rt_clock::duration m_start_rt, m_synch_rt, m_delib_rt;
      
      
      bool internal_sync(TREX::utils::Symbol name) const;
      bool external_sync(TREX::utils::Symbol name) const;
      
      void clear_int_sync();
      void clear_ext_sync();
      
      void use_sync(TREX::utils::Symbol name, details::transaction_flags f);
      bool unuse_sync(TREX::utils::Symbol name);
  
      void provide_sync(TREX::utils::Symbol name, details::transaction_flags f);
      bool unprovide_sync(TREX::utils::Symbol name);
     
      void observation_sync(Observation o, bool verbose);
      bool goal_sync(goal_id g);
      bool recall_sync(goal_id g);
      
      bool plan_sync(goal_id tok);
      void cancel_sync(goal_id tok);
      
      void clear_internals();
      void clear_externals();
      
      bool m_inited;
      bool m_firstTick;
      graph &m_graph;
      
      class Logger;
    
      void queue(std::list<goal_id> &l, goal_id g);
      
      void queue_goal(goal_id g);
      void queue_recall(goal_id g);
      void queue_token(goal_id g);
      void queue_cancel(goal_id g);
      
      
      void goal_flush(std::list<goal_id> &a, std::list<goal_id> &dest);
      
      std::list<goal_id>     m_sync_goals, m_sync_recalls, m_sync_toks, m_sync_cancels;
      utils::SharedVar<size_t> m_have_goals;
      
      bool have_goals();
      
      void collect_obs_sync(std::list<Observation> &l);
      
      /** @brief Request new observations
       *
       * This method is called at the beginnin of synchronizayion in order
       * for the reactor to collect its @e External observations
       * This function identifies all the @e External timelines for which
       * a postObservation has been called and will collect the last
       * obserbvation produced for this tick if it exist.
       *
       * @sa doSynchronize()
       * @sa postObservation()
       * @sa notify(Obserbvation const &)
       */
      void   doNotify();
      
      bool m_verbose;
      
      /** @brief Transaction logger
       *
       * A pointer to the transaction logger for this reactor. If the pointer
       * is not @c NULL. The this reactor will log all the transaction events
       * it produces during its lifetime in an output file named
       * <reactor>.tr.log.
       * Such file can be used by a TransactionPlayer to reproduce this reactor
       * behavior inside the agent.
       *
       * @sa Logger
       * @sa TransactionPlayer
       */
      Logger *m_trLog;
      
      /** @brief Reactor name
       *
       * This is the name of the reactor. It is used to idenitify the
       * reactor in the agent and for displaying log messages.
       *
       * @sa getName() const
       */
      TREX::utils::Symbol m_name;
      
      /** @brief Deliberation latency
       *
       * Identifies the maximum amount of time it takes for the reactor
       * to produce a plan.
       *
       * @note This value is expressed in tick while the number of steps would
       * make much more sense (as we do not really know how much steps are in a tick)
       */
      TICK m_latency;
      TICK m_maxDelay;
      /** @brief Planning look-ahead
       *
       * How much ticks ahead the reactor is looking while deliberating
       */
      TICK m_lookahead;
      
      /** @brief Initial tick value
       *
       * The tick date when this reactor started to execute (typically 0)
       */
      TICK   m_initialTick, m_obsTick;
      /** @brief Final tick value
       *
       * The tick date when the reactor will end. THis value often reflects
       * the same value as the agent end time
       */
      boost::optional<TICK>   m_finalTick;
      
      
      void reset_deadline();
      /** @brief Deliberation deadline
       *
       * This value indicates when this reactor is expected to complete
       * its current deliberation. This deadline is computed every tick
       * m_nSteps is equal to 0. The value is then set to the current tick plus
       * the latency of this reactor.
       *
       * This value is then used by workRatio in order to compute the reactor
       * priority level based on how close this reactor is close to  its deadline
       *
       * @sa workRatio()
       * @sa hasWork()
       * @sa m_nSteps
       */
      mutable TICK m_deadline;
      /** @brief Number of successive deliberation steps
       *
       * Count the number of steps of deliberation this reactor did execute
       * without declaring that it was done deliberating. As soon as
       * hasWork() return false this counter will be reset to 0
       *
       * This value is then used by workRatio in order to compute the reactor
       * priority level based on how close this reactor is close to  its deadline
       *
       * @sa workRatio()
       * @sa hasWork()
       * @sa m_deadline
       */
      mutable unsigned long m_nSteps;
      mutable unsigned long m_tick_steps;
      mutable bool m_past_deadline;
      mutable unsigned long m_validSteps;
      
      external_set m_externals;
      internal_set m_internals;
      internal_set m_updates;
      
      /** @brief TREX log entry point
       *
       * Used to all the configuration and logging management for this reactor
       *
       * @sa syslog() const
       */
      TREX::utils::SingletonUse<TREX::utils::LogManager> m_log;

      utils::async_ofstream m_stat_log;

      
      void isolate(bool failed=true);
      
      // call-backs from timeline
      void assigned(details::timeline *tl);
      void unassigned(details::timeline *tl);
      void subscribed(Relation const &r);
      void unsubscribed(Relation const &r);
      void latency_updated(TICK old_l, TICK new_l);
      void unblock(utils::Symbol const &name);
      
      
      friend class details::timeline;
      friend class details::external;
      friend class ReactorException;
      friend class Relation;
      
      friend class graph;
    }; // TREX::transaction::TeleoReactor
    
    /** @brief Synchronization related exception
     *
     * This exception will be throwned when an operation related to
     * synchronization failed.
     *
     * @author Frederic Py <fpy@mbari.org>
     * @relates class TeleoReactor
     * @ingroup transaction
     */
    class SynchronizationError :public ReactorException {
    public:
      /** @brief Constructor
       *
       * @param[in] r The reactor where the error did happen
       * @param[in] msg The error message
       */
      SynchronizationError(TeleoReactor const &r, std::string const &msg) throw()
      :ReactorException(r, msg) {}
      /** @brief Desturctor */
      ~SynchronizationError() throw() {}
    }; // TREX::transaction::SynchronizationError
    
    /** @brief Goal posting error
     *
     * This execption will be thrown when a goal posting was invalid
     *
     * @author Frederic Py <fpy@mbari.org>
     * @relates class TeleoReactor
     * @ingroup transaction
     */
    class DispatchError :public ReactorException {
    public:
      /** brief Constructor
       * @param[in] r The reactor where the error did occur
       * @param[in] g The goal that triggered the error
       * @param[in] msg The error msg
       */
      DispatchError(TeleoReactor const &r, goal_id const &g, std::string const &msg) throw()
      :ReactorException(r, build_msg(g, msg)) {}
      /** @brief Destructor */
      ~DispatchError() throw() {}
      
    private:
      /** @brief Construct the error message
       * @param[in] g   A goal
       * @param[in] msg An error message
       * 
       * @return A string that depicts that the error @p msg did occur with the goal @p g
       */
      static std::string build_msg(goal_id const &g, std::string const &msg) throw();
    };
    
    
  } // TREX::transaction
} // TREX

# include "bits/bgl_support.hh"
# include "bits/reactor_graph.tcc"

#endif
