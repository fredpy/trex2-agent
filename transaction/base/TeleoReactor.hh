/** @file "TeleoReactor.hh"
 * @brief Declares the main interface for a Teleo-Reactor
 *
 * Defines the basis to implement a Teleo-reactor
 * 
 * @author Frederic Py <fpy@mbari.org> (initial version from Conor McGann)
 * @ingroup transaction
 */
#ifndef H_TeleoReactor
# define H_TeleoReactor

# include <cmath>

# include "bits/external.hh"
# include "reactor_graph.hh"

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
     * indicates its planning horizon and the maximum time it is expected to
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
      typedef graph::reactor_id                   ref_type;
      typedef graph::xml_factory                  xml_factory;
      typedef xml_factory::argument_type          xml_arg_type;

      /** @brief XML based constructor
       *
       *
       * @param[in] arg A structure embedding the xml structure and the graph that
       *                called this constructor
       * @param[in] loadTL A flag to allow/prohibit the parsing of Internal
       *                  and External relations gfrom the xml structure
       *
       * This constructor creates a new reactor by parsing the xml structure @p arg.
       * Ig loadTL is true it will also extract Internal and External timelines
       * definitions from this structure.
       *
       * A typical reactor  definition is as follow
       * @code
       * < <RType> name="<name>" lookahead="<lookahead>" latency="<latency>"
       *           config="<config>" log="<logflag>" >
       *      <External name="<ename>" goals="<post goal flag>" />
       *      <Internal name="<iname>" />
       * </ <RType> >
       * @endcode
       *
       * With :
       * @li @c <RType> being the type of the reactor as declared to the factory
       * @li @c <name>  the name of the reactor
       * @li @c <lookahead> the reactor's look-ahead
       * @li @c <latency> the reactors's latency
       * @li @c <logflag> a flag use to indicate that observations and commands issued
       *                  from this reactor should be logged or not
       * @li @c <config> An optional extra file that extends the defintions of this tag
       * 
       * If @p loadTL is true then tha class will also parse the External and Internal
       * tags in roder to declare internal and external timelines of this reactor.
       *
       * @throw XmlError An error occured while extracting reactors information from
       *        the XML structure
       */
      explicit TeleoReactor(xml_arg_type &arg, bool loadTL=true);  
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
      TICK getLookAhead() const {
	return m_lookahead;
      }
      virtual void notify(Observation const &obs) {}

      /** @brief Final initialization
       *
       * @param[in] final   The final tick date
       *
       * @pre @c getCurrentTick() @< @p final
       * @pre the reactor has not been inited yet
       *
       * This method is called by the agent to allow the reactor to complete its
       * initialization. At this stage the treactor knwos that it is part of the
       * reactor graph and can complete its initialisation if needed. During this
       * call, the callback handleInit will be executed allowing to execute reactors
       * spacific initializations
       *
       * @throw ReactorException Attempted to call this method twice
       *
       * @post The reactor is ready to be executed in the agent
       *
       * @sa handleInit()
       * @sa getCurrentTick()
       */
      void initialize(TICK final);

      void   newTick();
      void   doNotify();
      bool   doSynchronize();
      double workRatio();
      void   step();
      
      typedef details::external external_iterator;
      typedef size_t            size_type;

      size_type         count_externals() const {
	return m_externals.size();
      }
      external_iterator ext_begin();
      external_iterator ext_end();

    protected:
      TeleoReactor(graph *owner, TREX::utils::Symbol const &name,
		   TICK latency, TICK lookahead, bool log=false);

      TREX::utils::LogManager &manager() const {
	return m_graph.manager();
      }



      virtual bool hasWork() {
	return false;
      }

      TICK getInitialTick() const {
	return 0; // May change in the future ...
      }
      TICK getCurrentTick() const {
	return m_graph.getCurrentTick();
      }
      TICK getFinalTick() const {
	return m_finalTick;
      }

      void postObservation(Observation const &obs);
      
      bool postGoal(goal_id const &g);
      goal_id postGoal(Goal const &g);
      /** @brief Goal completion
       *
       * @param[in] g A goal
       *
       * This method can be used from the reactor to inform the goal
       * management system that the goal @p g on one of its external
       * timeline has completed
       *
       * @deprecated This method should become useless as the goal management
       *             system is much more able to idenitify that a goal did
       *             complete
       */
      bool completedGoal(goal_id g);

      void relaxGoals();
      bool postRecall(goal_id const &g);

      virtual void handleInit() {}
      virtual void handleTickStart() {}

      virtual bool synchronize() =0;

      virtual void resume() {}
      
      virtual void handleRequest(goal_id const &g) {}
      virtual void handleRecall(goal_id const &g) {}

      void use(TREX::utils::Symbol const &timeline, bool control=true);
      void provide(TREX::utils::Symbol const &timeline);

      bool isInternal(TREX::utils::Symbol const &timeline) const;
      bool isExternal(TREX::utils::Symbol const &timeline) const;

      TREX::utils::internals::LogEntry 
      syslog(std::string const &context=std::string()) {
	if( context.empty() )
	  return m_graph.syslog(getName().str());
	else
	  return m_graph.syslog(getName().str()+"|"+context);
      }
      
    private:

      void clear_internals();
      void clear_externals();

      bool m_inited;
      graph &m_graph;

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
      TICK m_finalTick;
      
      mutable TICK m_deadline;
      mutable unsigned long m_nSteps;

      external_set m_externals;
      internal_set m_internals;

      /** @brief TREX log entry point
       *
       * Used to all the configuration and logging management for this reactor
       *
       * @sa syslog() const
       */
      TREX::utils::SingletonUse<TREX::utils::LogManager> m_log;

      void isolate() {
	clear_internals();
	clear_externals();
      }

      // call-backs from timeline
      void assigned(details::timeline *tl);
      void unassigned(details::timeline *tl);
      void subscribed(Relation const &r);
      void unsubscribed(Relation const &r);
      void latency_updated(TICK old_l, TICK new_l);
      
      
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
    

  } // TREX::transaction
} // TREX

# include "bits/bgl_support.hh"
    
#endif
