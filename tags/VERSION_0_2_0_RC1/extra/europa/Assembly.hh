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
#ifndef H_Assembly
# define H_Assembly

# include <trex/utils/LogManager.hh>
# include <trex/transaction/Predicate.hh>
# include <trex/transaction/Tick.hh>

# include <PLASMA/PlanDatabase.hh>
# include <PLASMA/RulesEngineDefs.hh>
# include <PLASMA/Module.hh>
# include <PLASMA/Engine.hh>

namespace TREX {
  namespace europa {
    
    /** @brief Europa related error
     *
     * This exception depicts an error related to Europa.
     * @author Frederic Py <fpy@mbari.org>
     * @ingroup europa
     */
    class EuropaException :public TREX::utils::Exception {
    public:
      /** @brief Constructor 
       * @param[in] Associated error message
       */
      explicit EuropaException(std::string const &msg) throw() 
      :TREX::utils::Exception("EUROPA error: "+msg) {}
      /** @brief Destructor */
      virtual ~EuropaException() throw() {}
    }; // TREX::europa::EuropaException
    
    /** @brief Token related error
     * 
     * This exception indicates an error related to a psecific europa token
     */
    class TokenError :public EuropaException {
    public:
      /** @brief Constructor
       * 
       * @param[in] tok A token
       * @param[in] msg An error message
       * 
       * Creates a new instance that associates the error message @p msg to the 
       * token @p tok
       */
      TokenError(EUROPA::Token const &tok, std::string const &msg) throw();
      ~TokenError() throw() {}
    }; // TREX::europa::TokenError
    
    
    class EuropaReactor;
    class SchemaPlugin;
    class Assembly;
    
    namespace details {
      
      class Schema :boost::noncopyable {
      public:
        void registerComponents(Assembly const &assembly);
        void registerPlugin(SchemaPlugin &plugin);
        void unregisterPlugin(SchemaPlugin &plugin);

	void setStream(std::ostream &out);
	void setStream(std::ofstream &out, std::string const &name);
        
      private:
        Schema();
        ~Schema() {}
        
        std::set<SchemaPlugin *> m_plugins;
	std::ofstream m_europa_debug;
	TREX::utils::SingletonUse<TREX::utils::LogManager> m_log;
        
        friend class TREX::utils::SingletonWrapper<Schema>;
      }; // TREX::europa::details::Schema
      
    } // TREX::europa::details
    
    class DbSolver;
    /** @brief Interface toward europa
     *
     * This class connects to europa components manipulated by TREX
     * @ingroup europa 
     * @author Conor Mcgann @& Frederic Py <fpy@mbari.org>
     */
    class Assembly :public EUROPA::EngineBase, boost::noncopyable {
    public:
      /** @brief TREX agent timelines
       * 
       * The name of the type used to specify a timeline shared in TREX
       */
      static EUROPA::LabelStr const TREX_TIMELINE;

      /** @brief Europa external timelines descriptor
       * 
       * The value used to specify a timeline as @e External in an nddl model
       */
      static TREX::utils::Symbol const EXTERNAL_MODE;
      /** @brief Europa observed timelines descriptor
       * 
       * The value used to specify a timeline as @e External which will 
       * not accept goals in an nddl model
       */
      static TREX::utils::Symbol const OBSERVE_MODE;
      /** @brief Europa internal timelines descriptor
       * 
       * The value used to specify a timeline as @e Internal in an nddl model
       */
      static TREX::utils::Symbol const INTERNAL_MODE;
      /** @brief Europa private timelines descriptor
       * 
       * The value used to specify a timeline as private in an nddl model. 
       * Private timelines are TREX timelines that are not shared with other 
       * reactors. They behave like regular europa timelines
       */
      static TREX::utils::Symbol const PRIVATE_MODE;
      /** @brief Europa ignored timelines descriptor
       * 
       * The value used to specify that a specific timeline is ignored by the 
       * model
       * 
       * @deprecated This kind of timleine was used in the past when all europa 
       * reactors were loading the same model. As we now have each reactor 
       * loading their own model such feature is not really useful and may 
       * disappear in future versions
       */
      static TREX::utils::Symbol const IGNORE_MODE;
      
      /** @brief Europa mission end variable
       * 
       * The name of the variable in the model that is used to specify the 
       * the life time of the TREX agent
       * 
       * @sa EuropaReactor::getFinalTick() const
       */
      static EUROPA::LabelStr const MISSION_END;
      /** @brief tick duration variable
       *
       * The name of the variable in the model that is used to specify the 
       * duration of a tick. 
       * 
       * @sa EuropaReactor::tickDuration() const
       */
      static EUROPA::LabelStr const TICK_DURATION;

      static EUROPA::LabelStr const CLOCK_VAR;
      
      
      /** @brief @c undefined predicate name
       * 
       * The name of the @c undefined predicate for a TREX agent timeline. This 
       * predicate is often used whenever a new observation comes with a 
       * predicate name that is unknown by the europa model
       */
      static std::string const UNDEFINED_PRED;
      
      /** @brief Constructor
       *
       * @param[in] owner the EuropaReactor that created this instance
       * 
       * Create a new instance associated to @p owner
       */
      Assembly(EuropaReactor &owner);
      ~Assembly();

      void setStream() {
	m_trexSchema->setStream(m_dbg);
      }

      /** @brief Load a model
       *
       * @param[in] nddl A nddl file symbolic name
       * 
       * @pre A nddl configuration file named "NDDL.cfg" or "temp_nddl_gen.cfg"
       *      can be found in the TREX_PATH
       * @pre The file @p nddl exist and is a valid nddl file 
       * 
       * This method initialize the plan database using the model specified in 
       * the file @p nddl
       * 
       * @throw TREX::transaction::ReactorException An error occured while 
       *        trying to load the model or either of the nddl configuartion 
       *         files
       * @throw TREX::utils::XmlError Erro while parsing the XML content of the 
       *         nddl configuration file
       * @retval true The model was lsucessfully loaded and the plan data base 
       *              is consistent
       * @retval false The plan database is inconsitent after loading the model
       */
      bool playTransaction(std::string const &nddl);
      /** @brief That plan solver configuration
       * 
       * @param[in] A solver configuration file
       * 
       * @pre @p cfg is a file that can be found in TREX_PATH
       * @pre @p cfg is a file in XML that can be used to create a DbSolver
       * @pre This method has not been called before for this instance
       * 
       * Associate this class to a DbSolver configured using the xml content of 
       * @p cfg
       * @throw TREX::transaction::ReactorException Unable to locate @p cfg
       * @throw TREX::transaction::ReactorException Unable to parse @p cfg as XML
       * 
       * @sa DbSolver::DbSolver(Assembly const &, EUROPA::TiXmlElement const &) 
       */
      void configure_solver(std::string const &cfg);
      
      /** @brief Get europa schema
       *
       * @return the schema associated to this class
       */
      EUROPA::SchemaId const &schema() const {
        return m_schema;
      }
      /** @brief Get europa constraint engine
       *
       * @return the constraint engine associated to this class
       */
      EUROPA::ConstraintEngineId const &constraint_engine() const {
        return m_constraintEngine;
      }
      /** @brief Get europa plan database
       *
       * @return the plan database associated to this class
       */
      EUROPA::PlanDatabaseId const &plan_db() const {
        return m_planDatabase;
      }
      /** @brief Get europa rules engine
       *
       * @return the rule engine associated to this class
       */
      EUROPA::RulesEngineId const &rules_engine() const {
        return m_rulesEngine;
      }
      
      /** @brief Extract an object attribute
       * 
       * @param[in] obj An object
       * @param[in] str An attribute name
       * 
       * Extract the attribute @p str from the object @p obj
       * 
       * @pre @p obj has an attribute named @p str
       * @throw EuropaException The attribute @p str does not exist on @p obj
       * @return The variable associated to the attribute @p str on @p obj
       */
      EUROPA::ConstrainedVariableId attribute(EUROPA::ObjectId const &obj,
                                              std::string const &attr) const;
      /** @brief Get timeline mode
       * 
       * @param[in] obj An object
       *
       * @pre @p obj is a a trex timeline
       * 
       * Extract the mode of the timeline @p obj. This mode reflects whether 
       * the timeline is @e Internal, @e External or any other mode available.  
       * 
       * @throw EuropaException @p obj is not a TREX timeline
       * @return the variable that gives the mode of @p obj
       *
       * @sa INTERNAL_MODE
       * @sa EXTERNAL_MODE
       * @sa OBSERVE_MODE
       * @sa PRIVATE_MODE
       * @sa IGNORE_MODE
       */
      EUROPA::ConstrainedVariableId mode(EUROPA::ObjectId const &obj) const {
        return attribute(obj, MODE_ATTR);
      }
      /** @brief Get default predicate value
       * 
       * @param[in] obj An object
       * 
       * @pre @p obj is a trex timeline
       * 
       * Gets the timeline default predicate value. This value will be used to
       * set the timeline state whwnever the model cannot idenitfy its value 
       * 
       * @throw EuropaException @p obj is not a TREX timeline
       * @return the variable that gives the default predicate of @p obj
       */
      EUROPA::ConstrainedVariableId default_pred(EUROPA::ObjectId const &obj) const {
        return attribute(obj, DEFAULT_ATTR);
      }
      /** @brief Check for default predicate
       * 
       * @param[in] obj An object
       * 
       * @pre @p obj is a TREX timeline
       * @pre the default predicate for @p obj is a fully specified value (ie a 
       *      single value) and correspond ot a possible predicate for this 
       *      object
       * 
       * Check that @p obj specifies a correct default predicate
       * 
       * @throw EuropaException @p obj is not a TREX timeline
       * @throw EuropaException The default predicate of @p obj is not a singleton
       * @throw EuropaException the default predicate of @p obj is not a valid
       *                        predicate
       * @sa default_pred(EUROPA::ObjectId const &) const
       */
      void check_default(EUROPA::ObjectId const &obj) const;
      /** @brief Get the solver
       *
       * @return the solver used by this class
       * @sa configure_solver((std::string const &)
       */
      DbSolver &solver() const {
        return *m_solver;
      }
      bool propagate();
      bool is_unit(EUROPA::TokenId const &tok, EUROPA::TokenId &cand) const;
      bool overlaps_now(EUROPA::TokenId const &tok) const;
      bool in_synch_scope(EUROPA::TokenId const &tok, EUROPA::TokenId &cand) const;
      
      bool merge_token(EUROPA::TokenId const &tok, EUROPA::TokenId const &cand);
      bool insert_token(EUROPA::TokenId const &tok, size_t &steps);
      bool resolve(EUROPA::TokenId const &tok, EUROPA::TokenId const &cand, size_t &steps);
      bool insert_default(EUROPA::ObjectId const &obj, EUROPA::TokenId &tok, size_t &steps);
      
      /** @brief Get trex timelines
       *
       * @param[out] objs A list 
       * 
       * Extract all the timleines of the model that are specified as trex 
       * timelines and store them in @p objs
       *
       * Trex timelines are derived from the base class AgentTimeline in nddl;
       * this method allows to selct of the objects that derive from this class 
       * in the plan database
       * 
       * @sa TREX_TIMELINE
       */
      void trex_timelines(std::list<EUROPA::ObjectId> &objs) const {
        m_planDatabase->getObjectsByType(TREX_TIMELINE, objs);
      }
      /** @brief Set as ignored object
       * 
       * @param[in] obj An object
       * 
       * Add @p obj to the object to be ignored in the plan database
       * @post all the tokens associated to @p obj will be ignored during 
       * deliberation 
       */
      void ignore(EUROPA::ObjectId const &obj) {
        m_ignore.insert(obj);
      }
      /** @pre Ignored tokens filtering
       * 
       * @param[in] tok A token
       * 
       * Check if the token @p tok should be ignored. This mean that all the
       * possible values of its associated object are in the @p ignore list 
       * 
       * @retval true @p tok is ignored and can be discarded  from deliberation
       * @retval false there's still a possibility that @p tok is not to be 
       *               ignored
       * @sa ignore(EUROPA::ObjectId const &)
       * @sa isIgnored(EUROPA::ObjectId const &) const
       *
       */
      bool ignored(EUROPA::TokenId const &tok) const;
      /** @pre Check if part of deliberation
       *
       * @param[in] tok A token
       * 
       * Check if the token @p tok is part of current deliberation
       * 
       * This method is used during synchronization in order to avoid to badly 
       * impact any possible planning that happens meanwhile
       * 
       * @return true if @p tok is part of pending deliberation
       * @retval false otherwise
       */
      bool in_deliberation(EUROPA::TokenId const &tok) const;
      
      /** @brief Check if internal
       * 
       * @param[in] tok A token
       * 
       * @retval true if all the possible objects for @p tok are trex 
       *              @p Internal timelines
       * @retval false otherwise
       * @sa isInternal(EUROPA::ObjectId const &) const
       * @sa external(EUROPA::ObjectId const &) const
       */
      bool internal(EUROPA::TokenId const &tok) const;      
      /** @brief Check if external
       * 
       * @param[in] tok A token
       * 
       * @retval true if all the possible objects for @p tok are trex 
       *              @p External timelines
       * @retval false otherwise
       * @sa isExternal(EUROPA::ObjectId const &) const
       * @sa internal(EUROPA::ObjectId const &) const
       */
      bool external(EUROPA::TokenId const &tok) const;      
      
      /** @brief Check if internal 
       *
       * @param[in] obj An object 
       * 
       * Check if @p obj is a trex @e Internal timeline.
       * 
       * @note This checkinfg is at the reactor level -- and not simply ate the 
       *       model level. Indeed, it may happen that a timeline has been 
       *       specified as @e Internal by the model but was already owned by 
       *       another reactor. In that case a warning message would have been 
       *       displayed in TREX.log and this timeline mode would have been set 
       *       as @e Private and this method will return @c false
       * @bug I am not sure that this policy to migrate a timline from 
       *       @e Internal o @e Private is the good one. It may be better to 
       *       demote failed @e Internal timelines to @e External until we can 
       *       take ownership of this timeline ...
       * 
       * @retval true if @p obj is a trex @e Internal timeline
       * @retval false otherwise
       * @sa isExternal(EUROPA::ObjectId const &) const
       * @sa isIgnored(EUROPA::ObjectId const &) const
       */
      bool isInternal(EUROPA::ObjectId const &obj) const;	
      /** @brief Check if external 
       *
       * @param[in] obj An object 
       * 
       * Check if @p obj is a trex @e External timeline.
       * 
       * @note This checking is at the reactor level -- and not simply ate the 
       *       model level. Although for @p External timelines both the model 
       *       and trex should be aligned 
       * @retval true if @p obj is a trex @e External timeline
       * @retval false otherwise
       *
       * @sa isInternal(EUROPA::ObjectId const &) const
       * @sa isIgnored(EUROPA::ObjectId const &) const
       */
      bool isExternal(EUROPA::ObjectId const &obj) const;
      /** @brief Check if ignored 
       *
       * @param[in] obj An object 
       * 
       * Check if @p obj is a part of the objects to be ignored
       * 
       * @retval true if @p obj is part of the ignore list
       * @retval false otherwise
       *
       * @sa isInternal(EUROPA::ObjectId const &) const
       * @sa isIgnored(EUROPA::ObjectId const &) const
       * @sa ignore(EUROPA::ObjectId const &)
       */			
      bool isIgnored(EUROPA::ObjectId const &obj) const {
        return m_ignore.end()!=m_ignore.find(obj);
      }
      
      std::pair<EUROPA::ObjectId, EUROPA::TokenId> 
      convert(TREX::transaction::Predicate const &pred, bool fact);
      
      bool inactive() const {
        return INACTIVE==m_state;
      }
      bool active() const {
        return ACTIVE==m_state;
      }
      bool invalid() const {
        return INVALID==m_state;
      }
      void mark_inactive() {
        m_state = INACTIVE;
      }
      void mark_active();
      void mark_invalid() {
        m_state = INVALID;
      }

      TREX::transaction::TICK final_tick() const;
      
      void logPlan(std::ostream &out) const;
    private:
      enum State {
        INACTIVE = 0,
        ACTIVE, 
        INVALID
      };
      State m_state;
      EuropaReactor &m_reactor;
      DbSolver      *m_solver;
      
      EUROPA::SchemaId           m_schema;
      EUROPA::ConstraintEngineId m_constraintEngine;
      EUROPA::PlanDatabaseId     m_planDatabase;
      EUROPA::RulesEngineId      m_rulesEngine;
      
      TREX::utils::SingletonUse<details::Schema>         m_trexSchema;
      std::ofstream m_dbg;
      
      EUROPA::ObjectSet m_ignore;
      
      static std::string const MODE_ATTR;
      static std::string const DEFAULT_ATTR;
      
      
      Assembly();
    }; // TREX::europa::Assembly
    
    
    class SchemaPlugin {
    public:
      virtual ~SchemaPlugin();
      
      virtual void registerComponents(Assembly const &) =0;
      
    protected:
      SchemaPlugin();
      
    private:
      TREX::utils::SingletonUse<details::Schema> m_schema;
    }; // TREX::europa::SchemaPlugin
    
  } // TREX::europa
} // TREX


#define TREX_REGISTER_CONSTRAINT(assembly, class_name, label, propagator)\
  {  \
    EUROPA::ConstraintEngine* ce = (EUROPA::ConstraintEngine*) assembly.getComponent("ConstraintEngine"); \
    using EUROPA::ConcreteConstraintType; \
    using EUROPA::LabelStr; \
    REGISTER_CONSTRAINT(ce->getCESchema(), class_name, #label, #propagator);\
  }

#define TREX_REGISTER_FLAW_FILTER(assembly, class_name, label) \
  REGISTER_FLAW_FILTER(((EUROPA::SOLVERS::ComponentFactoryMgr*)assembly.getComponent("ComponentFactoryMgr")), class_name, label);

#define TREX_REGISTER_FLAW_HANDLER(assembly, class_name, label) \
  REGISTER_FLAW_HANDLER(((EUROPA::SOLVERS::ComponentFactoryMgr*)assembly.getComponent("ComponentFactoryMgr")), class_name, label);

#define TREX_REGISTER_FLAW_MANAGER(assembly, class_name, label) \
  REGISTER_FLAW_MANAGER(((EUROPA::SOLVERS::ComponentFactoryMgr*)assembly.getComponent("ComponentFactoryMgr")), class_name, label);

#define TREX_REGISTER_COMPONENT_FACTORY(assembly, class_name, label) \
  REGISTER_COMPONENT_FACTORY(((EUROPA::SOLVERS::ComponentFactoryMgr*)assembly.getComponent("ComponentFactoryMgr")), class_name, label);

#endif // H_Assembly
