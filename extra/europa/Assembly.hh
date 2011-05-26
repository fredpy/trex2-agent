#ifndef H_Assembly
# define H_Assembly

# include <trex/utils/LogManager.hh>
# include <trex/transaction/Predicate.hh>

# include <PLASMA/PlanDatabase.hh>
# include <PLASMA/RulesEngineDefs.hh>
# include <PLASMA/Module.hh>
# include <PLASMA/Engine.hh>

namespace TREX {
  namespace europa {
    
    class EuropaException :public TREX::utils::Exception {
    public:
      EuropaException(std::string const &msg) throw() 
	:TREX::utils::Exception("EUROPA error: "+msg) {}
      virtual ~EuropaException() throw() {}
    }; // TREX::europa::EuropaException

    class TokenError :public EuropaException {
    public:
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

      private:
	Schema() {}
	~Schema() {}
	
	std::set<SchemaPlugin *> m_plugins;

	friend class TREX::utils::SingletonWrapper<Schema>;
      }; // TREX::europa::details::Schema
      
    } // TREX::europa::details

    class DbSolver;

    class Assembly :public EUROPA::EngineBase, boost::noncopyable {
    public:
      static TREX::utils::Symbol const EXTERNAL_MODE;
      static TREX::utils::Symbol const OBSERVE_MODE;
      static TREX::utils::Symbol const INTERNAL_MODE;
      static TREX::utils::Symbol const PRIVATE_MODE;
      static TREX::utils::Symbol const IGNORE_MODE;

      static EUROPA::LabelStr const MISSION_END;
      static EUROPA::LabelStr const TICK_DURATION;

      static std::string const UNDEFINED_PRED;

      Assembly(EuropaReactor &owner);
      ~Assembly();

      bool playTransaction(std::string const &nddl);
      void configure_solver(std::string const &cfg);
      
      EUROPA::SchemaId const &schema() const {
	return m_schema;
      }
      EUROPA::ConstraintEngineId const &constraint_engine() const {
	return m_constraintEngine;
      }
      EUROPA::PlanDatabaseId const &plan_db() const {
	return m_planDatabase;
      }
      EUROPA::RulesEngineId const &rules_engine() const {
	return m_rulesEngine;
      }

      EUROPA::ConstrainedVariableId attribute(EUROPA::ObjectId const &obj,
					      std::string const &attr) const;
      EUROPA::ConstrainedVariableId mode(EUROPA::ObjectId const &obj) const {
	return attribute(obj, MODE_ATTR);
      }
      EUROPA::ConstrainedVariableId default_pred(EUROPA::ObjectId const &obj) const {
	return attribute(obj, DEFAULT_ATTR);
      }
      DbSolver &solver() const {
	return *m_solver;
      }

      void trex_timelines(std::list<EUROPA::ObjectId> &objs) const {
	m_planDatabase->getObjectsByType(TREX_TIMELINE, objs);
      }

      void ignore(EUROPA::ObjectId const &obj) {
	m_ignore.insert(obj);
      }
      bool ignored(EUROPA::TokenId const &obj) const;
      

      bool isInternal(EUROPA::ObjectId const &obj) const;	
      bool isExternal(EUROPA::ObjectId const &obj) const;
      bool isIgnored(EUROPA::ObjectId const &obj) const {
	return m_ignore.end()!=m_ignore.find(obj);
      }

      std::pair<EUROPA::ObjectId, EUROPA::TokenId> 
      convert(TREX::transaction::Predicate const &pred, bool rejectable,
	      bool undefOnUnknown);

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

      EUROPA::ObjectSet m_ignore;

      static std::string const MODE_ATTR;
      static std::string const DEFAULT_ATTR;

      static EUROPA::LabelStr const TREX_TIMELINE;
      
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

#endif // H_Assembly
