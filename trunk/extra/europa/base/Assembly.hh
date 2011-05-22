#ifndef H_Assembly
# define H_Assembly

# include "LogManager.hh"

# include "PLASMA/PlanDatabase.hh"
# include "PLASMA/RulesEngineDefs.hh"
# include "PLASMA/Module.hh"
# include "PLASMA/Engine.hh"

namespace TREX {
  namespace europa {

    class EuropaException :public TREX::utils::Exception {
    public:
      EuropaException(std::string const &msg) throw() 
	:TREX::utils::Exception("EUROPA error: "+msg) {}
      virtual ~EuropaException() throw() {}
    }; // TREX::europa::EuropaException

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

    class Assembly :public EUROPA::EngineBase, boost::noncopyable {
    public:
      static EUROPA::LabelStr const EXTERNAL_MODE;
      static EUROPA::LabelStr const OBSERVE_MODE;
      static EUROPA::LabelStr const INTERNAL_MODE;
      static EUROPA::LabelStr const PRIVATE_MODE;
      static EUROPA::LabelStr const IGNORE_MODE;

      Assembly(EuropaReactor &owner);
      ~Assembly();

      bool playTransaction(std::string const &nddl);
      
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

      EUROPA::eint specified_attribute(EUROPA::ObjectId const &obj, 
				       std::string const &attr) const;
      EUROPA::LabelStr mode(EUROPA::ObjectId const &obj) const {
	return EUROPA::LabelStr(specified_attribute(obj, MODE_ATTR));	 
      }
      EUROPA::LabelStr default_pred(EUROPA::ObjectId const &obj) const {
	return EUROPA::LabelStr(specified_attribute(obj, DEFAULT_ATTR));
      }
      void trex_timelines(std::list<EUROPA::ObjectId> &objs) const {
	m_planDatabase->getObjectsByType(TREX_TIMELINE, objs);
      }

      void ignore(EUROPA::ObjectId const &obj) {
	m_ignore.insert(obj);
      }

      bool isInternal(EUROPA::ObjectId const &obj) const;	
      bool isExternal(EUROPA::ObjectId const &obj) const;
      bool isIgnored(EUROPA::ObjectId const &obj) const {
	return m_ignore.end()!=m_ignore.find(obj);
      }

    private:
      EuropaReactor &m_reactor;

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
