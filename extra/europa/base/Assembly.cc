#include "EuropaReactor.hh"

#include "PLASMA/ModuleConstraintEngine.hh"
#include "PLASMA/ModulePlanDatabase.hh"
#include "PLASMA/ModuleRulesEngine.hh"
#include "PLASMA/ModuleTemporalNetwork.hh"
#include "PLASMA/ModuleSolvers.hh"
#include "PLASMA/ModuleNddl.hh"

#include "PLASMA/RulesEngine.hh"
#include "PLASMA/Propagators.hh"
#include "PLASMA/Schema.hh"
#include "PLASMA/NddlInterpreter.hh"

using namespace TREX::europa;
using namespace TREX::europa::details;
using namespace TREX::transaction;
using namespace TREX::utils;

/*
 * class TREX::europa::details::Schema
 */

// modifiers

void Schema::registerComponents(Assembly const &assembly) {
  EUROPA::ConstraintEngineId const &ce=assembly.constraint_engine();

  for(std::set<SchemaPlugin *>::const_iterator i = m_plugins.begin();
      m_plugins.end()!=i; ++i) 
    (*i)->registerComponents(assembly);
}

void Schema::registerPlugin(SchemaPlugin &pg) {
  m_plugins.insert(&pg);
}

void Schema::unregisterPlugin(SchemaPlugin &pg) {
  m_plugins.erase(&pg);
}

/*
 * class TREX::europa::SchemaPlugin
 */
// structors 

SchemaPlugin::SchemaPlugin() {
  m_schema->registerPlugin(*this);
}

SchemaPlugin::~SchemaPlugin() {
  m_schema->unregisterPlugin(*this);
}

/*
 * class TREX::europa::Assembly
 */ 

// statics

std::string const Assembly::MODE_ATTR("mode");
std::string const Assembly::DEFAULT_ATTR("defaultPredicate");

EUROPA::LabelStr const Assembly::TREX_TIMELINE("AgentTimeline");

EUROPA::LabelStr const Assembly::EXTERNAL_MODE("External");
EUROPA::LabelStr const Assembly::OBSERVE_MODE("Observer");
EUROPA::LabelStr const Assembly::INTERNAL_MODE("Internal");
EUROPA::LabelStr const Assembly::PRIVATE_MODE("Private");
EUROPA::LabelStr const Assembly::IGNORE_MODE("Ignore");

// structors 

Assembly::Assembly(EuropaReactor &owner)
  :m_reactor(owner) {
  addModule((new EUROPA::ModuleConstraintEngine())->getId());
  addModule((new EUROPA::ModuleConstraintLibrary())->getId());  
  addModule((new EUROPA::ModulePlanDatabase())->getId());
  addModule((new EUROPA::ModuleRulesEngine())->getId());
  addModule((new EUROPA::ModuleTemporalNetwork())->getId());
  addModule((new EUROPA::ModuleSolvers())->getId());
  addModule((new EUROPA::ModuleNddl())->getId());

  // complete base class initialization
  doStart();

  // initialization europa entry points
  m_schema = ((EUROPA::Schema *)getComponent("Schema"))->getId();
  m_constraintEngine = ((EUROPA::ConstraintEngine *)getComponent("ConstraintEngine"))->getId();
  m_planDatabase = ((EUROPA::PlanDatabase *)getComponent("PlanDatabase"))->getId();
  m_rulesEngine = ((EUROPA::RulesEngine *)getComponent("RulesEngine"))->getId();
  
  // register the new propagator used during synchronization
  new EUROPA::DefaultPropagator(EUROPA::LabelStr("OnCommit"), m_constraintEngine);

  // make sure that we control when constraints are propagated
  m_constraintEngine->setAutoPropagation(false);
  
  // get extra features from TREX schema
  m_trexSchema->registerComponents(*this);
  
  EUROPA::DomainComparator::setComparator((EUROPA::Schema *)m_schema);
}

Assembly::~Assembly() {
  // cleanup base class
  doShutdown();
}

// observers 

EUROPA::eint Assembly::specified_attribute(EUROPA::ObjectId const &obj, 
					   std::string const &attr) const {
  std::string full_name = obj->getName().toString()+"."+attr;
  EUROPA::ConstrainedVariableId var = obj->getVariable(full_name);
  if( var.isNoId() )
    throw EuropaException("Variable "+full_name+" does not exist.");
  if( !var->isSpecified() )
    throw EuropaException("Variable "+full_name+" is not fully specified.");
  return var->getSpecifiedValue();
}

bool Assembly::isInternal(EUROPA::ObjectId const &obj) const {
  return m_schema->isA(obj->getType(), TREX_TIMELINE)  &&
    m_reactor.isInternal(obj->getName().c_str());
}

bool Assembly::isExternal(EUROPA::ObjectId const &obj) const {
  return m_schema->isA(obj->getType(), TREX_TIMELINE)  &&
    m_reactor.isExternal(obj->getName().c_str());  
}

// modifiers 

bool Assembly::playTransaction(std::string const &nddl) {
  bool found;
  std::string config;

  config = m_reactor.manager().use("NDDL.cfg", found);
  if( !found ) {
    config = m_reactor.manager().use("temp_nddl_gen.cfg", found);
    if( !found ) 
      throw ReactorException(m_reactor, "Unable to locate NDDL.cfg or temp_nddl_gen.cfg");

    // First load the config file
    rapidxml::file<> cfg(config.c_str());
    // parse the file content
    rapidxml::xml_document<> cfg_xml;
    
    cfg_xml.parse<0>(cfg.data());

    rapidxml::xml_node<> const *xml_root = cfg_xml.first_node();
    if( NULL== xml_root )
      throw ReactorException(m_reactor, config+" does not appear to be in XML");

    // Extract include information
    for(rapidxml::xml_node<> const *child = xml_root->first_node("include");
	NULL!=child; child = child->next_sibling("include") ) {
      std::string path = parse_attr<std::string>(*child, "path");
      // replace ';' by ':'
      for(size_t pos=path.find(';'); pos<path.size(); pos=path.find(pos, ';'))
	path[pos] = ':';
      getLanguageInterpreter("nddl")->getEngine()->getConfig()->setProperty("nddl.includePath", path);   
    }
  }
  // Now add TREX_PATH to nddl include path
  std::ostringstream oss;
  oss<<'.';
  for(LogManager::path_iterator i=m_reactor.manager().begin(); 
      m_reactor.manager().end()!=i; ++i)
    oss<<':'<<*i;
  getLanguageInterpreter("nddl")->getEngine()->getConfig()->setProperty("nddl.includePath", oss.str());
  
  std::string ret;
  try {
    ret = executeScript("nddl", nddl, true);
  } catch(EUROPA::PSLanguageExceptionList const &le) {
    std::ostringstream err;
    err<<"Error while parsing nddl file:\n"
       <<le;
    throw ReactorException(m_reactor, err.str());
  } catch(Error const &error) {
    throw ReactorException(m_reactor, "Error while parsing nddl file: "+error.getMsg());
  }
  if( !ret.empty() ) 
    throw ReactorException(m_reactor, "Error returned after parsing nddl file: "+ret);

  // Model was loaded : check that the constraint network is still consistent
  return m_constraintEngine->constraintConsistent();
}

