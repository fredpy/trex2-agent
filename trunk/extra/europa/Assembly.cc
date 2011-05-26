#include "EuropaReactor.hh"
#include "bits/europa_convert.hh"
#include "DbSolver.hh"

#include <PLASMA/ModuleConstraintEngine.hh>
#include <PLASMA/ModulePlanDatabase.hh>
#include <PLASMA/ModuleRulesEngine.hh>
#include <PLASMA/ModuleTemporalNetwork.hh>
#include <PLASMA/ModuleSolvers.hh>
#include <PLASMA/ModuleNddl.hh>

#include <PLASMA/RulesEngine.hh>
#include <PLASMA/Propagators.hh>
#include <PLASMA/Schema.hh>
#include <PLASMA/NddlInterpreter.hh>
#include <PLASMA/TokenVariable.hh>

#include <PLASMA/XMLUtils.hh>

using namespace TREX::europa;
using namespace TREX::europa::details;
using namespace TREX::transaction;
using namespace TREX::utils;

TokenError::TokenError(EUROPA::Token const &tok, std::string const &msg) throw()
  :EuropaException(msg+"; on\n"+tok.toLongString()) {}

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

Symbol const Assembly::EXTERNAL_MODE("External");
Symbol const Assembly::OBSERVE_MODE("Observe");
Symbol const Assembly::INTERNAL_MODE("Internal");
Symbol const Assembly::PRIVATE_MODE("Private");
Symbol const Assembly::IGNORE_MODE("Ignore");

EUROPA::LabelStr const Assembly::MISSION_END("MISSION_END");
EUROPA::LabelStr const Assembly::TICK_DURATION("TICK_DURATION");

std::string const Assembly::UNDEFINED_PRED("undefined");


// structors 

Assembly::Assembly(EuropaReactor &owner)
  :m_reactor(owner), m_solver(NULL) {
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
  if( NULL!=m_solver ) {
    delete m_solver;
    m_solver = NULL; // just for being on the safe side ...
  }
  // cleanup base class
  doShutdown();
}

// observers 

void Assembly::check_default(EUROPA::ObjectId const &obj) const {
  EUROPA::ConstrainedVariableId var = default_pred(obj);
  std::ostringstream oss;

  if( var.isNoId() || !var->isSpecified() ) {
    oss<<"Internal timeline "<<obj->getName().toString()
       <<" does not specify its "<<DEFAULT_ATTR;
    throw EuropaException(oss.str());
  }
  EUROPA::DataTypeId type = var->getDataType();
  std::string short_pred = type->toString(var->getSpecifiedValue()),
    long_pred = obj->getType().toString()+"."+short_pred;
  if( !m_schema->isPredicate(long_pred.c_str()) ) {
    oss<<"Internal timeline "<<obj->getName().toString()
       <<" default predicate \""<<short_pred<<"\" does not exist";
    throw EuropaException(oss.str());
  }
}

EUROPA::ConstrainedVariableId Assembly::attribute(EUROPA::ObjectId const &obj, 
						  std::string const &attr) const {
  std::string full_name = obj->getName().toString()+"."+attr;
  EUROPA::ConstrainedVariableId var = obj->getVariable(full_name);
  if( var.isNoId() )
    throw EuropaException("Variable "+full_name+" does not exist.");
  return var;
}

bool Assembly::isInternal(EUROPA::ObjectId const &obj) const {
  return m_schema->isA(obj->getType(), TREX_TIMELINE)  &&
    m_reactor.isInternal(obj->getName().c_str());
}

bool Assembly::isExternal(EUROPA::ObjectId const &obj) const {
  return m_schema->isA(obj->getType(), TREX_TIMELINE)  &&
    m_reactor.isExternal(obj->getName().c_str());  
}

bool Assembly::ignored(EUROPA::TokenId const &tok) const {
  EUROPA::ObjectDomain const &dom = tok->getObject()->lastDomain();
  std::list<EUROPA::ObjectId> objs = dom.makeObjectList();
  std::list<EUROPA::ObjectId>::const_iterator o = objs.begin();

  for( ; objs.end()!=o; ++o)     
    if( !isIgnored(*o) )
      return false;
  return !objs.empty();
}

bool Assembly::in_deliberation(EUROPA::TokenId const &tok) const {
  if( ignored(tok) )
    return false;
  if( tok->getState()->lastDomain().isMember(EUROPA::Token::REJECTED) )
    return true;
  if( !active() || tok->isCommitted() )
    return false;
  return m_solver->inDeliberation(tok);
}

// modifiers 

void Assembly::configure_solver(std::string const &cfg) {
  if( NULL==m_solver ) {
    bool found;
    std::string file = m_reactor.manager().use(cfg, found);
    
    if( !found )
      throw ReactorException(m_reactor, "Unable to locate solver config file \""+
			     cfg+"\".");
    std::auto_ptr<EUROPA::TiXmlElement> xml(EUROPA::initXml(file.c_str()));
    
    if( NULL==xml.get() )
      throw ReactorException(m_reactor, "Unable to parse xml content of "+file);
    m_solver = new DbSolver(*this, *xml);
  } else 
    m_reactor.syslog("WARN")<<"Attempted to configure the solver more than once.";
}

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

std::pair<EUROPA::ObjectId, EUROPA::TokenId> 
Assembly::convert(Predicate const &pred, bool rejectable,
		  bool undefOnUnknown) {
  // create the token
  EUROPA::DbClientId cli =  m_planDatabase->getClient();
  EUROPA::ObjectId timeline = cli->getObject(pred.object().str().c_str());
  std::string 
    pred_name = timeline->getType().toString()+"."+pred.predicate().str();
  
  
  
  
  EUROPA::TokenId tok;

  if( m_schema->isPredicate(pred_name.c_str()) ) {    
    tok = cli->createToken(pred_name.c_str(), NULL, rejectable);
  
    if( tok.isId() ) { // < double check ...
      // Restrict attributes (apart the temporal ones)
      for(Predicate::const_iterator i=pred.begin(); pred.end()!=i; ++i) {
	EUROPA::ConstrainedVariableId param = tok->getVariable(i->first.str());
	
	if( param.isId() ) {
	  try {
	    europa_restrict(param, i->second.domain());	  
	  } catch(DomainExcept const &de) {
	    m_reactor.log("WARN")<<"Restriciting attribute "<<i->first
				    <<" on token "<<pred.object()<<'.'<<pred.predicate()
				    <<" triggered an exception: "<<de;
	  }
	} else 
	  m_reactor.log("WARN")<<"Unknown attribute "<<i->first
				  <<" for token "<<pred.object()<<'.'<<pred.predicate();
      }
      // Set the object
      EUROPA::ConstrainedVariableId 
	obj_var = tok->getObject();
      obj_var->specify(timeline->getKey());
      return std::make_pair(timeline, tok);
    }
  }
  // If I am here the mean that I failed 
  if( undefOnUnknown  ) {
    pred_name = timeline->getType().toString()+"."+UNDEFINED_PRED;
    
    m_reactor.log("WARN")<<"predicate "<<pred.object()<<'.'
			    <<pred.predicate()<<" is unknown by europa.\n"
			    <<"\tReplacing it by "<<UNDEFINED_PRED;
    if( m_schema->isPredicate(pred_name.c_str()) ) {    
      tok = cli->createToken(pred_name.c_str(), NULL, rejectable);
      // Set the object
      EUROPA::ConstrainedVariableId 
	obj_var = tok->getObject();
      obj_var->specify(timeline->getKey());
      return std::make_pair(timeline, tok);
    } else 
      m_reactor.log("WARN")<<"Not even able to create undefined !!!!";
  } else 
    m_reactor.log("WARN")<<"predicate "<<pred.object()<<'.'
			    <<pred.predicate()<<" is unknown by europa.\n"
			    <<"\tIgnoring it.";    
  return std::make_pair(timeline, EUROPA::TokenId::noId());
}

void Assembly::mark_active() {
  m_state = ACTIVE;
  m_reactor.reset_deliberation();
}
