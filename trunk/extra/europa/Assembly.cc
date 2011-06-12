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
#include "EuropaReactor.hh"
#include "Constraints.hh"
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
#include <PLASMA/Debug.hh>
#include <PLASMA/Timeline.hh>

using namespace TREX::europa;
using namespace TREX::europa::details;
using namespace TREX::transaction;
using namespace TREX::utils;

namespace {
  
  class DefaultSchema :public SchemaPlugin {
  public:
    void registerComponents(Assembly const &assembly);
  }; 

  DefaultSchema trex_schema;

}


void DefaultSchema::registerComponents(Assembly const &assembly) {
  TREX_REGISTER_FLAW_FILTER(assembly, TREX::europa::DeliberationFilter, 
			    DeliberationFilter); 
  TREX_REGISTER_CONSTRAINT(assembly,TREX::europa::CheckExternal,isExternal,trex);
  TREX_REGISTER_CONSTRAINT(assembly,TREX::europa::CheckInternal,isInternal,trex);   
}

TokenError::TokenError(EUROPA::Token const &tok, std::string const &msg) throw()
 :EuropaException(msg+"; on\n"+tok.toLongString()) {}

/*
 * class TREX::europa::details::Schema
 */
std::string const Schema::EUROPA_DEBUG("Europa.log");


Schema::Schema() {
  m_europa_debug.open(m_log->file_name("Europa.log").c_str());
  DebugMessage::setStream(m_europa_debug);
  bool found;
  std::string cfg_dbg = m_log->use("Debug.cfg", found);
  
  if( found ) {
    std::ifstream cfg(cfg_dbg.c_str());
    DebugMessage::readConfigFile(cfg);
  } else
    m_log->syslog("EUROPA")<<"No Debug.cfg keeping default europa debug verbosity.";
}


// modifiers

void Schema::registerComponents(Assembly const &assembly) {
  // EUROPA::ConstraintEngineId const &ce=assembly.constraint_engine();
	
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
  
  // register the new propagator used by the reactor for special constraints
  new ReactorPropagator(*this, EUROPA::LabelStr("trex"), m_constraintEngine);
	
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

void Assembly::logPlan(std::ostream &out) const {
  EUROPA::TokenSet const all_toks = m_planDatabase->getTokens();
  out<<"digraph plan_"<<m_reactor.getCurrentTick()<<" {\n";
  out<<"  node[shape=\"box\"];\n";
  for(EUROPA::TokenSet::const_iterator i=all_toks.begin();
      all_toks.end()!=i; ++i) {
    if( !ignored(*i) ) {
      EUROPA::eint key;
      if( (*i)->isMerged() ) {
	key = (*i)->getActiveToken()->getKey();
      } else {
	key = (*i)->getKey();
	out<<"  t"<<key<<"[label=\""<<(*i)->getPredicateName().toString()<<"\\n"
	   <<" start "<<(*i)->start()->lastDomain().toString()<<"\\n"
	   <<" end "<<(*i)->end()->lastDomain().toString()<<"\\n"
	   <<" state "<<(*i)->getState()->lastDomain().toString()<<"\"";
	if( ignored(*i) || in_deliberation(*i) ) 
	  out<<" color=grey";
	else if( (*i)->isFact() )
	  out<<" color=red";
	else if( (*i)->isActive() )
	  out<<" color=blue";
	out<<"];\n";
      }      
      EUROPA::TokenId master = (*i)->master();
      if( master.isId() ) {
	if( master->isMerged() )
	  master = master->getActiveToken();
	out<<"  t"<<master->getKey()<<"->t"<<key
	   <<"[label=\""<<(*i)->getRelation().toString()<<"\"];\n";
      }
    }
  }
  out<<"}\n";
}

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

bool Assembly::internal(EUROPA::TokenId const &tok) const {
  EUROPA::ObjectDomain const &dom = tok->getObject()->lastDomain();
  std::list<EUROPA::ObjectId> objs = dom.makeObjectList();
  
  std::list<EUROPA::ObjectId>::const_iterator o = objs.begin();
  
  for( ; objs.end()!=o; ++o) 
    if( !isInternal(*o) )
      return false;
  return true;
}

bool Assembly::external(EUROPA::TokenId const &tok) const {
  EUROPA::ObjectDomain const &dom = tok->getObject()->lastDomain();
  std::list<EUROPA::ObjectId> objs = dom.makeObjectList();
  
  std::list<EUROPA::ObjectId>::const_iterator o = objs.begin();
  
  for( ; objs.end()!=o; ++o) 
    if( !isExternal(*o) )
      return false;
  return true;
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

bool Assembly::overlaps_now(EUROPA::TokenId const &tok) const {
  TREX::transaction::TICK cur = m_reactor.getCurrentTick();

  return tok->start()->lastDomain().getUpperBound()<=cur
  && tok->end()->lastDomain().getLowerBound()>=cur 
  && tok->end()->lastDomain().getLowerBound()>m_reactor.getInitialTick();
}

bool Assembly::in_synch_scope(EUROPA::TokenId const &tok, 
                              EUROPA::TokenId &cand) const {
  return overlaps_now(tok) && !( ignored(tok) || in_deliberation(tok) )
  && is_unit(tok, cand);
}

bool Assembly::is_unit(EUROPA::TokenId const &tok, EUROPA::TokenId &cand) const {
  if( tok->getObject()->lastDomain().isSingleton() ) {
    std::vector<EUROPA::TokenId> compats;
    size_t choices=0;
    
    m_planDatabase->getCompatibleTokens(tok, compats, UINT_MAX, true);
    for(std::vector<EUROPA::TokenId>::const_iterator i=compats.begin();
        compats.end()!=i && choices<=1; ++i)
      if( !in_deliberation(*i) ) {
        ++choices;
        cand = *i;
      }
    if( choices==1 && !m_planDatabase->hasOrderingChoice(tok) )
      return true;
    if( choices==0 ) {
      cand = EUROPA::TokenId::noId();
      return true;
    }
  }
  return false;
}


// modifiers 

bool Assembly::merge_token(EUROPA::TokenId const &tok, EUROPA::TokenId const &cand) {
  if( invalid() || cand.isNoId() )
    return false;
  if( !( tok->isInactive() && 
        tok->getState()->lastDomain().isMember(EUROPA::Token::MERGED) ) )
    return false;
  tok->merge(cand);
  propagate();
  return true;
}

bool Assembly::insert_token(EUROPA::TokenId const &tok, size_t &steps) {
  if( invalid() || in_deliberation(tok) )
    return false;
  if( tok->isInactive() )
    tok->activate();
  EUROPA::ObjectDomain const &objs=tok->getObject()->lastDomain();
  if( !objs.isSingleton() )
    return false;
  EUROPA::ObjectId obj = objs.getObject(objs.getSingletonValue());
  if( EUROPA::TimelineId::convertable(obj) ) {
    std::vector<EUROPA::OrderingChoice> choices;
    m_planDatabase->getOrderingChoices(tok, choices, 1);
    if( choices.empty() )
      return false;
    EUROPA::TokenId p = choices[0].second.first, s = choices[0].second.second;
    choices[0].first->constrain(p, s);
  }
  propagate();
  for(EUROPA::TokenSet::const_iterator i=tok->slaves().begin();
      tok->slaves().end()!=i; ++i) {
    if( invalid() )
      return false;
    EUROPA::TokenId cand;
    if( in_synch_scope(*i, cand) ) {
      ++steps;
      if( !resolve(*i, cand, steps) )
        return false;
    }
  }
  return true;
}

bool Assembly::resolve(EUROPA::TokenId const &tok, EUROPA::TokenId const &cand, size_t &steps) {
  if( merge_token(tok, cand) || insert_token(tok, steps) )
    return true;
  mark_invalid();
  return false;
}

bool Assembly::insert_default(EUROPA::ObjectId const &obj, EUROPA::TokenId &tok, size_t &steps) {
  TREX::transaction::TICK cur = m_reactor.getCurrentTick();
  EUROPA::IntervalIntDomain now(cur, cur);
  EUROPA::ConstrainedVariableId name = m_reactor.assembly().default_pred(obj);
  EUROPA::DataTypeId type = name->getDataType();
  std::string short_pred = type->toString(name->getSpecifiedValue()),
  pred_name = obj->getType().toString()+"."+short_pred;
  EUROPA::DbClientId cli = m_planDatabase->getClient();
  
  tok = cli->createToken(pred_name.c_str(), NULL, false);
  
  tok->activate();
  tok->start()->restrictBaseDomain(now);
  tok->getObject()->specify(obj->getKey());
  return insert_token(tok, steps);
}



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
    // Insert the DeliberationFilter 
    // EUROPA::TiXmlElement filter("FlawFilter");
    // filter.SetAttribute("component", "DeliberationFilter");
    // xml->InsertBeforeChild(xml->FirstChild(), filter);
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

bool Assembly::propagate() {
  if( invalid() )
    return false;
  if( !m_constraintEngine->propagate() ) {
    m_reactor.log("")<<"Inconsistent plan.";
    mark_invalid();
  }
  return !invalid();
}

std::pair<EUROPA::ObjectId, EUROPA::TokenId> Assembly::convert(Predicate const &pred, bool fact) {
  // First we create the token
  EUROPA::DbClientId cli    = m_planDatabase->getClient();
  EUROPA::ObjectId timeline = cli->getObject(pred.object().str().c_str());
  std::string pred_name = timeline->getType().toString()+"."+pred.predicate().str();
  EUROPA::TokenId tok = EUROPA::TokenId::noId();
  // if not a fact then it is rejectable
  bool rejectable = !fact;

  // Check that the predicate exists 
  if( m_schema->isPredicate(pred_name.c_str()) ) {
    tok = cli->createToken(pred_name.c_str(), NULL, rejectable, fact);
    if( tok.isId() ) {
      // restrict attributes 
      for(Predicate::const_iterator i=pred.begin(); pred.end()!=i; ++i) {
	EUROPA::ConstrainedVariableId param = tok->getVariable(i->first.str());
	
	if( param.isId() ) {
	  try {
	    europa_restrict(param, i->second.domain());
	  } catch(DomainExcept const &e) {
	    m_reactor.log("WARN")<<"Failed to restrict attribute "<<i->first
				 <<" on token "<<pred.object()<<'.'<<pred.predicate()
				 <<": "<<e;
	  }
	} else {
	  m_reactor.log("WARN")<<"Unknown attribute "<<i->first<<" on token "
			       <<pred.object()<<'.'<<pred.predicate();
	}
      }
    } 
  } else {
    // the predicate is unknown : give a warning 
    m_reactor.log("WARN")<<"Unknown predicate "<<pred.predicate()
			 <<" for object "<<pred.object();
    if( fact ) {
      // If it is a fact I need to deal with it : set it to "undefined"
      pred_name = timeline->getType().toString()+"."+UNDEFINED_PRED;
      if( m_schema->isPredicate(pred_name.c_str()) ) {
	tok = cli->createToken(pred_name.c_str(), NULL, rejectable, fact);
      } else {
	// that should never happen !!!
	std::ostringstream oss;
	oss<<"Unable to create special \""<<UNDEFINED_PRED
	   <<"\" predicate on object "<<timeline->getName().toString();
	throw EuropaException(oss.str());
      }
    }
  }
  if( tok.isId() ) {
    // Now restrict the object to timeline
    EUROPA::ConstrainedVariableId 
    obj_var=tok->getObject();
    obj_var->specify(timeline->getKey());
  }
  return std::make_pair(timeline, tok);
}

void Assembly::mark_active() {
  m_state = ACTIVE;
  m_reactor.reset_deliberation();
}
