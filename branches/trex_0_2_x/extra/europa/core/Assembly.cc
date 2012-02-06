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
#include "Assembly.hh"
#include "private/Schema.hh"
#include "private/CurrentState.hh"

#include "ReactorPropagator.hh"

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

#include <PLASMA/Timeline.hh>
#include <PLASMA/TokenVariable.hh>

#include <PLASMA/XMLUtils.hh>
#include <PLASMA/Debug.hh>


using namespace TREX::europa;

/*
 * class TREX::europa::Assembly;
 */

// statics 

EUROPA::LabelStr const Assembly::TREX_TIMELINE("AgentTimeline");
EUROPA::LabelStr const Assembly::EXTERNAL_MODE("External");
EUROPA::LabelStr const Assembly::OBSERVE_MODE("Observe");
EUROPA::LabelStr const Assembly::INTERNAL_MODE("Internal");
EUROPA::LabelStr const Assembly::PRIVATE_MODE("Private");
EUROPA::LabelStr const Assembly::IGNORE_MODE("Ignore");
EUROPA::LabelStr const Assembly::MISSION_END("MISSION_END");
EUROPA::LabelStr const Assembly::TICK_DURATION("TICK_DURATION");
EUROPA::LabelStr const Assembly::CLOCK_VAR("AGENT_CLOCK");

std::string const Assembly::MODE_ATTR("mode");
std::string const Assembly::DEFAULT_ATTR("defaultPredicate");


std::string const Assembly::UNDEFINED_PRED("undefined");
std::string const Assembly::FAILED_PRED("Failed");

// structors

Assembly::Assembly(std::string const &name) {
  //  reditrec log output to <name>.europa.log
  m_trex_schema->setStream(m_debug, name+".europa.log");
  
  addModule((new EUROPA::ModuleConstraintEngine())->getId());
  addModule((new EUROPA::ModuleConstraintLibrary())->getId());
  addModule((new EUROPA::ModulePlanDatabase())->getId());
  addModule((new EUROPA::ModuleRulesEngine())->getId());
  addModule((new EUROPA::ModuleTemporalNetwork())->getId());
  addModule((new EUROPA::ModuleSolvers())->getId());
  addModule((new EUROPA::ModuleNddl())->getId());

  // complete base class Initialization
  doStart();

  // gather europa entry points
  m_schema = ((EUROPA::Schema *)getComponent("Schema"))->getId();
  m_cstr_engine = ((EUROPA::ConstraintEngine *)getComponent("ConstraintEngine"))->getId();
  m_plan_db = ((EUROPA::PlanDatabase *)getComponent("PlanDatabase"))->getId();
  m_rules_engine = ((EUROPA::RulesEngine *)getComponent("RulesEngine"))->getId();

  // Register the new propagator used for reactor related constraints
  new ReactorPropagator(*this, EUROPA::LabelStr("trex"), m_cstr_engine);

  // Make sure that we control when constraints propagate
  m_cstr_engine->setAutoPropagation(false);

  // Get extra europa extensions 
  m_trex_schema->registerComponents(*this);

  EUROPA::DomainComparator::setComparator((EUROPA::Schema *)m_schema);  
}

Assembly::~Assembly() {
  // cleanup base class
  doShutdown();
}

// manipulators

void Assembly::setStream() const {
  m_trex_schema->setStream(m_debug);
}

bool Assembly::playTransaction(std::string const &nddl) {
  std::string const &path = m_trex_schema->nddl_path();
  std::string ret;
  
  // configure search path for nddl
  getLanguageInterpreter("nddl")->getEngine()->getConfig()->setProperty("nddl.includePath", path);

  // parese nddl
  try {
    ret = executeScript("nddl", nddl, true);
  } catch(EUROPA::PSLanguageExceptionList const &l_err) {
    std::ostringstream err;
    err<<"Error while parsing "<<nddl<<":\n"<<l_err;
    throw EuropaException(err.str());
  } catch(Error const &error) {
    throw EuropaException("Error on parsing "+nddl+": "+error.getMsg());
  }
  if( !ret.empty() )
    throw EuropaException("Error returned after parsing "+nddl+": "+ret);

  return constraint_engine()->constraintConsistent();
}

void Assembly::notify(details::CurrentState const &state) {
  EUROPA::LabelStr obj_name = state.timeline()->getName();

  if( is_internal(obj_name) ) 
    notify(obj_name, state.current());
}

// observers

bool Assembly::internal(EUROPA::TokenId const &tok) const {
  EUROPA::ObjectDomain const &dom = tok->getObject()->lastDomain();
  std::list<EUROPA::ObjectId> objs = dom.makeObjectList();
  
  for(std::list<EUROPA::ObjectId>::const_iterator o=objs.begin();
      objs.end()!=o; ++o) 
    if( !internal(*o) )
      return false;
  return true;
}

bool Assembly::internal(details::CurrentState const &state) const {
  return is_internal(state.timeline()->getName());
}

bool Assembly::internal(EUROPA::ObjectId const &obj) const {
  return is_agent_timeline(obj) && is_internal(obj->getName());
}

bool Assembly::external(EUROPA::TokenId const &tok) const {
  EUROPA::ObjectDomain const &dom = tok->getObject()->lastDomain();
  std::list<EUROPA::ObjectId> objs = dom.makeObjectList();
  
  for(std::list<EUROPA::ObjectId>::const_iterator o=objs.begin();
      objs.end()!=o; ++o) 
    if( !external(*o) )
      return false;
  return true;
}

bool Assembly::external(EUROPA::ObjectId const &obj) const {
  return is_agent_timeline(obj) && is_external(obj->getName());
}

bool Assembly::ignored(EUROPA::TokenId const &tok) const {
  EUROPA::ObjectDomain const &dom = tok->getObject()->lastDomain();
  std::list<EUROPA::ObjectId> objs = dom.makeObjectList();
  
  for(std::list<EUROPA::ObjectId>::const_iterator o=objs.begin();
      objs.end()!=o; ++o) 
    if( !ignored(*o) )
      return false;
  return !objs.empty();
}

EUROPA::ConstrainedVariableId Assembly::attribute(EUROPA::ObjectId const &obj,
						  std::string const &attr) const {
  std::string full_name = obj->getName().toString()+"."+attr;
  EUROPA::ConstrainedVariableId var = obj->getVariable(full_name);
  if( var.isNoId() )
    throw EuropaException("Variable \""+full_name+"\" does not exist.");
  return var;
}

