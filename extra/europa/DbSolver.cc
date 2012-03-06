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
#include "DbSolver.hh"

using namespace TREX::europa;

/*
 * class TREX::europa::DbSolver
 */
// structors 

DbSolver::DbSolver(Assembly const &assembly, EUROPA::TiXmlElement const &cfg) {
  m_solver = (new EUROPA::SOLVERS::Solver(assembly.plan_db(), cfg))->getId();
}

DbSolver::~DbSolver() {
  if( m_solver.isId() )
    delete (EUROPA::SOLVERS::Solver *)m_solver;
}

// observers 

bool DbSolver::noMoreFlaws() const {
  if( m_solver->noMoreFlaws() ) {
    EUROPA::IteratorId iter = m_solver->createIterator();
    bool ret = m_solver->noMoreFlaws() && iter->done();
    delete (EUROPA::Iterator *)iter;
    return  ret;
  }
  return false;
}

bool DbSolver::inDeliberation(EUROPA::EntityId const &entity) const {
  EUROPA::SOLVERS::DecisionStack const &decisions = m_solver->getDecisionStack();

  for(EUROPA::SOLVERS::DecisionStack::const_iterator it=decisions.begin();
      decisions.end()!=it; ++it) {
    if( entity->getKey()==(*it)->getFlawedEntityKey() )
      return true;
  }
  return false;
}
