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
  EUROPA::IteratorId iter = m_solver->createIterator();
  bool ret = m_solver->noMoreFlaws() && iter->done();
  delete (EUROPA::Iterator *)iter;
  return  ret;
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
