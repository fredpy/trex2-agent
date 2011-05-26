#ifndef H_DbSolver 
# define H_DbSolver

# include "Assembly.hh"

# include <PLASMA/Solver.hh>
# include <PLASMA/OpenConditionManager.hh>

namespace TREX {
  namespace europa {
    
    class DbSolver {
    public:
      DbSolver(Assembly const &assembly, EUROPA::TiXmlElement const &cfg);     
      ~DbSolver();

      bool isExhausted() const {
	return m_solver->isExhausted();
      }
      void step() {
	m_solver->step();
      }

      size_t getDepth() const {
	return m_solver->getDepth();
      }
      size_t getStepCount() const {
	return m_solver->getStepCount();
      }
      
      bool noMoreFlaws() const;
      void clear() {
	return m_solver->clear();
      }
      void reset() {
	return m_solver->reset();
      }

      bool inDeliberation(EUROPA::EntityId const &entity) const;

    private:
      EUROPA::SOLVERS::SolverId m_solver;      
    };

  } // TREX::europa
} // TREX

#endif // H_DbSolver 
