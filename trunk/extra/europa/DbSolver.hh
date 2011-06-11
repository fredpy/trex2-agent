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
