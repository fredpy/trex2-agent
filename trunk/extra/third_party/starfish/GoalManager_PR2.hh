#ifndef H_GOALMANAGER_PR2
#define H_GOALMANAGER_PR2

/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2007, MBARI.
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
#include <trex/europa/Assembly.hh>

#include <PLASMA/OpenConditionManager.hh>
#include <PLASMA/FlawFilter.hh>

/**
 * @brief The goal manager.
 */


namespace starfish {
namespace europa {
struct Position {
	double x;
	double y;
};
class CostEstimator;

class GoalsOnlyFilter: public EUROPA::SOLVERS::FlawFilter {
public:
	GoalsOnlyFilter(const EUROPA::TiXmlElement& configData);
	bool test(const EUROPA::EntityId& entity);
};

class NoGoalsFilter: public EUROPA::SOLVERS::FlawFilter {
public:
	NoGoalsFilter(const EUROPA::TiXmlElement& configData);
	bool test(const EUROPA::EntityId& entity);
};

typedef EUROPA::Id<CostEstimator> CostEstimatorId;

/**
 * @brief Goal Manager to manage goal flaw selection. Will involve solving an orienteering problem
 * over the relaxed problem to provide a good ordering. We can imagine this manager maintaining
 * a priority q and releasing goals 1 at a time.
 */
class GoalManager_PR2: public EUROPA::SOLVERS::OpenConditionManager {
public:

	/**
	 * @brief Solution is a sequence of tokens. None can be rejected. Each has a 2d position.
	 */
	typedef std::list<EUROPA::TokenId> SOLUTION;

	/**
	 * @brief Uses standard constructor
	 */
	GoalManager_PR2(const EUROPA::TiXmlElement& configData);


	/**
	 * @brief Destructor
	 */
	~GoalManager_PR2();

	static const EUROPA::LabelStr X;
	static const EUROPA::LabelStr Y;
	static const EUROPA::LabelStr PRIORITY;

	static const EUROPA::LabelStr CFG_POSITION_SOURCE;
	static const EUROPA::LabelStr CFG_MAP_SOURCE;
	static const EUROPA::LabelStr CFG_MAX_ITERATIONS;
	static const EUROPA::LabelStr CFG_PLATEAU;

	// Assumptions about the fields in a goal
	// DECLARE_STATIC_CLASS_CONST(LabelStr, X, "x");
	// DECLARE_STATIC_CLASS_CONST(LabelStr, Y, "y");
	//DECLARE_STATIC_CLASS_CONST(LabelStr, PRIORITY, "priority");

	// Parameters used for configuration
	//DECLARE_STATIC_CLASS_CONST(LabelStr, CFG_POSITION_SOURCE, "positionSource");
	//DECLARE_STATIC_CLASS_CONST(LabelStr, CFG_MAP_SOURCE, "mapSource");
	//DECLARE_STATIC_CLASS_CONST(LabelStr, CFG_MAX_ITERATIONS, "maxIterations");
	//DECLARE_STATIC_CLASS_CONST(LabelStr, CFG_PLATEAU, "plateau");
	/**
	 * @brief True if the token is the next in the plan.
	 */
	bool isNextToken(EUROPA::TokenId token);
	/**
	 * @brief True if the planner has no more work to do.
	 */
	bool noMoreFlaws();
	/**
	 * @brief Steps the solver.
	 */
	void step();

	/**
	 * @brief Resets internal state
	 */
	void reset();

private:
	/**
	 * @brief Used to synch mark current goal value as dirty
	 * @see OpenConditionManager::addFlaw
	 */
	void addFlaw(const EUROPA::TokenId& token);

	/**
	 * @brief Used to synch mark current goal value as dirty
	 * @see OpenConditionManager::removeFlaw
	 */
	void removeFlaw(const EUROPA::TokenId& token);

	/**
	 * @brief Used to synch mark current goal value as dirty
	 */
	void handleInitialize();

	/**
	 * @brief Generate an initial solution. May not be feasible.
	 */
	void generateInitialSolution();

	/**
	 * @brief Helper method
	 */
	int getPriority(const EUROPA::TokenId& token);

	/**
	 * @brief Evaluate the solution s. This will compute both the cost and the utility.
	 * @return true if feasible, false if infeasible
	 */
	bool evaluate(const SOLUTION& s, double& cost, double& utility);

	/**
	 * @brief Compute a neighboring solution for s
	 */
	void selectNeighbor(GoalManager_PR2::SOLUTION& s, EUROPA::TokenId& delta);

	/**
	 * @brief Set initial conditions in terms of position, time and energy
	 */
	void setInitialConditions();

	/**
	 * @brief Get a distance estimate between points.
	 */
	virtual double computeDistance(const Position& p1, const Position& p2);

	/**
	 * @brief Accessor to get a Position. For convenience
	 */
	static Position getPosition(const EUROPA::TokenId& token);

	/**
	 * @brief Accessor to get current position (at current tick)
	 */
	Position getCurrentPosition();

	/**
	 * @brief Accessor for robot speed
	 */
	double getSpeed() const;

	/**
	 * @brief Comparator
	 */
	int compare(const SOLUTION& s1, const SOLUTION& s2);

	void insert(SOLUTION&s, const EUROPA::TokenId& t, unsigned int pos);
	void swap(SOLUTION& s, unsigned int a, unsigned int b);
	void remove(SOLUTION& s, const EUROPA::TokenId& t);
	void update(SOLUTION& s, EUROPA::TokenId& delta, const SOLUTION& c, EUROPA::TokenId t);


	std::string toString(const SOLUTION& s);

	void postConstraints();

	EUROPA::IteratorId createIterator();

	/**
	 * @brief A Trivial iterator - one goal only
	 */
	class Iterator : public EUROPA::SOLVERS::FlawIterator {
	public:
		Iterator(GoalManager_PR2& manager, const EUROPA::TokenId& nextGoal);

	private:
		const EUROPA::EntityId nextCandidate();
		EUROPA::TokenId m_nextGoal;
	};

	/** The state of the system. */
	enum State {
		STATE_DONE, STATE_PLANNING, STATE_REQUIRE_PLANNING
	};

	/**
	 * @brief Set the state. Encapsulate all change.
	 */
	void setState(const State& s);

	// Configuration derived members
	unsigned int m_maxIterations;
	unsigned int m_plateau;
	EUROPA::LabelStr m_positionSourceCfg;
	EUROPA::TimelineId m_positionSource;

	State m_state;
	TREX::europa::Assembly *m_assembly;
	SOLUTION m_currentSolution;
	EUROPA::TokenSet m_ommissions;
	std::vector< std::pair<int, EUROPA::ConstraintId> > m_constraints;

	// Iteration variables.
	unsigned int m_iteration, m_watchDog;

	/*!< INITIAL CONDITIONS */
	int m_startTime;
	double m_timeBudget;

	Position m_position; /*! Cached position. */

	// Integration with wavefront planner
	//plan_t* wv_plan;

	static const int WORSE = -1;
	static const int EQUAL = 0;
	static const int BETTER = 1;
};


typedef EUROPA::Id<GoalManager_PR2> GoalManager_PR2Id;

class CostEstimator : public EUROPA::SOLVERS::Component {
public:
	CostEstimator(const EUROPA::TiXmlElement& configData);
	virtual ~CostEstimator();
	virtual double computeDistance(const Position& p1, const Position& p2) = 0;
};

class EuclideanCostEstimator : public CostEstimator {
public:
	EuclideanCostEstimator(const EUROPA::TiXmlElement& configData);
	~EuclideanCostEstimator();
	double computeDistance(const Position& p1, const Position& p2);

};

}
}

#endif
