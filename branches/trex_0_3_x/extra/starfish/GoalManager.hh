#ifndef H_starfish_europa_GoalManager
# define H_starfish_europa_GoalManager

# include <trex/europa/config.hh>

# include <PLASMA/OpenConditionManager.hh>
#include <PLASMA/FlawFilter.hh>
#include <PLASMA/FlawHandler.hh>

namespace starfish {
namespace europa {

class GoalManager :public EUROPA::SOLVERS::OpenConditionManager {
public:
	GoalManager(EUROPA::TiXmlElement const &cfg);
	/**
	 * @brief Solution is a sequence of goal tokens. None can be rejected.
	 */
	typedef std::list<EUROPA::TokenId> SOLUTION;

	/**
	 * @brief Handles the iteration. Will always impose a limit of the next goal
	 */
	EUROPA::IteratorId createIterator();

	static const EUROPA::LabelStr CONST_PRIORITY;
	static const EUROPA::LabelStr CONST_SPEED;
	static const EUROPA::LabelStr CONST_LOCATION;
	static const EUROPA::LabelStr CONST_FROM;
	static const EUROPA::LabelStr CONST_TO;
	static const EUROPA::LabelStr CONST_DISTANCE;

	static const EUROPA::LabelStr XPOS;
	static const EUROPA::LabelStr YPOS;

	static bool isGoal(const EUROPA::EntityId& candidate);

private:
private:

	/**
	 * @brief A Trivial iterator - one goal only
	 */
	class Iterator : public EUROPA::SOLVERS::FlawIterator {
	public:
		Iterator(GoalManager& manager, const EUROPA::TokenId& nextGoal);

	private:
		const EUROPA::EntityId nextCandidate();
		EUROPA::TokenId m_nextGoal;
	};

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
	 * @brief Search for best solution
	 */
	void search(int iterations);

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
	void selectNeighbor(GoalManager::SOLUTION& s, EUROPA::TokenId& delta);

	/**
	 * @brief Set initial conditions in terms of position, time and energy
	 */
	void setInitialConditions();

	/**
	 * @brief Compute euclidean distance for now
	 */
	EUROPA::edouble computeDistance(const EUROPA::ObjectId& from, const EUROPA::ObjectId& to);

	/**
	 * @brief Comparator
	 */
	int compare(const SOLUTION& s1, const SOLUTION& s2);

	void insert(SOLUTION&s, const EUROPA::TokenId& t, unsigned int pos);
	void swap(SOLUTION& s, unsigned int a, unsigned int b);
	void remove(SOLUTION& s, const EUROPA::TokenId& t);
	void update(SOLUTION& s, EUROPA::TokenId& delta, const SOLUTION& c, EUROPA::TokenId t);

	std::string toString(const SOLUTION& s);

	unsigned int m_cycleCount;
	unsigned int m_lastCycle;
	SOLUTION m_currentSolution;
	EUROPA::TokenSet m_ommissions;

	/*!< INITIAL CONDITIONS */
	int m_startTime;
	double m_timeBudget;
	double m_speed;
	EUROPA::ObjectId m_startNode;
	double m_energyBudget;

	static const int WORSE = -1;
	static const int EQUAL = 0;
	static const int BETTER = 1;
};

class GoalsOnlyFilter: public  EUROPA::SOLVERS::FlawFilter {
public:
  GoalsOnlyFilter(const EUROPA::TiXmlElement& configData);
  bool test(const EUROPA::EntityId& entity);
};

class NoGoalsFilter: public  EUROPA::SOLVERS::FlawFilter {
public:
  NoGoalsFilter(const EUROPA::TiXmlElement& configData);
  bool test(const EUROPA::EntityId& entity);
};



} // starfish::europa
} // starfish

#endif // H_starfish_europa_GoalManager
