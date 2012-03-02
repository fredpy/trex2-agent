#include "GoalManager.hh"
#include "Constraints.hh"

#include <PLASMA/Token.hh>
#include <PLASMA/TokenVariable.hh>
#include <PLASMA/TemporalAdvisor.hh>
#include <PLASMA/Number.hh>
#include <PLASMA/Filters.hh>
#include <PLASMA/Solver.hh>

using namespace starfish::europa;

const EUROPA::LabelStr GoalManager::CONST_PRIORITY("priority");
const EUROPA::LabelStr GoalManager::CONST_SPEED("DEFAULT_SPEED");
const EUROPA::LabelStr GoalManager::CONST_LOCATION("location");
const EUROPA::LabelStr GoalManager::CONST_FROM("from");
const EUROPA::LabelStr GoalManager::CONST_TO("to");
const EUROPA::LabelStr GoalManager::CONST_DISTANCE("distance");

//POSITION PARAMETER DEFINITION
const EUROPA::LabelStr GoalManager::XPOS("xpos");
const EUROPA::LabelStr GoalManager::YPOS("ypos");


GoalManager::GoalManager(EUROPA::TiXmlElement const &cfg)
:EUROPA::SOLVERS::OpenConditionManager(cfg),
 m_cycleCount(0),
 m_lastCycle(0) {}

/**
 * @brief When a new flaw is added we will re-evaluate all the options. This is accomplished
 * by incrementing a cycle count which makes existing solution stale.
 */
void GoalManager::addFlaw(const EUROPA::TokenId& token){
	OpenConditionManager::addFlaw(token);

	if(token->getState()->baseDomain().isMember(EUROPA::Token::REJECTED)){
		debugMsg("GoalManager:addFlaw", token->toString());
		m_cycleCount++;
	}
}

void GoalManager::removeFlaw(const EUROPA::TokenId& token){
	OpenConditionManager::removeFlaw(token);

	// If the flaw was rejected, come up with a revised solution to get better utility
	if(token->isRejected())
		m_cycleCount++;

	// If the token is active, remove it from the current solution if present
	if(token->isActive()) {
		SOLUTION::iterator it = m_currentSolution.begin();
		while(it != m_currentSolution.end()){
			EUROPA::TokenId t = *it;
			if(t == token){
				m_currentSolution.erase(it);
				break;
			}
			++it;
		}
	}

	// Clear from ommittedTokens
	m_ommissions.erase(token);
}

void GoalManager::handleInitialize(){
	OpenConditionManager::handleInitialize();
	m_cycleCount++;
}

/**
 * @brief Generates a simple seed solution
 */
void GoalManager::generateInitialSolution(){
	debugMsg("GoalManager:generateInitialSolution", "Resetting solution");

	m_currentSolution.clear();
	m_ommissions.clear();

	// Set the initial conditions since the problem may have moved on
	//initializes the startNode as well as assigning default speed
	setInitialConditions();

	// The empty solution is the default solution
	EUROPA::IteratorId it = OpenConditionManager::createIterator();
	while(!it->done()){
		EUROPA::TokenId goal = (EUROPA::TokenId) it->next();
		checkError(goal->getMaster().isNoId(), goal->toString());
		m_currentSolution.push_back(goal);
		debugMsg("GoalManager:debug", "adding solution");
	}

	delete (EUROPA::SOLVERS::FlawIterator*) it;
}

EUROPA::edouble GoalManager::computeDistance(const EUROPA::ObjectId& from, const EUROPA::ObjectId& to){
	if(from.isNoId())
		return 0;

	//checkError(from.isValid() && to.isValid(), "Bad id's");

	// Get source x,y
	EUROPA::edouble sourceX = from->getVariable(XPOS)->lastDomain().getSingletonValue();
	EUROPA::edouble sourceY = from->getVariable(YPOS)->lastDomain().getSingletonValue();

	// Get dest x,y
	EUROPA::edouble destX = to->getVariable(XPOS)->lastDomain().getSingletonValue();
	EUROPA::edouble destY = to->getVariable(YPOS)->lastDomain().getSingletonValue();

	return CalcDistanceConstraint::compute(sourceX, sourceY, destX, destY);
}

/**
 * @brief For now, this is going to be all about speed, distance and time
 */
bool GoalManager::evaluate(const SOLUTION& s, double& cost, double& utility){
	static const unsigned int MAX_PRIORITY = 5;
	cost = 0;
	utility = 0;
	EUROPA::TokenId predecessor;
	unsigned int numConflicts(0);

	static const EUROPA::LabelStr INACTIVE("Path.Inactive");
	EUROPA::edouble pathLength = 0;
	EUROPA::ObjectId currentNode = m_startNode;

	for(SOLUTION::const_iterator it = s.begin(); it != s.end(); ++it){
		EUROPA::TokenId candidate = *it;
		//checkError(candidate.isId() && candidate->getMaster().isNoId(), candidate->toString());
		debugMsg("GoalManager:debug","info " << candidate->toString());

		// Utility is 10 to the power of the priority
		utility += pow(10, (MAX_PRIORITY - getPriority(candidate)));

		// Check if there is a conflict
		if(predecessor.isId() && !getPlanDatabase()->getTemporalAdvisor()->canPrecede(predecessor, candidate))
			numConflicts++;

		// Retrieve Variables by parameter name
		EUROPA::ConstrainedVariableId loc = candidate->getVariable(CONST_LOCATION);
		EUROPA::ConstrainedVariableId distance = candidate->getVariable(CONST_DISTANCE);
		EUROPA::ConstrainedVariableId from = candidate->getVariable(CONST_FROM);
		EUROPA::ConstrainedVariableId to = candidate->getVariable(CONST_TO);

		if(loc.isId() || (from.isId() && to.isId() && distance.isId()) || candidate->getPredicateName() == INACTIVE)
			debugMsg("GoalManager:debug","Invalid goal structure for " << candidate->toString());

		if(loc.isId()){
			//checkError(loc->lastDomain().isSingleton(), loc->toString());
			EUROPA::ObjectId nextNode = EUROPA::Entity::getTypedEntity<EUROPA::Object>(loc->lastDomain().getSingletonValue());
			pathLength += computeDistance(currentNode, nextNode);
			currentNode = nextNode;
		}
		else if(distance.isId()){
			//checkError(from->lastDomain().isSingleton(), from->toString());
			//checkError(to->lastDomain().isSingleton(), to->toString());

			// Increment path length by distance to the start of the goal
			EUROPA::ObjectId fromNode = EUROPA::Entity::getTypedEntity<EUROPA::Object>(from->lastDomain().getSingletonValue());
			EUROPA::ObjectId toNode = EUROPA::Entity::getTypedEntity<EUROPA::Object>(to->lastDomain().getSingletonValue());
			pathLength += computeDistance(currentNode, fromNode);

			// Increment path length by internal distance estimation
			pathLength += distance->lastDomain().getLowerBound();
			currentNode = toNode;
		}

		predecessor = candidate;
	}

	// The use of a fixed multiplier here is a hack. This should be derived from the model and in particular, computed costs
	// from input tokens and the path, However, absent this, the constraint is too loose and thus we end up working hard in the search.
	cost = (EUROPA::cast_double(pathLength) * 1.30 / m_speed) + (numConflicts * pow(10, MAX_PRIORITY));
	debugMsg("GoalManager:debug","cost " <<cost<<" utility "<<utility);
	// Finally, feasibility is based on cost being within available budget for time
	return cost <= m_timeBudget;
}

std::string GoalManager::toString(const SOLUTION& s){
	double cost, utility;
	evaluate(s, cost, utility);

	std::stringstream ss;
	for(SOLUTION::const_iterator it = s.begin(); it != s.end(); ++it){
		EUROPA::TokenId t = *it;
		ss << t->getKey() << ":";
	}
	ss << "(" << cost << "/" << utility << ")";
	return ss.str();
}

/**
 * @brief Get the best neighbor for the current solution. The neighborhood is the set of solutions
 * within a single operation from the current solution. The operators are:
 * 1. insert
 * 2. swap
 * 3. remove
 * There are O(n^2) neigbors
 *
 * @note There is alot more we can do to exploit temporal constraints and evaluate feasibility. We are not including
 * deadlines and we are not factoring in the possibility that insertion in the solution imposes impled temporal constraints.
 */
void GoalManager::selectNeighbor(GoalManager::SOLUTION& s, EUROPA::TokenId& delta){
	//checkError(!m_currentSolution.empty() || !m_ommissions.empty(), "There must be something to do");
	s = m_currentSolution;
	delta = EUROPA::TokenId::noId();

	// Try insertions - could skip if current solution is infeasible.
	for(EUROPA::TokenSet::const_iterator it = m_ommissions.begin(); it != m_ommissions.end(); ++it){
		EUROPA::TokenId t = *it;
		for (unsigned int i = 0; i <= m_currentSolution.size(); i++){
			SOLUTION c = m_currentSolution;
			insert(c, t, i);
			update(s, delta, c, t);
		}
	}

	// Finally, swap
	for(unsigned int i=0; i< m_currentSolution.size(); i++){
		for(unsigned int j=i+1; j<m_currentSolution.size(); j++){
			if(i != j){
				SOLUTION c = m_currentSolution;
				swap(c, i, j);
				update(s, delta, c, EUROPA::TokenId::noId());
			}
		}
	}

	// Try removals
	for(SOLUTION::const_iterator it = m_currentSolution.begin(); it != m_currentSolution.end(); ++it){
		EUROPA::TokenId t = *it;

		if(t->isActive())
			continue;

		SOLUTION c = m_currentSolution;
		remove(c, t);
		update(s, delta, c, t);
	}
}

void GoalManager::insert(SOLUTION& s, const EUROPA::TokenId& t, unsigned int pos){
	//checkError(pos <= s.size(), pos << " > " << m_currentSolution.size());

	debugMsg("GoalManager:insert", "Inserting " << t->toString() << " at [" << pos << "] in " << toString(s));

	SOLUTION::iterator it = s.begin();
	for(unsigned int i=0;i<=pos;i++){
		// If we have the insertion point, do it and quit
		if (i == pos){
			s.insert(it, t);
			break;
		}

		// Otherwise advance
		++it;
	}
}

void GoalManager::swap(SOLUTION& s, unsigned int a, unsigned int b){
	debugMsg("GoalManager:swap", "Swapping [" << a << "] and [" << b << "] in " << toString(s));
	SOLUTION::iterator it = s.begin();
	SOLUTION::iterator it_a = s.begin();
	SOLUTION::iterator it_b = s.begin();
	EUROPA::TokenId t_a, t_b;
	unsigned int i = 0;
	unsigned int max_i = std::max(a, b);

	// Compute swap data
	while (i <= max_i){
		EUROPA::TokenId t = *it;
		if (i == a){
			it_a = it;
			t_a = t;
		}
		else if(i == b){
			it_b = it;
			t_b = t;
		}

		i++;
		++it;
	}

	checkError(a != b && it_a != it_b && t_a != t_b, "Should be different");

	// Execute swap
	it_a = s.erase(it_a);
	s.insert(it_a, t_b);
	it_b = s.erase(it_b);
	s.insert(it_b, t_a);
}

void GoalManager::remove(SOLUTION& s, const EUROPA::TokenId& t){
	debugMsg("GoalManager:remove", "Removing " << t->toString() << " from " << toString(s));
	SOLUTION::iterator it = s.begin();
	while((*it) != t && it != s.end()) ++it;
	//checkError(it != s.end(), "Not found");
	s.erase(it);
}

void GoalManager::update(SOLUTION& s, EUROPA::TokenId& delta, const SOLUTION& c, EUROPA::TokenId t){
	debugMsg("GoalManager:update", "Evaluating " << toString(c));
	if(compare(c, s) >= 0){
		s = c;
		delta = t;
		debugMsg("GoalManager:update", "Promote " << toString(c));
	}
}

int GoalManager::getPriority(const EUROPA::TokenId& token){
	// Slaves take top priority. Cannot be rejected.
	if(token->getMaster() == NULL)
		return 0;

	// Goals with no priority specified are assumed to be priority 0
	EUROPA::ConstrainedVariableId p = token->getVariable(CONST_PRIORITY);
	if(p.isNoId())
		return 0;

	// Now we get the actual priority value
	//checkError(p->lastDomain().isSingleton(), p->toString() << " must be set for " << token->toString());

	return EUROPA::cast_int(p->lastDomain().getSingletonValue());
}

void GoalManager::setInitialConditions(){
	// Start time
	const EUROPA::IntervalIntDomain& horizon = EUROPA::SOLVERS::HorizonFilter::getHorizon(); // has been changed to HorizonFilter
	m_startTime = EUROPA::cast_int(horizon.getLowerBound());
	m_timeBudget = EUROPA::cast_int((horizon.getUpperBound() - horizon.getLowerBound()));

	// TODO: Get a more accurate initial position and time
	m_startNode = EUROPA::ObjectId::noId();

	// SPEED: Eventually it will be based on each goal
	EUROPA::ConstrainedVariableId speed = getPlanDatabase()->getGlobalVariable(CONST_SPEED);
	//checkError(speed.isValid(), "No " << CONST_SPEED.toString() << ". " << speed);
	m_speed = EUROPA::cast_int(speed->lastDomain().getLowerBound());

	debugMsg("GoalManager:debug","startTime "<<m_startTime<<" timeBuget "<<m_timeBudget<<" m_speed "<<m_speed);
}

// callback method by the europa to obtain possible token iterator
// allow to return noId iterator.
EUROPA::IteratorId GoalManager::createIterator(){
	if(m_lastCycle < m_cycleCount){
		generateInitialSolution();

		search(1000);

		m_lastCycle = m_cycleCount;

		// Now we allocate a trivial iterator with just a single goal - the first available inactive token
		debugMsg("GoalManager:search", "Solution: " << toString(m_currentSolution));
	}

	EUROPA::TokenId nextGoal;
	for(SOLUTION::const_iterator it = m_currentSolution.begin(); it != m_currentSolution.end(); ++it){
		EUROPA::TokenId t = *it;
		if(t->isInactive()){
			nextGoal = t;
			debugMsg("GoalManager:search", "Selecting " << nextGoal->toString() << " in " << toString(m_currentSolution));
			break;
		}
	}
	debugMsg("GoalManager:debug", "goal dispatched : " << (new GoalManager::Iterator(*this, nextGoal))->getId());
	return (new GoalManager::Iterator(*this, nextGoal))->getId(); //pass iterator with only one token
}

void GoalManager::search(int iterations){
	// If nothing to be done, quit
	if(m_currentSolution.empty() && m_ommissions.empty())
		return;

	// Local Search for a max number of iterations
	int watchDog(0);
	int i(0);
	while(i < iterations && watchDog < 5){
		// Update counters to handle termination
		i++;
		watchDog++;

		// Get best neighbor
		EUROPA::TokenId delta;
		GoalManager::SOLUTION candidate;
		selectNeighbor(candidate, delta);

		// In the event that the candidate is the same solution, we have hit the end of exploration unless
		// we escape somehow
		if(candidate == m_currentSolution)
			break;

		// Promote if not worse. Allos for some exploration
		int result = compare(candidate, m_currentSolution);
		if(result >= 0){
			// If a token was removed, insert into ommitted token list, and opposite if appended
			if(m_currentSolution.size() > candidate.size())
				m_ommissions.insert(delta);
			else if(m_currentSolution.size() < candidate.size())
				m_ommissions.erase(delta);

			// Update current solution Values
			m_currentSolution = candidate;

			// Only pet the watchdog if we have improved the score. This allows us to terminate when
			// we have plateaued
			if(result == 1)
				watchDog = 0;

			debugMsg("GoalManager:search", "Switching to new solution: " << toString(m_currentSolution));
		}
		else
			break;
	}
}

/**
 * @brief is s1 better than s2
 * @return WORSE if s1 < s2. EQUAL if s1 == s2. BETTER if s1 > s2
 */
int GoalManager::compare(const SOLUTION& s1, const SOLUTION& s2){
	bool f1, f2;
	double c1, c2, u1, u2;
	f1 = evaluate(s1, c1, u1);
	f2 = evaluate(s2, c2, u2);

	// Feasibility is dominant.
	if(f1 && !f2)
		return BETTER;
	if(f2 && !f1)
		return WORSE;

	// If both infeasible, cost dominates
	if(!f1 && !f2 && c1 < c2)
		return BETTER;
	if(!f1 && !f2 && c1 > c2)
		return WORSE;

	// If both feasible, utility dominates
	if(f1 && f2 && u1 > u2)
		return BETTER;
	if(f1 && f2 && u1 < u2)
		return WORSE;

	// Finally, a straight comparison
	if((u1/c1) > (u2/c2))
		return BETTER;
	if((u1/c1) < (u2/c2))
		return WORSE;
	else
		return EQUAL;
}

GoalManager::Iterator::Iterator(GoalManager& manager, const EUROPA::TokenId& nextGoal)
: EUROPA::SOLVERS::FlawIterator(manager), m_nextGoal(nextGoal){ advance();}

const EUROPA::EntityId GoalManager::Iterator::nextCandidate(){
	EUROPA::EntityId nextGoal = (EUROPA::EntityId) m_nextGoal;
	m_nextGoal = EUROPA::EntityId::noId();
	return nextGoal;
}

// Intersect was here
bool GoalManager::isGoal(const EUROPA::EntityId& entity){
	//checkError(TokenId::convertable(entity), "Invalid configuration for " << entity->toString());
	EUROPA::TokenId candidate(entity);
	static const EUROPA::LabelStr INACTIVE("Path.Inactive");

	// If not rejectable, not a goal
	if(!candidate->getState()->baseDomain().isMember(EUROPA::Token::REJECTED))
		return false;

	// Retrieve Variables by parameter name
	EUROPA::ConstrainedVariableId loc = candidate->getVariable(CONST_LOCATION);
	EUROPA::ConstrainedVariableId distance = candidate->getVariable(CONST_DISTANCE);
	EUROPA::ConstrainedVariableId from = candidate->getVariable(CONST_FROM);
	EUROPA::ConstrainedVariableId to = candidate->getVariable(CONST_TO);

	return loc.isId() || (from.isId() && to.isId() && distance.isId()) || candidate->getPredicateName() == INACTIVE;
}

GoalsOnlyFilter::GoalsOnlyFilter(const EUROPA::TiXmlElement& configData): EUROPA::SOLVERS::FlawFilter(configData, true) {}

  bool GoalsOnlyFilter::test(const EUROPA::EntityId& entity){
    return !GoalManager::isGoal(entity);
  }

  NoGoalsFilter::NoGoalsFilter(const EUROPA::TiXmlElement& configData): EUROPA::SOLVERS::FlawFilter(configData, true) {}

  bool NoGoalsFilter::test(const EUROPA::EntityId& entity){
    return GoalManager::isGoal(entity);
  }


