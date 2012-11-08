#ifndef H_STARFISH_Constraints
#define H_STARFISH_Constraints

#include <trex/europa/config.hh>

#include <PLASMA/Constraint.hh>
#include <PLASMA/Domain.hh>

namespace starfish {
namespace europa {

/**
 * @brief Precision tolerant form of intersection
 */
bool intersect(EUROPA::Domain& dom, EUROPA::edouble lb, EUROPA::edouble ub, EUROPA::edouble precision_error = 0.5);

class GeoUTMConstraint :public EUROPA::Constraint {
public:
	GeoUTMConstraint(EUROPA::LabelStr const &name,
			EUROPA::LabelStr const &propagator,
			EUROPA::ConstraintEngineId const &cstrEngine,
			std::vector<EUROPA::ConstrainedVariableId> const &vars);

	void handleExecute();

private:
	EUROPA::Domain &m_lat;
	EUROPA::Domain &m_lon;
	EUROPA::Domain &m_northing;
	EUROPA::Domain &m_easting;

	enum indexes {
		LAT = 0,
		LON = 1,
		NORTH = 2,
		EAST = 3,
		NARGS = 4
	};
};

class CalcDistanceConstraint : public EUROPA::Constraint {
public:
	CalcDistanceConstraint(EUROPA::LabelStr const &name,
			EUROPA::LabelStr const &propagator,
			EUROPA::ConstraintEngineId const &cstrEngine,
			std::vector<EUROPA::ConstrainedVariableId> const &vars);

	void handleExecute();

	/**
	 * Calculates the actual distance
	 */
	static EUROPA::edouble compute(EUROPA::edouble x1, EUROPA::edouble y1, EUROPA::edouble x2, EUROPA::edouble y2);

	/**
	 * Calculates the hypotenuse w. pythagaras
	 */
	static EUROPA::edouble compute(EUROPA::edouble a, EUROPA::edouble b);

private:

	static const unsigned int ARG_COUNT = 5;
	static const unsigned int DISTANCE = 0;
	static const unsigned int X1 = 1;
	static const unsigned int Y1 = 2;
	static const unsigned int X2 = 3;
	static const unsigned int Y2 = 4;

	EUROPA::Domain& m_distance;
	EUROPA::Domain& m_x1;
	EUROPA::Domain& m_y1;
	EUROPA::Domain& m_x2;
	EUROPA::Domain& m_y2;

};

    class Bind :public EUROPA::Constraint {
    public:
      Bind(EUROPA::LabelStr const &name, 
	   EUROPA::LabelStr const &propagatorName,
	   EUROPA::ConstraintEngineId const &cstrEngine,
	   std::vector<EUROPA::ConstrainedVariableId> const &variables);
      bool guardSatisfied() const;
      void handleExecute();
    private:
      EUROPA::ConstrainedVariableId m_target;
      EUROPA::ConstrainedVariableId m_default;
    }; // Bind


}
}

#endif // H_STARFISH_Constraints
