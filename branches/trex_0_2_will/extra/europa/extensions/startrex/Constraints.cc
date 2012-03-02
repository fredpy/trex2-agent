#include "Constraints.hh"
#include "GoalManager.hh"


#include <mbari/shared/GeoUTM.hh>

#include <PLASMA/Number.hh>


namespace {
int const WGS_84 = 23;

void geo_to_utm(double lat, double lon, double &north, double &east) {

}


class STARFISHPlugin :public TREX::europa::SchemaPlugin {
public:
	void registerComponents(TREX::europa::Assembly const &assembly) {
		TREX_REGISTER_CONSTRAINT(assembly,starfish::europa::GeoUTMConstraint,
				geo_to_utm, trex);
		//TREX_REGISTER_CONSTRAINT(assembly,starfish::europa::CalcDistanceConstraint,
		//		calcDistance, trex);

		//TREX_REGISTER_FLAW_MANAGER(assembly, starfish::europa::GoalManager,
		//		GoalManager);
		//TREX_REGISTER_FLAW_FILTER(assembly, starfish::europa::GoalsOnlyFilter,
		//		GoalsOnly);
		//TREX_REGISTER_FLAW_FILTER(assembly, starfish::europa::NoGoalsFilter,
		//		NoGoals);
	}
};

STARFISHPlugin europa_extensions;

}

using namespace starfish::europa;

GeoUTMConstraint::GeoUTMConstraint(EUROPA::LabelStr const &name,
		EUROPA::LabelStr const &propagator,
		EUROPA::ConstraintEngineId const &cstrEngine,
		std::vector<EUROPA::ConstrainedVariableId> const &vars)
:EUROPA::Constraint(name, propagator, cstrEngine, vars),
 m_lat(getCurrentDomain(m_variables[GeoUTMConstraint::LAT])),
 m_lon(getCurrentDomain(m_variables[GeoUTMConstraint::LON])),
 m_northing(getCurrentDomain(m_variables[GeoUTMConstraint::NORTH])),
 m_easting(getCurrentDomain(m_variables[GeoUTMConstraint::EAST])) {
	// assert ?
}

void GeoUTMConstraint::handleExecute() {

}

CalcDistanceConstraint::CalcDistanceConstraint(EUROPA::LabelStr const &name,
		EUROPA::LabelStr const &propagator,
		EUROPA::ConstraintEngineId const &cstrEngine,
		std::vector<EUROPA::ConstrainedVariableId> const &vars)
:EUROPA::Constraint(name, propagator, cstrEngine, vars),
 m_distance(getCurrentDomain(m_variables[DISTANCE])),
 m_x1(getCurrentDomain(m_variables[X1])),
 m_y1(getCurrentDomain(m_variables[Y1])),
 m_x2(getCurrentDomain(m_variables[X2])),
 m_y2(getCurrentDomain(m_variables[Y2])){}

void CalcDistanceConstraint::handleExecute() {
	if(!m_x1.areBoundsFinite() ||
			!m_y1.areBoundsFinite()  ||
			!m_x2.areBoundsFinite() ||
			!m_y2.areBoundsFinite())
		return;

	debugMsg("CalcDistanceConstraint:handleExecute", "BEFORE:" << toString());

	// Compute bounds for dx
	EUROPA::NumericDomain dx;
	dx.insert(std::abs(m_x1.getLowerBound() - m_x2.getLowerBound()));
	dx.insert(std::abs(m_x1.getLowerBound() - m_x2.getUpperBound()));
	dx.insert(std::abs(m_x1.getUpperBound() - m_x2.getLowerBound()));
	dx.insert(std::abs(m_x1.getUpperBound() - m_x2.getUpperBound()));
	if(m_x1.intersects(m_x2))
		dx.insert(0.0);
	dx.close();

	// Compute bounds for dy
	EUROPA::NumericDomain dy;
	dy.insert(std::abs(m_y1.getLowerBound() - m_y2.getLowerBound()));
	dy.insert(std::abs(m_y1.getLowerBound() - m_y2.getUpperBound()));
	dy.insert(std::abs(m_y1.getUpperBound() - m_y2.getLowerBound()));
	dy.insert(std::abs(m_y1.getUpperBound() - m_y2.getUpperBound()));
	if(m_y1.intersects(m_y2))
		dy.insert(0.0);
	dy.close();

	intersect(m_distance, compute(dx.getLowerBound(), dy.getLowerBound()), compute(dx.getUpperBound(), dy.getUpperBound()));

	debugMsg("CalcDistanceConstraint:handleExecute", "AFTER:" << toString());
}

EUROPA::edouble CalcDistanceConstraint::compute(EUROPA::edouble x1, EUROPA::edouble y1, EUROPA::edouble x2, EUROPA::edouble y2){
	EUROPA::edouble result = std::sqrt(std::pow(x2-x1, 2)+ std::pow(y2-y1, 2));
	return result;
}

EUROPA::edouble CalcDistanceConstraint::compute(EUROPA::edouble a, EUROPA::edouble b){
	EUROPA::edouble result = std::sqrt(std::pow(a, 2)+ std::pow(b, 2));
	return result;
}

bool CalcDistanceConstraint::intersect(EUROPA::Domain& dom, EUROPA::edouble lb, EUROPA::edouble ub, EUROPA::edouble precision_error){
	//     static const double PRECISION_ERROR = 0.5;

	debugMsg("intersect", "Restrict " << dom.toString() << " with [" << lb << ", " << ub << "]");

	EUROPA::edouble _lb = (lb > dom.getUpperBound() ? lb - precision_error : lb);
	EUROPA::edouble _ub = (ub < dom.getLowerBound() ? ub + precision_error : ub);

	return dom.intersect(_lb, _ub);
}
