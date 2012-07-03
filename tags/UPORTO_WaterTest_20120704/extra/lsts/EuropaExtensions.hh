#ifndef H_lsts_europa
# define H_lsts_europa

# include <trex/europa/config.hh>

# include <PLASMA/Constraint.hh>
# include <PLASMA/Domain.hh>

# include <Dune/Dune.hpp>

# include <boost/polygon/polygon.hpp>


namespace TREX {
namespace LSTS {

typedef boost::polygon::polygon_data<double> Polygon;
typedef boost::polygon::polygon_traits<Polygon>::point_type Point;

class LatLonToOffset :public EUROPA::Constraint {
public:
	LatLonToOffset(EUROPA::LabelStr const &name,
			EUROPA::LabelStr const &propagator,
			EUROPA::ConstraintEngineId const &cstrEngine,
			std::vector<EUROPA::ConstrainedVariableId> const &vars);

	void handleExecute();

	static void set_home(Dune::IMC::HomeRef *home);

private:
	static Dune::IMC::HomeRef *s_home;

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

class LatLonDist :public EUROPA::Constraint {
public:
	LatLonDist(EUROPA::LabelStr const &name,
			EUROPA::LabelStr const &propagator,
			EUROPA::ConstraintEngineId const &cstrEngine,
			std::vector<EUROPA::ConstrainedVariableId> const &vars);

	void handleExecute();

private:

	EUROPA::Domain &m_dist;
	EUROPA::Domain &m_lat1;
	EUROPA::Domain &m_lon1;
	EUROPA::Domain &m_lat2;
	EUROPA::Domain &m_lon2;

	enum indexes {
		DIST = 0,
		LAT1 = 1,
		LON1 = 2,
		LAT2 = 3,
		LON2 = 4,
		NARGS = 5
	};
};

class InsideOpLimits :public EUROPA::Constraint {
public:
	InsideOpLimits(EUROPA::LabelStr const &name,
			EUROPA::LabelStr const &propagator,
			EUROPA::ConstraintEngineId const &cstrEngine,
			std::vector<EUROPA::ConstrainedVariableId> const &vars);

	void handleExecute();

	static void set_oplimits(Dune::IMC::OperationalLimits *oplimits);

private:
	static Dune::IMC::OperationalLimits * s_oplimits;

	EUROPA::Domain &m_inside;
	EUROPA::Domain &m_lat;
	EUROPA::Domain &m_lon;
	EUROPA::Domain &m_depth;

	enum indexes {
		INSIDE = 0,
	  LAT = 1,
		LON = 2,
		DEPTH = 3,
		NARGS = 4
	};
};

class RadDeg :public EUROPA::Constraint {
public:
	RadDeg(EUROPA::LabelStr const &name,
			EUROPA::LabelStr const &propagator,
			EUROPA::ConstraintEngineId const &engine,
			std::vector<EUROPA::ConstrainedVariableId> const &vars);
	void handleExecute();
private:
	EUROPA::Domain &m_deg;
	EUROPA::Domain &m_rad;

	enum indexes {
		RADIANS = 0,
		DEGREES = 1,
		NARGS   = 2
	};
}; // TREX::LSTS::RadDeg

}
}

#endif // H_lsts_europa
