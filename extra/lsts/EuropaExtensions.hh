#ifndef H_lsts_europa
# define H_lsts_europa

# include <trex/europa/config.hh>

# include <PLASMA/Constraint.hh>
# include <PLASMA/Domain.hh>

# include <Dune/Dune.hpp>

namespace TREX {
namespace LSTS {

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

	EUROPA::Domain &m_lat1;
	EUROPA::Domain &m_lon1;
	EUROPA::Domain &m_lat2;
	EUROPA::Domain &m_lon2;
	EUROPA::Domain &m_dist;

	enum indexes {
		DIST = 0,
		LAT1 = 1,
		LON1 = 2,
		LAT2 = 3,
		LON2 = 4,
		NARGS = 5
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
