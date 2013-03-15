#ifndef H_MBARI_Constraints
# define H_MBARI_Constraints

# include <trex/europa/config.hh>

# pragma warning (push : 0)
# pragma GCC diagnostic ignored "-Wall"
// europa has a lot of warnings: lets make it more silent
#  include <PLASMA/Constraint.hh>
#  include <PLASMA/Domain.hh>
# pragma warning (pop)



namespace mbari {
  namespace europa {

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

  }
}

#endif // H_MBARI_Constraints
