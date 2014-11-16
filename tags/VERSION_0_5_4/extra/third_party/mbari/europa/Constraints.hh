#ifndef H_MBARI_Constraints
# define H_MBARI_Constraints

# include <trex/europa/config.hh>

# define TREX_PP_SYSTEM_FILE <PLASMA/Constraint.hh>
# include <trex/europa/bits/system_header.hh>
# define TREX_PP_SYSTEM_FILE <PLASMA/Domain.hh>
# include <trex/europa/bits/system_header.hh>



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
      EUROPA::Domain &m_number;
      EUROPA::Domain &m_letter;
      
      enum indexes {
	LAT = 0,
	LON = 1,
	NORTH = 2,
	EAST = 3,
        UTM_NUM = 4,
        UTM_LET = 5,
	NARGS = 6
      };
    };
    
  }
}

#endif // H_MBARI_Constraints
