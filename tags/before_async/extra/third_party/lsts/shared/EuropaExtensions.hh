#ifndef H_lsts_europa
# define H_lsts_europa

# include <trex/europa/config.hh>


// It may look like I include the same header twice but it is just a
// C preprocessor trick to make gcc thinks that PLASMA headers are
// system headers and therefore not complain but its bad implementation
# define TREX_PP_SYSTEM_FILE <PLASMA/Constraint.hh>
# include <trex/europa/bits/system_header.hh>
# define TREX_PP_SYSTEM_FILE <PLASMA/Domain.hh>
# include <trex/europa/bits/system_header.hh>
# include <trex/utils/shared_var.hh>
# include <DUNE/DUNE.hpp>

//# include <boost/polygon/polygon.hpp>


namespace TREX {
  namespace LSTS {
    
    //typedef boost::polygon::polygon_data<double> Polygon;
    //typedef boost::polygon::polygon_traits<Polygon>::point_type Point;
    
    class LatLonToOffset :public EUROPA::Constraint {
    public:
      LatLonToOffset(EUROPA::LabelStr const &name,
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
    
    class LatLonDisplace :public EUROPA::Constraint {
    public:
      LatLonDisplace(EUROPA::LabelStr const &name,
                     EUROPA::LabelStr const &propagator,
                     EUROPA::ConstraintEngineId const &cstrEngine,
                     std::vector<EUROPA::ConstrainedVariableId> const &vars);

      void handleExecute();

    private:

      EUROPA::Domain &m_lat;
      EUROPA::Domain &m_lon;
      EUROPA::Domain &m_northing;
      EUROPA::Domain &m_easting;
      EUROPA::Domain &m_latr;
      EUROPA::Domain &m_lonr;

      enum indexes {
        LAT = 0,
        LON = 1,
        NORTH = 2,
        EAST = 3,
        LATR = 4,
        LONR = 5,
        NARGS = 6
      };
    };

    class LatLonDisplacement :public EUROPA::Constraint {
       public:
         LatLonDisplacement(EUROPA::LabelStr const &name,
                        EUROPA::LabelStr const &propagator,
                        EUROPA::ConstraintEngineId const &cstrEngine,
                        std::vector<EUROPA::ConstrainedVariableId> const &vars);

         void handleExecute();

       private:

         EUROPA::Domain &m_lat;
         EUROPA::Domain &m_lon;
         EUROPA::Domain &m_northing;
         EUROPA::Domain &m_easting;
         EUROPA::Domain &m_latr;
         EUROPA::Domain &m_lonr;

         enum indexes {
           LAT = 0,
           LON = 1,
           NORTH = 2,
           EAST = 3,
           LATR = 4,
           LONR = 5,
           NARGS = 6
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
      
      static void set_oplimits(DUNE::IMC::OperationalLimits *oplimits);
      static DUNE::IMC::OperationalLimits * get_oplimits();
    private:
      static TREX::utils::shared_var<DUNE::IMC::OperationalLimits *> s_oplimits;
      
      EUROPA::Domain &m_lat;
      EUROPA::Domain &m_lon;
      EUROPA::Domain &m_depth;
      
      enum indexes {
        LAT = 0,
        LON = 1,
        DEPTH = 2,
        NARGS = 3
      };
    };
    
    // class RadDeg :public EUROPA::Constraint {
    // public:
    //   RadDeg(EUROPA::LabelStr const &name,
    //          EUROPA::LabelStr const &propagator,
    //          EUROPA::ConstraintEngineId const &engine,
    //          std::vector<EUROPA::ConstrainedVariableId> const &vars);
    //   void handleExecute();
    // private:
    //   EUROPA::Domain &m_deg;
    //   EUROPA::Domain &m_rad;
      
    //   enum indexes {
    //     RADIANS = 0,
    //     DEGREES = 1,
    //     NARGS   = 2
    //   };
    // }; // TREX::LSTS::RadDeg
    
    class PathCar :public EUROPA::Constraint {
    public:
      PathCar(EUROPA::LabelStr const &name,
              EUROPA::LabelStr const &propagator,
              EUROPA::ConstraintEngineId const &engine,
              std::vector<EUROPA::ConstrainedVariableId> const &vars);
      void handleExecute();
    }; // TREX::LSTS::PathCar

    class PathCdr :public EUROPA::Constraint {
    public:
      PathCdr(EUROPA::LabelStr const &name,
              EUROPA::LabelStr const &propagator,
              EUROPA::ConstraintEngineId const &engine,
              std::vector<EUROPA::ConstrainedVariableId> const &vars);
      void handleExecute();
    }; // TREX::LSTS::PathCar

    class PathEmpty :public EUROPA::Constraint {
    public:
      PathEmpty(EUROPA::LabelStr const &name,
                EUROPA::LabelStr const &propagator,
                EUROPA::ConstraintEngineId const &engine,
                std::vector<EUROPA::ConstrainedVariableId> const &vars);
      void handleExecute();
    }; // TREX::LSTS::PathCar
    
    
  }
}

#endif // H_lsts_europa
