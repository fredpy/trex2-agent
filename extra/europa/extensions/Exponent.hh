#ifndef H_trex_europa_Exponent
# define H_trex_europa_Exponent

# include <trex/europa/config.hh>

// include plasma header as system files in order to disable warnings
# define TREX_PP_SYSTEM_FILE <PLASMA/Constraint.hh>
# include <trex/europa/bits/system_header.hh>

namespace TREX {
  namespace europa {

    class Exponent :public EUROPA::Constraint {
    public: 
      Exponent(EUROPA::LabelStr const &name,
	       EUROPA::LabelStr const &propagatorName,
	       EUROPA::ConstraintEngineId const &cstrEngine,
	       std::vector<EUROPA::ConstrainedVariableId> const &vars);

    private:
      EUROPA::Domain &m_exp;
      EUROPA::Domain &m_log;
      
      void handleExecute();

   };

  }
}


#endif // H_trex_europa_Exponent
