#ifndef H_europa_Bind
# define H_europa_Bind

# include <trex/europa/config.hh>

// include plasma header as system files in order to disable warnings
# define TREX_PP_SYSTEM_FILE <PLASMA/Constraint.hh>
# include <trex/europa/bits/system_header.hh>
# define TREX_PP_SYSTEM_FILE <PLASMA/Domain.hh>
# include <trex/europa/bits/system_header.hh>


namespace TREX {
  namespace europa {
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

#endif // H_europa_Bind
