#ifndef H_europa_Bind
# define H_europa_Bind

#include <trex/europa/config.hh>

#include <PLASMA/Constraint.hh>
#include <PLASMA/Domain.hh>

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
