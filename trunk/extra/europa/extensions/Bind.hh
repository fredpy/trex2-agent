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
    
    /** @brief Binding constraint
     *
     * This constraint is a backward compatibility constraint from TREX
     * It allows to set the first argument to the closed value of the 
     * second argument when this 2nd arg is a singleton.
     *
     * @deprecated This constraint was replaced by the much more flexible
     * TowardZero unbound variable manager. By using an unbound variable 
     * manager this allows to separate fixing variables ploiucy from the 
     * core model along with better configuarion ability on when such 
     * fixing should occur during deliberation.
     */
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
