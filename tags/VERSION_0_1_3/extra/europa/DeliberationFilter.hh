#ifndef H_DeliberationFilter 
# define H_DeliberationFilter

# include "Assembly.hh"
# include <trex/transaction/Tick.hh>

# include <PLASMA/Filters.hh> 


namespace TREX {
  namespace europa {

    class DeliberationFilter :public EUROPA::SOLVERS::FlawFilter {
    public:
      DeliberationFilter(EUROPA::TiXmlElement const &cfg);
      bool test(EUROPA::EntityId const &entity);
      
    private:
      void set_horizon(TREX::transaction::TICK start,
		       TREX::transaction::TICK end);

      static void set_current(EuropaReactor *me);
      static EuropaReactor *s_current;      

      EUROPA::IntervalIntDomain  m_horizon;
      Assembly                  &m_assembly;
      friend class EuropaReactor;
    }; 

  }
}

#endif // H_DeliberationFilter
