#ifndef H_trex_europa_TrexThreatDecisionPoint 
# define H_trex_europa_TrexThreatDecisionPoint

#include <trex/europa/config.hh>

# include <PLASMA/ThreatDecisionPoint.hh>

namespace TREX {
  namespace europa {

    class Assembly;

    class TrexThreatDecisionPoint :public EUROPA::SOLVERS::ThreatDecisionPoint {
    public:
      TrexThreatDecisionPoint(EUROPA::DbClientId const &client,
			      EUROPA::TokenId const &tokenToOrder,
			      EUROPA::TiXmlElement const &configData,
			      EUROPA::LabelStr const &explanation = "trex");
            
    protected:
      void handleInitialize();
      
      EUROPA::eint now() const;
      
    private:
      EUROPA::ConstrainedVariableId m_clock;
    }; // TREX::europa::TrexThreatDecisionPoint


  } // TREX::europa
} // TREX

#endif // H_trex_europa_TrexThreatDecisionPoint
