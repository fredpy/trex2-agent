#ifndef H_trex_europa_TrexThreatDecisionPoint 
# define H_trex_europa_TrexThreatDecisionPoint

#include <trex/europa/config.hh>

# include <PLASMA/ThreatDecisionPoint.hh>

namespace TREX {
  namespace europa {

    class Assembly;

    /** @brief A threat decision point that excludes insertion to the past
     *
     * This class implements a europa threat decision point that excludes decisions
     * which will insert the token necessarily into the past. It is useful to ensure 
     * that planning is made in the future and do not mistakenly insert a token in a 
     * past location (left open for example by the archiving of past tokens)
     *
     * @ingroup europa
     * @author Frderic Py
     */
    class TrexThreatDecisionPoint :public EUROPA::SOLVERS::ThreatDecisionPoint {
    public:
      /** @brief Constructor 
       *
       * @param[in] client A client to the plan database
       * @param[in] tokenToOrder the token to be ordered by this
       *              decision point
       * @param[in] configData Configuration information 
       * @param[in] rexplanation A string used to explain the decision
       *              source
       */
      TrexThreatDecisionPoint(EUROPA::DbClientId const &client,
			      EUROPA::TokenId const &tokenToOrder,
			      EUROPA::TiXmlElement const &configData,
			      EUROPA::LabelStr const &explanation = "trex");
      virtual ~TrexThreatDecisionPoint();
            
    protected:
      /** @brief initialize decision choices
       *
       * This method identifies all the valid choices available for
       * ordering this token.  It uses the same technique as the
       * default decision point but aafter that removes all the
       * insertion choices that would put the token to order
       * necessarily before the current tick -- dentified using the
       * global AGENT_CLOCK model variable
       *
       * @sa now()
       */
      void handleInitialize();
      
      /** @brief Current tick
       *
       * Extract the current tick date by extracting the lower bound
       * of the AGENT_CLOCK varaiable
       */
      EUROPA::eint now() const;
      
      virtual std::string toString() const;
      virtual std::string toShortString() const;


    private:
      EUROPA::ConstrainedVariableId m_clock;
    }; // TREX::europa::TrexThreatDecisionPoint

  } // TREX::europa
} // TREX

#endif // H_trex_europa_TrexThreatDecisionPoint
