#ifndef FWD_Agent
# define FWD_Agent

# include "TeleoReactor_fwd.hh"

namespace TREX {
  namespace agent {

    class Agent;
    
    /** @brief Agent related exception
     *
     * This class is used to represent exceptiosn related to an agent error.
     *
     * @ingroups agent
     * @relates class Agent
     * @author Frederic Py <fpy@mbari.org>
     */
    class AgentException :public TREX::transaction::GraphException {
    public:
      /** @brief Constructor
       * @param[in] agent the agent that produced the exception
       * @param[in] msg An eroor message
       */
      AgentException(TREX::transaction::graph const &agent,
		     std::string const &msg) throw();
      /** @brief Destructor */
      virtual ~AgentException() throw() {}
    };

  } // TREX::agent
} // TREX

#endif // FWD_Agent
