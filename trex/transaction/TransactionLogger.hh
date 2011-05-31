/** @file TransactionLogger.hh
 * @brief Definition of reactors transaction logging class
 *
 * This header defines the class and utilities used to log the transactions
 * (meaning observation, requests and recalls) of a reactor.
 *
 * @author Frederic Py <fpy@mbari.org>
 * @ingroup transaction
 */
#ifndef H_TransactionLogger		       
# define H_TransactionLogger

# include <fstream>
# include <map>
# include <set>

# include "Relation.hh"
# include <trex/utils/LogManager.hh>

namespace TREX {
  namespace transaction {
    
    /** @brief Observation/Goal logger
     *
     * This class is used to log the transactions of reactors. The transactions
     * of a reactor are the observations produced and goals requests and recalls
     * this reactor did produce during its lifetime.
     *
     * This class is used to capture and serialize such transactions in as gfile
     * so they can be replayed for potential degbugging
     *
     * @note Since TREX maintain the dependency graph between reactor this class may
     *       have to change slightly in order to be more adapted to the new internal
     *       structure of an agent.
     * @deprecated This implemntation was assuming that the reactor relation were
     *             static and decided before starting the agent execution. This will
     *             become an invalid asumption as we allow to have the reactor graph
     *             being modified dynamically. 
     *
     * @author Frederic Py <fpy@mbari.org>
     * @ingroup transaction
     * @sa class LogPlayer
     * @deprecated Due to recent changes in the code the TransactionLogger needs
     *             a serious rework in order to truely support the new dynamic
     *             reactor management. It has been deactivated for now until fully
     *             compatible.
     */
    class TransactionLogger {
    public:
      /** @brief Internal timeline declaration
       *
       * @param[in] timeline A timeline name
       * @param[in] owner    A reactor name
       *
       * Notifies this instance that the reactor @p owner has declared
       * the @p timeline as @e Internal.
       */
      void declInternal(TREX::utils::Symbol const &timeline,
			TREX::utils::Symbol const &owner);
      /** @brief External timeline declaration
       *
       * @param[in] timeline A timeline name
       * @param[in] owner    A reactor name
       *
       * Notifies this instance that the reactor @p owner has declared
       * the @p timeline as @e External
       */
      void declExternal(TREX::utils::Symbol const &timeline,
			TREX::utils::Symbol const &owner);

      void endHeader(TICK init, std::string const &fileName);
      void newTick(TICK value);

      void logObservation(Observation const &obs);
      void logRequest(TREX::utils::Symbol const &reactor, goal_id const &goal);
      void logRecall(TREX::utils::Symbol const &reactor, goal_id const &goal);

      void endFile();
    private:
      bool m_inHeader;
      bool m_empty;
      
      std::ofstream m_logFile;

      std::map<TREX::utils::Symbol, TREX::utils::Symbol> m_internals;
      std::map<TREX::utils::Symbol, std::set<TREX::utils::Symbol> > m_externals;
      
      TICK m_lastTick;
      bool m_hasData;

      TREX::utils::SingletonUse<TREX::utils::LogManager> m_log;

 
      TransactionLogger();
      ~TransactionLogger();

      void startFile(std::string fileName);
      void openTick();
      friend class TREX::utils::SingletonWrapper<TransactionLogger>;
    }; // TREX::transaction::TransactionLogger

  } // TREX::transaction
} // TREX

#endif // H_TransactionLogger 
