#ifndef H_LogPlayer
# define H_LogPlayer

# include "TeleoReactor.hh"

namespace TREX {
  namespace transaction {

    class LogPlayer :public TeleoReactor {
    public:
      LogPlayer(TeleoReactor::xml_arg_type arg);
      ~LogPlayer();

    private:

      void handleInit();
      void handleTickStart();
      bool synchronize();

      void loadTransactions(rapidxml::xml_node<> &root, std::string const &reactor);
      
      std::multimap<TICK, Observation> m_obsLog;
      std::multimap<TICK, Observation>::const_iterator m_nextObs;

      std::multimap<TICK, goal_id> m_requestLog;
      std::multimap<TICK, goal_id>::const_iterator m_nextRequest;
      std::multimap<TICK, goal_id> m_recallLog;
      std::multimap<TICK, goal_id>::const_iterator m_nextRecall;
      
    }; // TREX::transaction::LogPlayer

  } // TREX::transaction 
} // TREX 

#endif // H_LogPlayer
