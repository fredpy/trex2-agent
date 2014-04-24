/*
 * TimelineProxy.hh
 *
 *  Created on: May 7, 2013
 *      Author: zp
 */

#ifndef TIMELINEPROXY_HH_
#define TIMELINEPROXY_HH_

# include <trex/transaction/reactor.hh>
# include <trex/utils/Plugin.hh>
# include <trex/utils/log_manager.hh>
# include <DUNE/DUNE.hpp>
# include "../shared/ImcAdapter.hh"

namespace TREX {
  namespace LSTS {

    class TimelineProxy :public TREX::transaction::reactor {
    public:
      TimelineProxy(TREX::transaction::reactor::xml_arg_type arg);
      virtual ~TimelineProxy();

    private:

      // called before first tick
      void handle_init();

      // called each tick
      bool synchronize();

      // called when a goal is requested
      void handle_request(goal_id const &g);

      // called when an observation is posted
      void notify(Observation const &obs);

      /**
       * @brief should this reactor be used for posting goals (true) or observations (false)?
       */
      bool m_goalProxy;

      /**
       * @brief should this reactor use Iridium for forwarding tokens (true) or something else (false)?
      */
      bool m_useIridium;

      /**
       * Destination where to send tokens
       */
      int m_destport;
      std::string m_destaddr;

      /**
       * Where to receive tokens
       */
      int m_localport;

      /**
       * IMC object used for sending and receiving IMC messages
       */
      ImcAdapter m_adapter;

      /**
       * Timeline where to post goals / observations
       */
      std::string m_timeline;

      typedef std::map<std::string, transaction::goal_id> goal_map;
      goal_map m_goals;
    };
  }
}

#endif /* TIMELINEPROXY_HH_ */
