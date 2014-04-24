/*
 * TimelineReporter.hh
 *
 *  Created on: Apr 7, 2013
 *      Author: zp
 */

#ifndef TIMELINEREPORTER_HH_
#define TIMELINEREPORTER_HH_

# include <trex/transaction/reactor.hh>
# include <trex/utils/Plugin.hh>
# include <trex/utils/log_manager.hh>
# include <DUNE/DUNE.hpp>
# include "../shared/ImcAdapter.hh"

namespace TREX {
  /** @brief lsts plug-in
   *
   * This namespace is shared by all LSTS reactors/plugins
   * @ingroup lsts
   *
   * @author Jose Pinto <zepinto@gmail.com>
   */
  namespace LSTS {

    class TimelineReporter :public TREX::transaction::reactor,
    TREX::transaction::graph::timelines_listener {
    public:
      TimelineReporter(TREX::transaction::reactor::xml_arg_type arg);
      virtual ~TimelineReporter();


    private:
      void declared(transaction::details::timeline const &timeline);
      void undeclared(transaction::details::timeline const &timeline);
      //TREX::utils::SingletonUse<SharedEnvironment> m_env;
      bool aborted;
      bool m_output;
      int m_hostport;
      std::string m_hostaddr;
      ImcAdapter m_adapter;
      bool synchronize()
      {
        // nothing to do
        return !aborted;
      }

      void notify(TREX::transaction::Observation const &obs);
      void new_plan_token(TREX::transaction::goal_id const &t);
      void cancelled_plan_token(goal_id const &t);

    };
  }
}

#endif /* TIMELINEREPORTER_HH_ */
