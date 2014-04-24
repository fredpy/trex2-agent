/*
 * EchoTimeline.hpp
 *
 *  Created on: Mar 27, 2014
 *      Author: zp
 */

#ifndef ECHOTIMELINE_HPP_
#define ECHOTIMELINE_HPP_

# include <trex/transaction/reactor.hh>
# include <trex/utils/Plugin.hh>
# include <trex/utils/log_manager.hh>
# include <DUNE/DUNE.hpp>
# include "../shared/ImcAdapter.hh"
# include "../shared/LstsReactor.hh"

namespace TREX {
  namespace LSTS {

    class EchoTimeline :public TREX::LSTS::LstsReactor
    {
    public:
      // constructor
      EchoTimeline(TREX::transaction::reactor::xml_arg_type arg);
      // called before first tick
      bool synchronize();
      // called before first tick
      void handle_init();
      // called when a goal is requested
      void handle_request(goal_id const &g);
      // destructor
      ~EchoTimeline();

    private:
      bool m_first_tick;
      std::string m_timeline;
      std::string m_initial_state;
    };

  } /* namespace TREX */
} /* namespace LSTS */

#endif /* ECHOTIMELINE_HPP_ */
