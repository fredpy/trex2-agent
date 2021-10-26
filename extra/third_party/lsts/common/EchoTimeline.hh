/*
 * EchoTimeline.hpp
 *
 *  Created on: Mar 27, 2014
 *      Author: zp
 */

#ifndef ECHOTIMELINE_HPP_
#define ECHOTIMELINE_HPP_

# include <trex/transaction/TeleoReactor.hh>
# include <trex/utils/Plugin.hh>
# include <trex/utils/LogManager.hh>
# include <trex/lsts/ImcAdapter.hh>
# include "../shared/LstsReactor.hh"

namespace TREX {
  namespace LSTS {

    class EchoTimeline :public TREX::LSTS::LstsReactor
    {
    public:
      // constructor
      EchoTimeline(TREX::transaction::TeleoReactor::xml_arg_type arg);
      // called before first tick
      bool synchronize();
      // called before first tick
      void handleInit();
      // called when a goal is requested
      void handleRequest(goal_id const &g);
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
