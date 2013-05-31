/*
 * YoYoController.hpp
 *
 *  Created on: May 30, 2013
 *      Author: zp
 */

#ifndef YOYOCONTROLLER_HPP_
#define YOYOCONTROLLER_HPP_

# include <DUNE/DUNE.hpp>
# include <trex/transaction/Observation.hh>
# include "ReferenceController.hpp"

using DUNE_NAMESPACES;
using namespace TREX::transaction;

namespace TREX
{
  namespace LSTS
  {
    enum ExecutionState {INIT, YOYO_DOWN, YOYO_UP, SURFACE, END};

    class YoYoController : ReferenceController
    {
    public:
      YoYoController();
      void init(goal_id const &goal);
      bool finished();
      Reference control(EstimatedState * estate, FollowRefState * frefState);
      virtual
      ~YoYoController();

    private:
      double m_minz;
      double m_maxz;
      double m_endz;
      ExecutionState m_exec_state;
      int m_max_time_underwater, m_time_underwater;
      DUNE::IMC::Reference m_original_ref;
      DUNE::IMC::Reference m_ref;
      bool m_active;
    };

  } /* namespace LSTS */
} /* namespace TREX */
#endif /* YOYOCONTROLLER_HPP_ */
