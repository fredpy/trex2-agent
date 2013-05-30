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

using DUNE_NAMESPACES;
using namespace TREX::transaction;

namespace TREX
{
  namespace LSTS
  {
    enum ExecutionState {INIT, YOYO_DOWN, YOYO_UP, SURFACE, END};

    class YoYoController
    {
    public:
      YoYoController(double minz, double maxz, double endz, int secs_underwater);
      void setFollowRefState(DUNE::IMC::FollowRefState * state);
      void setEstimatedState(DUNE::IMC::EstimatedState * state);
      void setTimeUnderwater(int seconds);
      void setReference(DUNE::IMC::Reference ref);
      Reference getCurrentReference();
      bool active;
      virtual
      ~YoYoController();

    private:
      double m_minz;
      double m_maxz;
      double m_endz;
      ExecutionState m_exec_state;
      int m_max_time_underwater, m_time_underwater;
      bool ascending;
      DUNE::IMC::FollowRefState * m_fref_state;
      DUNE::IMC::EstimatedState * m_estate;
      DUNE::IMC::Reference m_original_ref;
      DUNE::IMC::Reference m_ref;
    };

  } /* namespace LSTS */
} /* namespace TREX */
#endif /* YOYOCONTROLLER_HPP_ */
