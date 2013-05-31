/*
 * GoingController.hpp
 *
 *  Created on: May 31, 2013
 *      Author: zp
 */

#ifndef GOINGCONTROLLER_HPP_
#define GOINGCONTROLLER_HPP_

# include "ReferenceController.hpp"
# include <DUNE/DUNE.hpp>
# include <trex/transaction/Observation.hh>

using DUNE_NAMESPACES;
using namespace TREX::transaction;

namespace TREX
{
  namespace LSTS
  {

    class GoingController : ReferenceController
    {
    public:
      GoingController();
      void init(goal_id const &goal);
      bool finished();
      Reference control(EstimatedState * estate, FollowRefState * frefState);

      virtual
      ~GoingController();
    };

  } /* namespace LSTS */
} /* namespace TREX */
#endif /* GOINGCONTROLLER_HPP_ */
