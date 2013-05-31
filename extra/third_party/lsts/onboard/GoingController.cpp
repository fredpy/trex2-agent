/*
 * GoingController.cpp
 *
 *  Created on: May 31, 2013
 *      Author: zp
 */

#include "extra/third_party/lsts/onboard/GoingController.hpp"

namespace TREX
{
  namespace LSTS
  {

    GoingController::GoingController()
    {
      // TODO Auto-generated constructor stub

    }

    void init(goal_id const &goal);
    bool finished();
    Reference control(EstimatedState * estate, FollowRefState * frefState);


    GoingController::~GoingController()
    {
      // TODO Auto-generated destructor stub
    }

  } /* namespace LSTS */
} /* namespace TREX */
