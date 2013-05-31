/*
 * ReferenceController.hpp
 *
 *  Created on: May 31, 2013
 *      Author: zp
 */

#ifndef REFERENCECONTROLLER_HPP_
#define REFERENCECONTROLLER_HPP_

# include <DUNE/DUNE.hpp>
# include <trex/transaction/Observation.hh>
# include <trex/transaction/TeleoReactor.hh>

using DUNE_NAMESPACES;
using namespace TREX::transaction;


namespace TREX
{
  namespace LSTS
  {

    class ReferenceController
    {
    public:
      ReferenceController();
      virtual void init(goal_id const &goal);
      virtual bool finished();
      virtual Reference control(EstimatedState * estate, FollowRefState * frefState);
      virtual
      ~ReferenceController();
    };

  } /* namespace LSTS */
} /* namespace TREX */
#endif /* REFERENCECONTROLLER_HPP_ */
