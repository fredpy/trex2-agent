/*
 * LstsReactor.cpp
 *
 *  Created on: May 6, 2013
 *      Author: zp
 */

# include "LstsReactor.hh"

using namespace TREX::transaction;

namespace TREX
{
  namespace LSTS
  {

    bool
    LstsReactor::isObservationNew(TREX::transaction::Observation obs)
    {

      std::string timeline = obs.object().str();
      obs_map::iterator it = postedObservations.find(timeline);

      if (it == postedObservations.end() || !it->second->consistentWith(obs))
      {
        postedObservations[timeline].reset(new Observation(obs));
        postObservation(obs);
        return true;
      }
      return false;
    }

  } /* namespace LSTS */
} /* namespace TREX */
