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

    LstsReactor::LstsReactor(TeleoReactor::xml_arg_type arg)
    : TeleoReactor(arg, false)
    {

    }
    
    LstsReactor::~LstsReactor() {}

    bool
    LstsReactor::postUniqueObservation(TREX::transaction::Observation obs)
    {

      std::string timeline = obs.object().str();
      obs_map::iterator it = postedObservations.find(timeline);

      if (it == postedObservations.end() || !it->second->consistentWith(obs))
      {
        // If timeline wasn't previously created, create a new internal timeline
        if (!isInternal(timeline) && !isExternal(timeline))
          provide(timeline, false);

        postedObservations[timeline].reset(new Observation(obs));
        postObservation(obs, true);
        return true;
      }
      return false;
    }
  } /* namespace LSTS */
} /* namespace TREX */
