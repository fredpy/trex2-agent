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

    LstsReactor::LstsReactor(reactor::xml_arg_type arg)
    : reactor(arg, false)
    {

    }
    
    LstsReactor::~LstsReactor() {}


    bool
    LstsReactor::isObservationNew(TREX::transaction::token obs)
    {
      std::string timeline = obs.object().str();
      obs_map::iterator it = postedObservations.find(timeline);

      return (it == postedObservations.end() || !it->second->consistent_with(obs));
    }

    bool
    LstsReactor::postUniqueObservation(TREX::transaction::token obs)
    {

      utils::symbol timeline = obs.object();
      obs_map::iterator it = postedObservations.find(timeline.str());

      if (it == postedObservations.end() || !it->second->consistent_with(obs))
      {
        // If timeline wasn't previously created, create a new internal timeline
        if (!is_internal(timeline) && !is_external(timeline))
          provide(timeline, false);

        postedObservations[timeline.str()].reset(new token(obs));
        post_observation(obs, true);
        return true;
      }

      return false;
    }
  } /* namespace LSTS */
} /* namespace TREX */
