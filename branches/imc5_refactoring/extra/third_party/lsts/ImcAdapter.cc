/*
 * ImcAdapter.cc
 *
 *  Created on: Apr 4, 2013
 *      Author: zp
 */

#include "ImcAdapter.hh"

namespace TREX {
  namespace LSTS {

    ImcAdapter::ImcAdapter()
    {
      // TODO Auto-generated constructor stub
    }

    Observation ImcAdapter::gpsFixObservation(GpsFix * msg)
    {
      if (msg == NULL)
        return Observation("gps", "Boot");

      if ((msg->validity & GpsFix::GFV_VALID_POS) != 0)
        return Observation("gps", "Valid");
      else
        return Observation("gps", "Invalid");
    }

    Observation ImcAdapter::estimatedStateObservation(EstimatedState * msg)
    {
      if (msg == NULL)
        return Observation("estate", "Boot");

      Observation obs("estate", "Position");
      double latitude, longitude;
      latitude = msg->lat;
      longitude = msg->lon;
      WGS84::displace(msg->x, msg->y, &latitude, &longitude);

      obs.restrictAttribute("latitude", FloatDomain(latitude));
      obs.restrictAttribute("longitude", FloatDomain(longitude));

      if (msg->depth != -1)
        obs.restrictAttribute("z", FloatDomain(msg->depth));
      else if (msg->alt != -1 )
        obs.restrictAttribute("z", FloatDomain(-msg->alt));
      else if (msg->alt != -1 )
        obs.restrictAttribute("z", FloatDomain(msg->height));

      return obs;
    }

    Observation ImcAdapter::followRefStateObservation(FollowRefState * msg)
    {
      if (msg == NULL || msg->reference.isNull() || msg->control_src != TREX_ID || msg->state == FollowRefState::FR_TIMEOUT)
        return Observation("reference", "Boot");

      bool xy_near = (msg->proximity & FollowRefState::PROX_XY_NEAR) != 0;
      bool z_near = (msg->proximity & FollowRefState::PROX_Z_NEAR) != 0;
      bool arrived = xy_near && z_near;

      std::string predicate = "Going";
      if (arrived)
        predicate = "At";

      Observation obs("reference", predicate);

      obs.restrictAttribute("latitude", FloatDomain(msg->reference->lat));
      obs.restrictAttribute("longitude", FloatDomain(msg->reference->lon));

      if (!msg->reference->z.isNull())
      {
        switch(msg->reference->z->z_units)
        {
          case (Z_DEPTH):
            obs.restrictAttribute("z", FloatDomain(msg->reference->z->value));
            break;
          case (Z_ALTITUDE):
            obs.restrictAttribute("z", FloatDomain(-msg->reference->z->value));
            break;
          case (Z_HEIGHT):
            obs.restrictAttribute("z", FloatDomain(msg->reference->z->value));
            break;
          default:
            break;
        }
      }

      if (!msg->reference->speed.isNull())
      {
        obs.restrictAttribute("speed",
                              FloatDomain((msg->reference->speed->value)));
      }
      return obs;
    }

    Observation ImcAdapter::planControlStateObservation(PlanControlState * msg)
    {
      if (msg != NULL)
      {
        if (msg->state == PlanControlState::PCS_BLOCKED)
        {
          return Observation("pcstate", "Blocked");
        }

        if (msg->state == PlanControlState::PCS_READY)
        {
          return Observation("pcstate", "Ready");
        }

        if (msg->state == PlanControlState::PCS_INITIALIZING)
        {
          Observation obs =  Observation("pcstate", "Initializing");
          obs.restrictAttribute("plan", StringDomain(msg->plan_id));
          return obs;
        }

        if (msg->state == PlanControlState::PCS_EXECUTING)
        {
          Observation obs =  Observation("pcstate", "Executing");
          obs.restrictAttribute("plan", StringDomain(msg->plan_id));
          return obs;
        }
      }
      return Observation("pcstate", "Boot");
    }

    Observation ImcAdapter::opLimitsObservation(OperationalLimits * msg)
    {
      if (msg == NULL)
        return Observation("oplimits", "Boot");

      Observation obs("oplimits", "Limits");

      if (msg->mask & IMC::OPL_MAX_DEPTH)
        obs.restrictAttribute("depth", FloatDomain(0, msg->max_depth));
      else
        obs.restrictAttribute("depth", FloatDomain(0, FloatDomain::plus_inf));

      FloatDomain::bound min_alt = 0, max_alt = FloatDomain::plus_inf;

      if ((msg->mask & IMC::OPL_MAX_ALT))
        max_alt = msg->max_altitude;

      if(msg->mask & IMC::OPL_MIN_ALT)
        min_alt = msg->min_altitude;

      obs.restrictAttribute("altitude", FloatDomain(min_alt, max_alt));

      FloatDomain::bound min_speed = 0, max_speed = FloatDomain::plus_inf;

      if (msg->mask & IMC::OPL_MAX_SPEED)
        max_speed = msg->max_speed;

      if (msg->mask & IMC::OPL_MIN_SPEED)
        min_speed = msg->min_speed;

      obs.restrictAttribute("speed", FloatDomain(min_speed, max_speed));

      InsideOpLimits::set_oplimits(msg);

      return obs;
    }

    ImcAdapter::~ImcAdapter()
    {
      // TODO Auto-generated destructor stub
    }
  }
}

