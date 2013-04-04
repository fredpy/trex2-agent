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
      obs.restrictAttribute("depth", FloatDomain(msg->depth));
      obs.restrictAttribute("altitude", FloatDomain(msg->alt));
      obs.restrictAttribute("height", FloatDomain(msg->height));

      return obs;
    }

    Observation ImcAdapter::followRefStateObservation(FollowRefState * msg)
    {
      if (msg == NULL || msg->control_src != TREX_ID || msg->state == FollowRefState::FR_TIMEOUT)
        return Observation("frefstate", "Boot");

      Observation obs("frefstate", "Active");

      obs.restrictAttribute("tgt_lat", FloatDomain(msg->reference->lat));
      obs.restrictAttribute("tgt_lon", FloatDomain(msg->reference->lon));
      obs.restrictAttribute("tgt_z", FloatDomain(msg->reference->z->value));
      switch(msg->reference->z->z_units)
      {
        case (Z_DEPTH):
          obs.restrictAttribute("z_mode", StringDomain("depth"));
          break;
        case (Z_ALTITUDE):
          obs.restrictAttribute("z_mode", StringDomain("altitude"));
          break;
        case (Z_HEIGHT):
          obs.restrictAttribute("z_mode", StringDomain("height"));
          break;
        default:
          obs.restrictAttribute("z_mode", StringDomain("none"));
          break;
      }

      obs.restrictAttribute("xy_near",
                            BooleanDomain((msg->proximity & FollowRefState::PROX_XY_NEAR) != 0));
      obs.restrictAttribute("z_near",
                            BooleanDomain((msg->proximity & FollowRefState::PROX_Z_NEAR) != 0));

      obs.restrictAttribute("speed",
                            FloatDomain((msg->reference->speed->value)));

      return obs;
    }

    Observation ImcAdapter::vehicleStateObservation(VehicleState * msg)
    {
      if (msg == NULL)
        return Observation("vstate", "Boot");

    }

    ImcAdapter::~ImcAdapter()
    {
      // TODO Auto-generated destructor stub
    }
  }
}

