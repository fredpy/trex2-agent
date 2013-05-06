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

    Observation ImcAdapter::vehicleMediumObservation(VehicleMedium * msg)
    {
      if (msg != NULL)
      {
        switch (msg->medium) {
          case (VehicleMedium::VM_WATER):
            return Observation("medium", "Water");
            break;
          case (VehicleMedium::VM_UNDERWATER):
            return Observation("medium", "Underwater");
            break;
          case (VehicleMedium::VM_AIR):
            return Observation("medium", "Air");
            break;
          case (VehicleMedium::VM_GROUND):
            return Observation("medium", "Ground");
            break;
          default:
            break;
        }
      }
      return Observation("medium", "Unknown");
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
      else if (msg->height != -1 )
        obs.restrictAttribute("z", FloatDomain(msg->height));

      return obs;
    }

    Observation ImcAdapter::followRefStateObservation(FollowRefState * msg)
    {
      if (msg == NULL || msg->reference.isNull() || msg->control_src != TREX_ID
          || msg->state == FollowRefState::FR_TIMEOUT
          || msg->state == FollowRefState::FR_WAIT)
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

        if (msg->state == PlanControlState::PCS_EXECUTING && msg->plan_id == "trex_plan")
        {
          Observation obs =  Observation("control", "TREX");
          return obs;
        }
        return Observation("control", "DUNE");
      }

      return Observation("control", "Boot");
    }

    Observation ImcAdapter::opLimitsObservation(OperationalLimits * msg)
    {
      if (msg == NULL)
        return Observation("oplimits", "Boot");

      Observation obs("oplimits", "Limits");

      if (msg->mask & IMC::OPL_MAX_DEPTH)
        obs.restrictAttribute("max_depth", FloatDomain(msg->max_depth));

      if ((msg->mask & IMC::OPL_MAX_ALT))
        obs.restrictAttribute("max_altitude", FloatDomain(msg->max_altitude));

      if(msg->mask & IMC::OPL_MIN_ALT)
        obs.restrictAttribute("min_altitude", FloatDomain(msg->min_altitude));

      if (msg->mask & IMC::OPL_MAX_SPEED)
        obs.restrictAttribute("max_speed", FloatDomain(msg->max_speed));

      if (msg->mask & IMC::OPL_MIN_SPEED)
        obs.restrictAttribute("min_speed", FloatDomain(msg->min_speed));

      InsideOpLimits::set_oplimits(msg);

      return obs;
    }

    Observation genericObservation(TrexToken * msg)
    {
      Observation obs(msg->timeline, msg->predicate);
      IntegerDomain::bound min_i, max_i;
      FloatDomain::bound min_f, max_f;
      MessageList<TrexAttribute>::const_iterator it;

      for (it = msg->attributes.begin(); it != msg->attributes.end(); it++)
      {
        std::string min = (*it)->min;
        std::string max = (*it)->max;
        std::string attr = (*it)->name;

        switch((*it)->attr_type)
        {
          case TrexAttribute::TYPE_STRING:
            obs.restrictAttribute(attr, StringDomain(min));
            break;

          case TrexAttribute::TYPE_BOOL:
            if (min == max && min != "")
              obs.restrictAttribute(attr, BooleanDomain(min != "false" || min != "0"));
            else
              obs.restrictAttribute(attr, BooleanDomain());
            break;

          case TrexAttribute::TYPE_INT:
            if (min == "")
              min_i = IntegerDomain::minus_inf;
            else
              min_i = strtoll(max.c_str(), NULL, 10);

            if (max == "")
              max_i = IntegerDomain::plus_inf;
            else
              max_i = strtoll(max.c_str(), NULL, 10);

            obs.restrictAttribute(attr, IntegerDomain(min_i, max_i));
            break;

          case TrexAttribute::TYPE_FLOAT:
            if (min == "")
              min_f = FloatDomain::minus_inf;
            else
              min_f = strtod(min.c_str(), NULL);

            if (max == "")
              max_f = FloatDomain::plus_inf;
            else
              max_f = strtod(max.c_str(), NULL);

            obs.restrictAttribute(attr, FloatDomain(min_f, max_f));
            break;

          case TrexAttribute::TYPE_ENUM:
            if (min == "" || max == "")
              obs.restrictAttribute(attr, EnumDomain());
            else
              obs.restrictAttribute(attr, EnumDomain(min));
            break;

          default:
            break;
        }
      }

      return obs;
    }

    ImcAdapter::~ImcAdapter()
    {
      // TODO Auto-generated destructor stub
    }
  }
}

