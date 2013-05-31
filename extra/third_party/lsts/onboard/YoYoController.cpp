/*
 * YoYoController.cpp
 *
 *  Created on: May 30, 2013
 *      Author: zp
 */

#include "YoYoController.hpp"

using DUNE_NAMESPACES;

namespace TREX
{
  namespace LSTS
  {


    void
    YoYoController::init(goal_id const &goal)
    {
      double z;
      Goal *g = goal.get();
      Variable v;
      v = g->getAttribute("latitude");
      int flags = Reference::FLAG_LOCATION;

      if (v.domain().isSingleton())
        m_ref.lat = v.domain().getTypedSingleton<double, true>();

      v = g->getAttribute("longitude");
      if (v.domain().isSingleton())
        m_ref.lon = v.domain().getTypedSingleton<double, true>();

      v = g->getAttribute("z");
      if (v.domain().isSingleton())
      {
        z = v.domain().getTypedSingleton<double, true>();
        DesiredZ desZ;
        flags |= Reference::FLAG_Z;

        if (z >= 0)
        {
          desZ.value = z;
          desZ.z_units = Z_DEPTH;
        }
        else
        {
          desZ.value = -z;
          desZ.z_units = Z_ALTITUDE;
        }
        m_ref.z.set(desZ);
      }

      v = g->getAttribute("minz");
      if (v.domain().isSingleton())
      {
        m_minz = v.domain().getTypedSingleton<double, true>();
      }

      v = g->getAttribute("maxz");
      if (v.domain().isSingleton())
      {
        m_maxz = v.domain().getTypedSingleton<double, true>();
      }

      v = g->getAttribute("speed");
      if (v.domain().isSingleton())
      {
        double speed = v.domain().getTypedSingleton<double, true>();
        DesiredSpeed desSpeed;
        flags |= Reference::FLAG_SPEED;
        desSpeed.value = speed;
        desSpeed.speed_units = SUNITS_METERS_PS;
        m_ref.speed.set(desSpeed);
      }

      m_active = true;
    }

    bool
    YoYoController::finished()
    {
      return m_active;
    }

    Reference
    YoYoController::control(EstimatedState * estate, FollowRefState * frefstate)
    {
      if (estate == NULL || frefstate == NULL)
        return m_ref;

      DesiredZ desZ;
      switch(m_exec_state)
      {
        case INIT:
          std::cout << "INIT" << std::endl;
          desZ.value = m_maxz;
          desZ.z_units = Z_DEPTH;
          m_exec_state = YOYO_DOWN;
          break;

        case YOYO_DOWN:
          std::cout << "DOWN" << std::endl;
          if ((frefstate->proximity & FollowRefState::PROX_XY_NEAR) != 0)
          {
            desZ = *m_original_ref.z.get();
            m_exec_state = END;
          }
          else if (m_time_underwater >= m_max_time_underwater)
          {
            m_ref.lat = estate->lat;
            m_ref.lon = estate->lon;
            WGS84::displace(estate->x, estate->y, &(m_ref.lat), &(m_ref.lon));
            desZ.value = 0;
            desZ.z_units = Z_DEPTH;
            m_exec_state = SURFACE;
          }
          else if ((frefstate->proximity & FollowRefState::PROX_Z_NEAR) != 0)
          {
            desZ.value = m_minz;
            desZ.z_units = Z_DEPTH;
            m_exec_state = YOYO_UP;
          }
          break;

        case YOYO_UP:
          std::cout << "UP" << std::endl;
          if ((frefstate->proximity & FollowRefState::PROX_XY_NEAR) != 0)
          {
            desZ = *m_original_ref.z.get();
            m_exec_state = END;
          }
          else if (m_time_underwater >= m_max_time_underwater)
          {
            m_ref.lat = estate->lat;
            m_ref.lon = estate->lon;
            WGS84::displace(estate->x, estate->y, &(m_ref.lat), &(m_ref.lon));
            desZ.value = 0;
            desZ.z_units = Z_DEPTH;
            m_exec_state = SURFACE;
          }
          else if ((frefstate->proximity & FollowRefState::PROX_Z_NEAR) != 0)
          {
            desZ.value = m_maxz;
            desZ.z_units = Z_DEPTH;
            m_exec_state = YOYO_DOWN;
          }
          break;

        case SURFACE:
          std::cout << "SURFACE" << std::endl;
          if ((frefstate->proximity & FollowRefState::PROX_Z_NEAR) != 0)
          {
            m_ref = m_original_ref;
            desZ.value = m_maxz;
            desZ.z_units = Z_DEPTH;
            m_exec_state = YOYO_DOWN;
          }
          break;
        case END:
          std::cout << "END" << std::endl;
          m_active = false;
          break;
      }
      m_ref.z.set(&desZ);

      return m_ref;
    }

    YoYoController::YoYoController()
    {
      m_active = false;
      m_max_time_underwater = m_time_underwater = 0;
      m_exec_state = INIT;
      m_maxz = m_minz = m_endz = 0;
    }

    YoYoController::~YoYoController()
    {
      // TODO Auto-generated destructor stub
    }

  } /* namespace LSTS */
} /* namespace TREX */
