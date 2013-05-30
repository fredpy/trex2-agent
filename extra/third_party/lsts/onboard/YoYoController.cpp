/*
 * YoYoController.cpp
 *
 *  Created on: May 30, 2013
 *      Author: zp
 */

#include "extra/third_party/lsts/onboard/YoYoController.hpp"

using DUNE_NAMESPACES;

namespace TREX
{
  namespace LSTS
  {

    YoYoController::YoYoController(double minz, double maxz, double endz, int secs_underwater)
    {
      m_fref_state = NULL;
      m_estate = NULL;
      m_minz = minz;
      m_maxz = maxz;
      m_endz = endz;
      m_exec_state = INIT;
      m_max_time_underwater = secs_underwater;
      m_time_underwater = 0;
      ascending = false;
      active = false;

      std::cout << "YoyoController: " << minz << " -> " << maxz << std::endl;
    }

    void YoYoController::setFollowRefState(DUNE::IMC::FollowRefState * state)
    {
      m_fref_state = state;
    }

    void YoYoController::setEstimatedState(DUNE::IMC::EstimatedState * state)
    {
      m_estate = state;
    }

    void YoYoController::setReference(DUNE::IMC::Reference ref)
    {
      m_ref = m_original_ref = ref;
    }

    void YoYoController::setTimeUnderwater(int seconds)
    {
      m_time_underwater = seconds;
    }

    Reference YoYoController::getCurrentReference()
    {

      if (m_fref_state == NULL || m_estate == NULL)
        return m_ref;

      DesiredZ desZ;
      switch(m_exec_state)
      {
        case INIT:
          desZ.value = m_maxz;
          desZ.z_units = Z_DEPTH;
          m_exec_state = YOYO_DOWN;
          break;

        case YOYO_DOWN:
          if ((m_fref_state->proximity & FollowRefState::PROX_XY_NEAR) != 0)
          {
            desZ = *m_original_ref.z.get();
            m_exec_state = END;
          }
          else if (m_time_underwater >= m_max_time_underwater)
          {
            m_ref.lat = m_estate->lat;
            m_ref.lon = m_estate->lon;
            WGS84::displace(m_estate->x, m_estate->y, &(m_ref.lat), &(m_ref.lon));
            desZ.value = 0;
            desZ.z_units = Z_DEPTH;
            m_exec_state = SURFACE;
          }
          else if ((m_fref_state->proximity & FollowRefState::PROX_Z_NEAR) != 0)
          {
            desZ.value = m_minz;
            desZ.z_units = Z_DEPTH;
            m_exec_state = YOYO_UP;
          }
          break;

        case YOYO_UP:
          if ((m_fref_state->proximity & FollowRefState::PROX_XY_NEAR) != 0)
          {
            desZ = *m_original_ref.z.get();
            m_exec_state = END;
          }
          else if (m_time_underwater >= m_max_time_underwater)
          {
            m_ref.lat = m_estate->lat;
            m_ref.lon = m_estate->lon;
            WGS84::displace(m_estate->x, m_estate->y, &(m_ref.lat), &(m_ref.lon));
            desZ.value = 0;
            desZ.z_units = Z_DEPTH;
            m_exec_state = SURFACE;
          }
          else if ((m_fref_state->proximity & FollowRefState::PROX_Z_NEAR) != 0)
          {
            desZ.value = m_maxz;
            desZ.z_units = Z_DEPTH;
            m_exec_state = YOYO_DOWN;
          }
          break;

        case SURFACE:
          if ((m_fref_state->proximity & FollowRefState::PROX_Z_NEAR) != 0)
          {
            m_ref = m_original_ref;
            desZ.value = m_maxz;
            desZ.z_units = Z_DEPTH;
            m_exec_state = YOYO_DOWN;
          }
          break;
        case END:
          active = false;
          break;
      }
      m_ref.z.set(&desZ);

      return m_ref;
    }

    YoYoController::~YoYoController()
    {
      // TODO Auto-generated destructor stub
    }

  } /* namespace LSTS */
} /* namespace TREX */
