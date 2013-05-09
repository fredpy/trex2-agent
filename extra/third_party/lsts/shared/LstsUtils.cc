/*
 * LstsUtils.cpp
 *
 *  Created on: May 6, 2013
 *      Author: zp
 */

# include "LstsUtils.hh"
# include <cmath>

namespace TREX
{
  namespace LSTS
  {

    LstsUtils::LstsUtils()
    {
      // TODO Auto-generated constructor stub

    }

    float LstsUtils::normalizeDecPlaces(double coordinate, int decimalPlaces)
    {
      double factor = pow(10, decimalPlaces);
      return std::floor(coordinate * factor) / factor;
    }

    LstsUtils::~LstsUtils()
    {
      // TODO Auto-generated destructor stub
    }
  } /* namespace LSTS */
} /* namespace TREX */
