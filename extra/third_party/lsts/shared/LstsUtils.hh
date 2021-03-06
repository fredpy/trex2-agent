/*
 * LstsUtils.h
 *
 *  Created on: May 6, 2013
 *      Author: zp
 */

#ifndef LSTSUTILS_H_
#define LSTSUTILS_H_

namespace TREX
{
  namespace LSTS
  {

    class LstsUtils
    {
    public:
      static float normalizeDecPlaces(double coordinate, int decimalPlaces = 8);
      LstsUtils();
      virtual
      ~LstsUtils();
    };

  } /* namespace LSTS */
} /* namespace TREX */
#endif /* LSTSUTILS_H_ */
