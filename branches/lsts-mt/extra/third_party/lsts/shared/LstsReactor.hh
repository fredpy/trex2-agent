/*
 * LstsReactor.h
 *
 *  Created on: May 6, 2013
 *      Author: zp
 */

#ifndef TIMELINENORMALIZER_H_
#define TIMELINENORMALIZER_H_

# include <trex/transaction/TeleoReactor.hh>

using namespace TREX::transaction;

namespace TREX
{
  namespace LSTS
  {

    class LstsReactor : public TeleoReactor
    {
    public:
      LstsReactor(TeleoReactor::xml_arg_type arg);
      virtual ~LstsReactor();
      bool isObservationNew(TREX::transaction::Observation obs);
      bool postUniqueObservation(TREX::transaction::Observation obs);
    private:
      typedef std::map<std::string, boost::shared_ptr<TREX::transaction::Observation> > obs_map;
      obs_map postedObservations;
    };

  } /* namespace LSTS */
} /* namespace TREX */
#endif /* TIMELINENORMALIZER_H_ */
