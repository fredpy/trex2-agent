/*
 * LstsReactor.h
 *
 *  Created on: May 6, 2013
 *      Author: zp
 */

#ifndef TIMELINENORMALIZER_H_
#define TIMELINENORMALIZER_H_

# include <trex/transaction/reactor.hh>

using namespace TREX::transaction;

namespace TREX
{
  namespace LSTS
  {

    class LstsReactor : public reactor
    {
    public:
      LstsReactor(reactor::xml_arg_type arg);
      virtual ~LstsReactor();
      bool isObservationNew(TREX::transaction::token obs);
      bool postUniqueObservation(TREX::transaction::token obs);
    private:
      typedef std::map<std::string, SHARED_PTR<TREX::transaction::token> > obs_map;
      obs_map postedObservations;
    };

  } /* namespace LSTS */
} /* namespace TREX */
#endif /* TIMELINENORMALIZER_H_ */
