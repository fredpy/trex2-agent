/* -*- C++ -*- */
/** @file "Tick.hh"
 * @brief TREX time representation 
 *
 * This file defines all the utilities to manipulate
 * time as represented for TREX transaction.
 *
 * @ingroup transaction
 */
#ifndef H_Tick
# define H_Tick

# include <trex/domain/IntegerDomain.hh>

namespace TREX {
  namespace transaction {

    /** @brief TREX unit of time
     * @ingroup transaction
     */
    typedef IntegerDomain::base_type TICK;

  } // TREX::transaction
} // TREX

#endif // H_Tick
