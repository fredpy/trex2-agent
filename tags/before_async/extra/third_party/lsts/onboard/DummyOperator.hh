/*
 * DummyOperator.hh
 *
 *  Created on: Apr 29 2013
 *      Author: zp
 */

#ifndef DUMMYOP_HH_
#define DUMMYOP_HH_

# include <trex/transaction/reactor.hh>
# include <DUNE/DUNE.hpp>
# include"SharedEnvironment.hh"

namespace TREX {
  /** @brief lsts plug-in
   *
   * This namespace is shared by all LSTS reactors/plugins
   * @ingroup lsts
   *
   * @author Jose Pinto <zepinto@gmail.com>
   */
  namespace LSTS {

    class DummyOperator :public TREX::transaction::reactor {
    public:
      DummyOperator(TREX::transaction::reactor::xml_arg_type arg);
      virtual ~DummyOperator();

    private:
      TREX::utils::singleton::use<SharedEnvironment> m_env;
      void notify(TREX::transaction::token const &obs);
      bool synchronize() {return true;}
    };
  }
}

#endif /* DUMMYOP_HH_ */
