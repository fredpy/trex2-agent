/*
 * SafetyBug.hh
 *
 *  Created on: Jun 21, 2012
 *      Author: zp
 */

#ifndef SAFETYBUG_HH_
#define SAFETYBUG_HH_

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

class SafetyBug :public TREX::transaction::reactor {
public:
	SafetyBug(TREX::transaction::reactor::xml_arg_type arg);
	virtual ~SafetyBug();

private:
  TREX::utils::singleton::use<SharedEnvironment> m_env;
	bool aborted;

	bool synchronize()
	{
		// nothing to do
		return !aborted;
	}
	void notify(TREX::transaction::Observation const &obs);
	void new_plan_token(TREX::transaction::goal_id const &t);

};
}
}

#endif /* SAFETYBUG_HH_ */
