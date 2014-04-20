/*
 * SafetyBug.hh
 *
 *  Created on: Jun 21, 2012
 *      Author: zp
 */

#ifndef SAFETYBUG_HH_
#define SAFETYBUG_HH_

# include <trex/transaction/TeleoReactor.hh>
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

class SafetyBug :public TREX::transaction::TeleoReactor {
public:
	SafetyBug(TREX::transaction::TeleoReactor::xml_arg_type arg);
	virtual ~SafetyBug();

private:
	TREX::utils::SingletonUse<SharedEnvironment> m_env;
	bool aborted;

	bool synchronize()
	{
		// nothing to do
		return !aborted;
	}
	void notify(TREX::transaction::Observation const &obs);
	void newPlanToken(TREX::transaction::goal_id const &t);

};
}
}

#endif /* SAFETYBUG_HH_ */
