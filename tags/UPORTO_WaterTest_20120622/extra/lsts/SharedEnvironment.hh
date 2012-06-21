/*
 * SharedEnvironment.hh
 *
 *  Created on: Jun 19, 2012
 *      Author: zp
 */

#ifndef SHAREDENVIRONMENT_HH_
#define SHAREDENVIRONMENT_HH_
#include <trex/utils/SingletonUse.hh>

namespace TREX {
	namespace LSTS {
		class Platform;
		class ControlInterface;
	}
}

class SharedEnvironment {
private:
	SharedEnvironment()
	{
		m_platform = NULL;
		m_controlInterface = NULL;
	}

	~SharedEnvironment()
	{
		// nothing
	}

	TREX::LSTS::Platform * m_platform;
	TREX::LSTS::ControlInterface * m_controlInterface;
public:

	TREX::LSTS::Platform * getPlatformReactor()
	{
		return m_platform;
	}

	void setPlatformReactor(TREX::LSTS::Platform * platf)
	{
		m_platform = platf;
	}

	TREX::LSTS::ControlInterface * getControlInterfaceReactor()
	{
		return m_controlInterface;
	}

	void setControlInterfaceReactor(TREX::LSTS::ControlInterface * citf)
	{
		m_controlInterface = citf;
	}

	friend class TREX::utils::SingletonWrapper<SharedEnvironment>;
};

#endif /* SHAREDENVIRONMENT_HH_ */
