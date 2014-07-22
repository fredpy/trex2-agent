/*
 * SharedEnvironment.hh
 *
 *  Created on: Jun 19, 2012
 *      Author: zp
 */

#ifndef SHAREDENVIRONMENT_HH_
#define SHAREDENVIRONMENT_HH_
#include <trex/utils/singleton.hh>

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
            if( NULL!=platf && NULL!=m_platform )
              throw std::runtime_error("Attempt to have more than one platform.");
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

  friend class TREX::utils::singleton::wrapper<SharedEnvironment>;
};

#endif /* SHAREDENVIRONMENT_HH_ */
