#include "SurgSim/Devices/Oculus/OculusDevice.h"

#include "SurgSim/Devices/Oculus/OculusScaffold.h"

namespace SurgSim 
{
namespace Device
{

OculusDevice::OculusDevice(const std::string& name) :
	SurgSim::Input::CommonDevice(name, OculusScaffold::buildDeviceInputData())
{
}

OculusDevice::~OculusDevice()
{
	if (isInitialized())
	{
		finalize();
	}
}


bool OculusDevice::initialize()
{
	SURGSIM_ASSERT(!isInitialized()) << getName() << " is already initialized, cannot initialize again.";

	m_scaffold = OculusScaffold::getOrCreateSharedInstance();
	SURGSIM_ASSERT(m_scaffold != nullptr) << "OculusDevice::initialize(): Failed to obtain an Oculus scaffold.";

	return m_scaffold->registerDevice(this);
}

bool OculusDevice::finalize()
{
	SURGSIM_ASSERT(isInitialized()) << getName() << " is not initialized, cannot finalized.";

	bool result = m_scaffold->unregisterDevice(this);
	m_scaffold.reset();

	return result;
}

bool OculusDevice::isInitialized() const
{
	return (m_scaffold != nullptr);
}

};  // namespace Device
};  // namespace SurgSim
