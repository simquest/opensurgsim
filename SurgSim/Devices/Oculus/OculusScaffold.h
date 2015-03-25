#ifndef SURGSIM_DEVICE_OCULUSSCAFFOLD_H
#define SURGSIM_DEVICE_OCULUSSCAFFOLD_H

#include <memory>

#include <SurgSim/Framework/BasicThread.h>

namespace SurgSim
{
namespace DataStructures
{
class DataGroup;
};

namespace Framework
{
class Logger;
}
};

namespace SurgSim
{

namespace Device
{
class OculusDevice;

/// A class that manages Oculus Rift DK2 devices.
///
/// \sa SurgSim::Device::OculusDevice
class OculusScaffold : SurgSim::Framework::BasicThread
{
public:
	/// Destructor.
	~OculusScaffold();

	/// Gets or creates the scaffold shared by all OculusDevice instances.
	/// The scaffold is managed using a SharedInstance object, so it will be destroyed when all devices are released.
	/// \return the scaffold object.
	static std::shared_ptr<OculusScaffold> getOrCreateSharedInstance();

protected:
	bool doInitialize() override;
	bool doStartUp() override;
	bool doUpdate(double dt) override;

private:
	/// Internal shared state data type.
	struct StateData;
	/// Internal per-device information.
	struct DeviceData;

	friend class OculusDevice;

	/// Constructor.
	OculusScaffold();

	/// Registers the specified device object.
	/// \param device The device object to be used, which should have a unique name.
	/// \return True if the initialization succeeds, false if it fails.
	bool registerDevice(OculusDevice* device);
	/// Unregisters the specified device object.
	/// \param device The device object.
	/// \return true on success, false on failure.
	bool unregisterDevice(const OculusDevice* device);

	/// Initializes Oculus SDK.
	/// \return true on success; false otherwise.
	bool initializeSdk();
	/// Finalizes (de-initializes) Oculus SDK.
	/// \return true on success; false otherwise.
	bool finalizeSdk();

	/// Builds the data layout for the application input (i.e. device output).
	static SurgSim::DataStructures::DataGroup buildDeviceInputData();

	/// Logger used by the scaffold and all devices.
	std::shared_ptr<SurgSim::Framework::Logger> m_logger;

	/// Internal scaffold state.
	std::unique_ptr<StateData> m_state;
};

};  // namespace Device
};  // namespace SurgSim 

#endif  // SURGSIM_DEVICE_OCULUSSCAFFOLD_H
