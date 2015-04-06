#ifndef SURGSIM_DEVICE_OCULUSDEVICE_H
#define SURGSIM_DEVICE_OCULUSDEVICE_H

#include <memory>
#include <string>

#include "SurgSim/Input/CommonDevice.h"

namespace SurgSim
{
namespace Device
{
class OculusScaffold;

/// A class implementing the communication with Oculus Rift DK2.
///
/// \par Application input provided by the device:
///   | type       | name              |                                        |
///   | ----       | ----              | ---                                    |
///   | pose       | "pose"            | %Device pose (units are meters).       |
///
/// \par Application output used by the device: none.
/// \note The axes of the pose of the HMD are a right-handed system: X & Z are in the plane of the floor,
///       with X pointing to the camera's left (i.e. to the HMD's right) and Z pointing towards the camera, 
///       while Y points up from the floor.
///       By default the tracking origin is located one meter away from the positional tracking camera in the direction
///       of the optical axis but with the same height as the camera.
/// \sa SurgSim::Input::CommonDevice, SurgSim::Input::DeviceInterface
class OculusDevice : public SurgSim::Input::CommonDevice
{
public:
	/// Constructor.
	/// \param name A unique name for the device.
	explicit OculusDevice(const std::string& name);

	/// Destructor.
	virtual ~OculusDevice();

	bool initialize() override;
	bool finalize() override;

	/// Check whether this device is initialized.
	/// \return True if this device is initialized; false otherwise.
	bool isInitialized() const;

private:
	friend class OculusScaffold;

	/// Communication with hardware is handled by scaffold.
	std::shared_ptr<OculusScaffold> m_scaffold;
};

}; // namespace Device
}; // namespace Burrhole

#endif  // SURGSIM_DEVICE_OCULUSDEVICE_H
