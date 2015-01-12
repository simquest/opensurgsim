// This file is a part of the OpenSurgSim project.
// Copyright 2013, SimQuest Solutions Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef SURGSIM_DEVICES_NOVINT_NOVINTCOMMONDEVICE_H
#define SURGSIM_DEVICES_NOVINT_NOVINTCOMMONDEVICE_H

#include <memory>
#include <string>

#include "SurgSim/Input/CommonDevice.h"

namespace SurgSim
{
namespace Device
{
class NovintScaffold;


/// A class implementing the communication with a generic Novint Falcon device.
///
/// \sa NovintDevice, Novint7DofHapticDevice
/// \sa SurgSim::Input::CommonDevice, SurgSim::Input::DeviceInterface
class NovintCommonDevice : public SurgSim::Input::CommonDevice
{
public:
	/// Constructor.
	///
	/// \param uniqueName A unique name for the device that will be used by the application.
	/// \param initializationName The name passed to HDAL when initializing the device.  This should match a
	/// 	configured Novint device; alternately, an empty string indicates the default device.
	NovintCommonDevice(const std::string& uniqueName, const std::string& initializationName);

	/// Destructor.
	virtual ~NovintCommonDevice();

	/// Gets the name used by the Novint device configuration to refer to this device.
	/// Note that this may or may not be the same as the device name retrieved by getName().
	/// An empty string indicates the default device.
	/// \return	The initialization name.
	std::string getInitializationName() const;

	bool initialize() override;

	bool finalize() override;

	/// Check whether this device is initialized.
	bool isInitialized() const;

	/// Sets the position scale for this device.
	/// The position scale controls how much the pose changes for a given device translation.
	/// The default value for a raw device tries to correspond to the actual physical motion of the device.
	/// \param scale The multiplicative factor to apply to the position.
	void setPositionScale(double scale);
	/// Gets the position scale for this device.
	double getPositionScale() const;

	/// Sets the orientation scale for this device.
	/// The orientation scale controls how much the pose changes for a given device rotation.
	/// The default value for a raw device tries to correspond to the actual physical motion of the device.
	/// \param scale The multiplicative factor to apply to the rotation angles.
	void setOrientationScale(double scale);
	/// Gets the orientation scale for this device.
	double getOrientationScale() const;

private:
	friend class NovintScaffold;

	/// Query if this object represents a 7 degree of freedom hardware device.
	/// \return	true if 7 degree of freedom device, false if not.
	virtual bool is7DofDevice() const;

	/// The scaffold handles all the communication with the SDK.
	NovintScaffold& m_scaffold;

	/// true if this device has been registered with the scaffold.
	bool m_initialized;

	/// The name passed to the SDK to specify which hardware device should be used.
	std::string m_initializationName;

	/// Scale factor for the position axes; stored locally before the device is initialized.
	double m_positionScale;
	/// Scale factor for the orientation axes; stored locally before the device is initialized.
	double m_orientationScale;
};

};  // namespace Device
};  // namespace SurgSim

#endif  // SURGSIM_DEVICES_NOVINT_NOVINTCOMMONDEVICE_H
