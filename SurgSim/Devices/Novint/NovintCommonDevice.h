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

	virtual bool initialize() override;

	virtual bool finalize() override;

	/// Check whether this device is initialized.
	bool isInitialized() const;

private:
	friend class NovintScaffold;

	/// Query if this object represents a 7 degree of freedom hardware device.
	/// \return	true if 7 degree of freedom device, false if not.
	virtual bool is7DofDevice() const;


	std::shared_ptr<NovintScaffold> m_scaffold;
	std::string m_initializationName;
};

};  // namespace Device
};  // namespace SurgSim

#endif  // SURGSIM_DEVICES_NOVINT_NOVINTCOMMONDEVICE_H
