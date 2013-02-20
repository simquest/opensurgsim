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

#ifndef SURGSIM_DEVICE_NULL_DEVICE_H
#define SURGSIM_DEVICE_NULL_DEVICE_H

#include <memory>
#include <string>

#include <SurgSim/Input/CommonInputDevice.h>

namespace SurgSim
{
namespace Device
{

class PhantomManager;

/// A class implementing the "null" device, i.e. a device that doesn't move.
///
/// \sa SurgSim::Input::InputDeviceInterface
class NullDevice : public SurgSim::Input::CommonInputDevice
{
public:
	/// Constructor.
	/// \param uniqueName A unique name for the device that will be used by the application.
	NullDevice(const std::string& uniqueName);
	
	virtual bool addListener(std::shared_ptr<SurgSim::Input::InputDeviceListenerInterface> listener);

	virtual bool addInputListener(std::shared_ptr<SurgSim::Input::InputDeviceListenerInterface> listener);

protected:
	virtual bool initialize();

	virtual bool finalize();

	/// Builds the data layout for the application input (i.e. device output).
	static SurgSim::Input::DataGroup buildInputData();
};

};  // namespace Device
};  // namespace SurgSim

#endif // SURGSIM_DEVICE_NULL_DEVICE_H
