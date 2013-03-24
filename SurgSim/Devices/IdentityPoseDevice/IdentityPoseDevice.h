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

#ifndef SURGSIM_DEVICE_IDENTITY_POSE_DEVICE_H
#define SURGSIM_DEVICE_IDENTITY_POSE_DEVICE_H

#include <memory>
#include <string>

#include <SurgSim/Input/CommonDevice.h>

namespace SurgSim
{
namespace Device
{


/// A class implementing the identity pose device, which is a pretend device that doesn't move.
///
/// The identity pose device produces a pose that's always the identity transform (no translation from the origin
/// and no rotation from the model orientation).
/// This can be useful not only for writing tests, but also as a way to replace real hardware devices in situations
/// where the simulator needs to be run but the hardware is not currently available.
///
/// \sa SurgSim::Input::DeviceInterface
class IdentityPoseDevice : public SurgSim::Input::CommonDevice
{
public:
	/// Constructor.
	/// \param uniqueName A unique name for the device that will be used by the application.
	IdentityPoseDevice(const std::string& uniqueName);

	virtual bool addInputConsumer(std::shared_ptr<SurgSim::Input::InputConsumerInterface> inputConsumer);

protected:
	virtual bool initialize();

	virtual bool finalize();

	/// Builds the data layout for the application input (i.e. device output).
	static SurgSim::DataStructures::DataGroup buildInputData();
};


};  // namespace Device
};  // namespace SurgSim

#endif // SURGSIM_DEVICE_IDENTITY_POSE_DEVICE_H
