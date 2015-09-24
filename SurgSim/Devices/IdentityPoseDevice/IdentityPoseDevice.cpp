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

#include "SurgSim/Devices/IdentityPoseDevice/IdentityPoseDevice.h"

#include "SurgSim/DataStructures/DataGroup.h"
#include "SurgSim/DataStructures/DataGroupBuilder.h"
#include "SurgSim/Framework/Log.h"
#include "SurgSim/Math/RigidTransform.h"

using SurgSim::DataStructures::DataGroup;
using SurgSim::DataStructures::DataGroupBuilder;
using SurgSim::Math::RigidTransform3d;


namespace SurgSim
{
namespace Devices
{

SURGSIM_REGISTER(SurgSim::Input::DeviceInterface, SurgSim::Devices::IdentityPoseDevice, IdentityPoseDevice);

IdentityPoseDevice::IdentityPoseDevice(const std::string& uniqueName) :
	Input::CommonDevice(uniqueName, buildInputData())
{
}

bool IdentityPoseDevice::initialize()
{
	SURGSIM_LOG_INFO(Framework::Logger::getLogger("Devices/IdentityPose")) << "Device " << getName() << " initialized.";
	return true;
}

bool IdentityPoseDevice::isInitialized() const
{
	return true;
}

bool IdentityPoseDevice::finalize()
{
	SURGSIM_LOG_INFO(Framework::Logger::getLogger("Devices/IdentityPose")) << "Device " << getName() << " finalized.";
	return true;
}

DataGroup IdentityPoseDevice::buildInputData()
{
	DataGroupBuilder builder;
	builder.addPose(DataStructures::Names::POSE);
	return builder.createData();
}

bool IdentityPoseDevice::addInputConsumer(std::shared_ptr<Input::InputConsumerInterface> inputConsumer)
{
	if (!CommonDevice::addInputConsumer(std::move(inputConsumer)))
	{
		return false;
	}

	// The IdentityPoseDevice doesn't have any input events; it just sits there.
	// So we push the output to all the consumers, including the new one, right away after we add a consumer.
	// This ensures that all consumers always see the identity pose.
	getInputData().poses().set(DataStructures::Names::POSE, RigidTransform3d::Identity());
	pushInput();

	return true;
}


};  // namespace Devices
};  // namespace SurgSim
