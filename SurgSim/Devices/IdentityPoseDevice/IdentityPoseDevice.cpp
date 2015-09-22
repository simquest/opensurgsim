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

#include "SurgSim/Math/Vector.h"
#include "SurgSim/Math/Matrix.h"
#include "SurgSim/Math/RigidTransform.h"
#include "SurgSim/Framework/Log.h"
#include "SurgSim/DataStructures/DataGroup.h"
#include "SurgSim/DataStructures/DataGroupBuilder.h"

using SurgSim::Math::Vector3d;
using SurgSim::Math::Matrix44d;
using SurgSim::Math::Matrix33d;
using SurgSim::Math::RigidTransform3d;

using SurgSim::DataStructures::DataGroup;
using SurgSim::DataStructures::DataGroupBuilder;


namespace SurgSim
{
namespace Devices
{

SURGSIM_REGISTER(SurgSim::Input::DeviceInterface, SurgSim::Devices::IdentityPoseDevice, IdentityPoseDevice);

IdentityPoseDevice::IdentityPoseDevice(const std::string& uniqueName) :
	SurgSim::Input::CommonDevice(uniqueName, buildInputData())
{
}

bool IdentityPoseDevice::initialize()
{
	// required by the DeviceInterface API
	return true;
}

bool IdentityPoseDevice::finalize()
{
	// required by the DeviceInterface API
	return true;
}

DataGroup IdentityPoseDevice::buildInputData()
{
	DataGroupBuilder builder;
	builder.addPose(SurgSim::DataStructures::Names::POSE);
	builder.addBoolean(SurgSim::DataStructures::Names::BUTTON_0);
	return builder.createData();
}

bool IdentityPoseDevice::addInputConsumer(std::shared_ptr<SurgSim::Input::InputConsumerInterface> inputConsumer)
{
	if (! CommonDevice::addInputConsumer(std::move(inputConsumer)))
	{
		return false;
	}

	// The IdentityPoseDevice doesn't have any input events; it just sits there.
	// So we push the output to all the consumers, including the new one, right away after we add a consumer.
	// This ensures that all consumers always see the identity pose.
	getInputData().poses().set(SurgSim::DataStructures::Names::POSE, RigidTransform3d::Identity());
	getInputData().booleans().set(SurgSim::DataStructures::Names::BUTTON_0, false);
	pushInput();

	return true;
}


};  // namespace Devices
};  // namespace SurgSim
