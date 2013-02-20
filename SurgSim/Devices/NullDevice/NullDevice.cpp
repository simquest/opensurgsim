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

#include "SurgSim/Devices/NullDevice/NullDevice.h"

#include <iostream>
#include <iomanip>

#include <HD/hd.h>

#include <SurgSim/Math/Vector.h>
#include <SurgSim/Math/Matrix.h>
#include <SurgSim/Math/RigidTransform.h>
#include <SurgSim/Framework/Log.h>
#include <SurgSim/Input/DataGroup.h>
#include <SurgSim/Input/DataGroupBuilder.h>

using SurgSim::Math::Vector3d;
using SurgSim::Math::Matrix44d;
using SurgSim::Math::Matrix33d;
using SurgSim::Math::RigidTransform3d;

using SurgSim::Input::DataGroup;
using SurgSim::Input::DataGroupBuilder;


namespace SurgSim
{
namespace Device
{


NullDevice::NullDevice(const std::string& uniqueName) :
	SurgSim::Input::CommonInputDevice(uniqueName, buildInputData())
{
}

bool NullDevice::initialize()
{
	return true;
}

bool NullDevice::finalize()
{
	return true;
}

DataGroup NullDevice::buildInputData()
{
	DataGroupBuilder builder;
	builder.addPose("pose");
	builder.addBoolean("button0");
	builder.addBoolean("button1");
	builder.addBoolean("button2");
	builder.addBoolean("button3");
	return builder.createData();
}

bool NullDevice::addListener(std::shared_ptr<SurgSim::Input::InputDeviceListenerInterface> listener)
{
	bool status = CommonInputDevice::addListener(std::move(listener));

	// The NullDevice doesn't have any input events; it just sits there.
	// So we push the output to all the listeners, including the new one, right away after we add it.
	getInputData().poses().put("pose", RigidTransform3d::Identity());
	getInputData().booleans().put("button0", false);
	pushInput();

	return status;
}

bool NullDevice::addInputListener(std::shared_ptr<SurgSim::Input::InputDeviceListenerInterface> listener)
{
	bool status = CommonInputDevice::addInputListener(std::move(listener));

	// The NullDevice doesn't have any input events; it just sits there.
	// So we push the output to all the listeners, including the new one, right away after we add it.
	getInputData().poses().put("pose", RigidTransform3d::Identity());
	getInputData().booleans().put("button0", false);
	pushInput();

	return status;
}

};  // namespace Device
};  // namespace SurgSim
