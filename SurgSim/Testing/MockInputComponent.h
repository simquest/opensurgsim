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

#ifndef SURGSIM_TESTING_MOCKINPUTCOMPONENT_H
#define SURGSIM_TESTING_MOCKINPUTCOMPONENT_H

#include "SurgSim/DataStructures/DataGroup.h"
#include "SurgSim/Input/InputComponent.h"

namespace SurgSim
{
namespace Testing
{

/// A MockInputComponent allows Components to test connections with devices without using an
/// instance of DeviceInterface (and then an InputManager to connect the device with the InputComponent, etc).
class MockInputComponent : public SurgSim::Input::InputComponent
{
public:
	/// Constructor
	/// \parameter name The name.
	MockInputComponent(const std::string& name);

	/// Set the data that this mock InputComponent will provide via getData.
	/// \param data The datagroup as if from a device.
	void setData(const SurgSim::DataStructures::DataGroup& data);

	void getData(SurgSim::DataStructures::DataGroup* dataGroup) override;

	/// The data provided via getData.
	SurgSim::DataStructures::DataGroup m_data;
};

};
};
#endif // SURGSIM_TESTING_MOCKINPUTCOMPONENT_H
