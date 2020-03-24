// This file is a part of the OpenSurgSim project.
// Copyright 2013-2016, SimQuest Solutions Inc.
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

#ifndef SURGSIM_INPUT_INPUTCOMPONENT_H
#define SURGSIM_INPUT_INPUTCOMPONENT_H

#include <atomic>
#include <memory>
#include <string>

#include "SurgSim/DataStructures/DataGroup.h"
#include "SurgSim/Framework/LockedContainer.h"
#include "SurgSim/Framework/Representation.h"
#include "SurgSim/Input/InputConsumerInterface.h"

namespace SurgSim
{
namespace Input
{
class DeviceInterface;

SURGSIM_STATIC_REGISTRATION(InputComponent);

/// InputComponents connect devices to SceneElements, facilitating data transfer
/// from a device to SceneElements and other Components.
class InputComponent : public SurgSim::Framework::Representation, public InputConsumerInterface
{
public:
	/// Constructor
	/// \param name Name of this input component
	explicit InputComponent(const std::string& name);

	/// Destructor
	virtual ~InputComponent();

	SURGSIM_CLASSNAME(SurgSim::Input::InputComponent);

	/// Set name of the device this input component connects to.
	/// \param deviceName Name of the device this input component connects
	void setDeviceName(const std::string& deviceName);

	/// Gets device name.
	/// \return	The device name.
	std::string getDeviceName() const;

	/// Gets the input data.
	/// \param [out] dataGroup The location to write the data.  The pointer must be non-null.
	/// \exception Asserts if the InputComponent is not connected to a device.
	void getData(SurgSim::DataStructures::DataGroup* dataGroup);

	bool doInitialize() override;

	bool doWakeUp() override;

	void initializeInput(const std::string& device, const SurgSim::DataStructures::DataGroup& initialData) override;

	void handleInput(const std::string& device, const SurgSim::DataStructures::DataGroup& inputData) override;

	SurgSim::Math::RigidTransform3d getToDeviceTransform() const;
	void setToDeviceTransform(const SurgSim::Math::RigidTransform3d& val);

	
	SurgSim::Math::RigidTransform3d getToElementTransform() const;
	void setToElementTransform(const SurgSim::Math::RigidTransform3d& val);

private:
	/// Name of the device to which this input component connects
	std::string m_deviceName;

	/// Thread safe container of most recent input data
	SurgSim::Framework::LockedContainer<SurgSim::DataStructures::DataGroup> m_lastInput;

	SurgSim::Math::RigidTransform3d m_toElementTransform;

	std::atomic<bool> m_hasInput;
};

}; // namespace Input
}; // namespace SurgSim


#endif
