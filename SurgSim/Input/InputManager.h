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

#ifndef SURGSIM_INPUT_INPUTMANAGER_H
#define SURGSIM_INPUT_INPUTMANAGER_H

#include <boost/thread/mutex.hpp>
#include <memory>
#include <unordered_map>
#include <vector>

#include "SurgSim/Framework/ComponentManager.h"

namespace SurgSim
{
namespace Input
{
class DeviceInterface;
class InputComponent;
class OutputComponent;

/// Manager to handle InputComponent and OutputComponent, SceneElement can add these to
/// get input from devices, or even write output to devices. The devices have to be added
/// to this class before components can be added to it.
class InputManager : public SurgSim::Framework::ComponentManager
{
public:
	InputManager();
	virtual ~InputManager();

	friend class InputManagerTest;

	/// Adds a device to the manager.
	/// \param	device	The device.
	/// \return	true if it succeeds, false if the device already exists in the manager.
	bool addDevice(std::shared_ptr<SurgSim::Input::DeviceInterface> device);

	/// Removes the device described by device.
	/// \param	device	The device.
	/// \return	true if it succeeds, false if the device is not in.
	bool removeDevice(std::shared_ptr<SurgSim::Input::DeviceInterface> device);

	int getType() const override;

private:
	bool doInitialize() override;
	bool doStartUp() override;
	bool doUpdate(double dt) override;

	/// Adds a component, this can be either input or output, it will call the appropriate
	/// function in the device. For an InputComonent this will succeed if the device name
	/// inside the component is known to the InputManager and if the component has not
	/// been added as an input yet. For an OutputComponent the call will fail if the device
	/// does not exist or the device has already been assigned an output.
	/// \param	component	The component.
	/// \return	true if it succeeds, it will fail if the device cannot be found to the component
	/// 		has already been added to the manager, and return false.
	bool executeAdditions(const std::shared_ptr<SurgSim::Framework::Component>& component) override;

	/// Removes the component described by component.
	/// \param	component	The component.
	/// \return	true if it succeeds, it will fail if the component cannot be found and return false.
	bool executeRemovals(const std::shared_ptr<SurgSim::Framework::Component>& component) override;


	/// Specific call for input components.
	/// Link input consumer to input device
	/// Data produced by device will then be consumed by input consumer
	bool addInputComponent(const std::shared_ptr<InputComponent>& input);
	/// Specific call for output components.
	bool addOutputComponent(const std::shared_ptr<OutputComponent>& output);

	/// Collection of all input components.
	std::vector<std::shared_ptr<InputComponent>> m_inputs;
	/// Collection of all output components.
	std::vector<std::shared_ptr<OutputComponent>> m_outputs;

	/// Collection of all devices that have been added to the input manager
	/// key is the name, no two devices with the same name can be added to the
	/// input manager
	std::unordered_map<std::string, std::shared_ptr<SurgSim::Input::DeviceInterface>> m_devices;

	/// Protect critical sections
	boost::mutex m_mutex;
};

}; //namespace Input
}; //namespace SurgSim
#endif
