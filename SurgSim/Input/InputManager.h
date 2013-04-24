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

#ifndef HEMSIM_INPUTMANAGER_H
#define HEMSIM_INPUTMANAGER_H

#include <vector>
#include <unordered_map>
#include <memory>
#include <SurgSim/Framework/BasicThread.h>
#include <SurgSim/Input/InputComponent.h>
#include <SurgSim/Input/OutputComponent.h>
#include <SurgSim/Input/DeviceInterface.h>

#include <boost/thread/mutex.hpp>


namespace SurgSim
{
namespace Input
{

/// Manager to handle InputComponent and OutputComponent, SceneElement can add these to
/// get input from devices, or even write output to devices. The devices have to be added
/// to this class before components can be added to it.
class InputManager : public SurgSim::Framework::BasicThread
{
public:
	InputManager() ;
	virtual ~InputManager(void);

	/// Adds a component, this can be either input or output, it will call the appropriate
	/// function in the device. For an InputComonent this will suceed if the device name 
	/// inside the component is known to the InputManager and if the component has not 
	/// been added as an input yet. For an OutputComponent the call will fail if the device
	/// does not exist or 
	/// \param	component	The component.
	/// \return	true if it succeeds, it will fail if the device cannot be found to the component
	/// 		has already been added to the manager, and return false.
	virtual bool addComponent(std::shared_ptr<SurgSim::Framework::Component> component);

	/// Removes the component described by component.
	/// \param	component	The component.
	/// \return	true if it succeeds, it will fail if the component cannot be found and return false.
	virtual bool removeComponent(std::shared_ptr<SurgSim::Framework::Component> component);

	/// Adds a device to the manager.
	/// \param	device	The device.
	/// \return	true if it succeeds, false if the device already exists in the manager.
	bool addDevice(std::shared_ptr<SurgSim::Input::DeviceInterface> device);

	/// Removes the device described by device.
	/// \param	device	The device.
	/// \return	true if it succeeds, false if the device is not in.
	bool removeDevice(std::shared_ptr<SurgSim::Input::DeviceInterface> device);

private:
	virtual bool doInitialize();
	virtual bool doStartUp();
	virtual bool doUpdate(double dt);

	/// Specific call for input components.
	bool addInputComponent(std::shared_ptr<InputComponent> input);

	/// Specific call for output components.
	bool addOutputComponent(std::shared_ptr<OutputComponent> output);

	/// Collection of all input components.
	std::vector<std::shared_ptr<InputComponent>> m_inputs;

	/// Collection of all output components.
	std::vector<std::shared_ptr<OutputComponent>> m_outputs;

	/// Collection of all devices that have been added to the input manager
	/// key is the name, no two devices with the same name can be added to the
	/// input manager
	std::unordered_map<std::string, std::shared_ptr<SurgSim::Input::DeviceInterface>> m_devices;

	/// The logger for this manager
	std::shared_ptr<SurgSim::Framework::Logger> m_logger;

	/// Protect critical sections
	boost::mutex m_mutex;
};

}
}
#endif
