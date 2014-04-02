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

#ifndef SURGSIM_BLOCKS_KEYBOARDTOGGLESGRAPHICSBEHAVIOR_H
#define SURGSIM_BLOCKS_KEYBOARDTOGGLESGRAPHICSBEHAVIOR_H

#include <unordered_set>

#include "SurgSim/Devices/Keyboard/KeyCode.h"
#include "SurgSim/Framework/Behavior.h"
#include "SurgSim/Graphics/Representation.h"

namespace SurgSim
{

namespace Framework
{
class Component;
}

namespace Input
{
class InputComponent;
}

namespace Blocks
{

/// This behavior is used to turn on and off the visibility of registered graphical representation(s)
/// when the corresponding registered key is pressed.
class KeyboardTogglesGraphicsBehavior : public SurgSim::Framework::Behavior
{
public:
	/// Constructor
	/// \param	name	Name of the behavior
	explicit KeyboardTogglesGraphicsBehavior(const std::string& name);

	/// Set the input component from which pressed keys come.
	/// \param	inputComponent	The input component which contains the pressed key(s).
	void setInputComponent(std::shared_ptr<SurgSim::Input::InputComponent> inputComponent);

	/// Register a key with a component in this behavior.
	/// \param key A key used to control the component.
	/// \param component The component being controlled by the key.
	/// \note A key can be registered several times, so does a component.
	void registerKey(SurgSim::Device::KeyCode key, std::shared_ptr<SurgSim::Framework::Component> component);

	/// Update the behavior
	/// \param dt	The length of time (seconds) between update calls.
	virtual void update(double dt) override;

protected:
	/// Initialize this behavior
	/// \return True on success, otherwise false.
	/// \note In current implementation, this method always returns "true".
	virtual bool doInitialize() override;

	/// Wakeup this behavior
	/// \return True on success, otherwise false.
	/// \note In current implementation, this method always returns "true".
	virtual bool doWakeUp() override;

private:
	/// Input component from which pressed keys come.
	std::shared_ptr<SurgSim::Input::InputComponent> m_inputComponent;

	/// A mapping between key and the graphical representation(s) it controls.
	std::unordered_map<int, std::unordered_set<std::shared_ptr<SurgSim::Graphics::Representation>>> m_register;

	/// Record if any key has been pressed.
	bool m_keyPressedLastUpdate;
};

}; // namespace Blocks

}; // namespace SurgSim


#endif //SURGSIM_BLOCKS_KEYBOARDTOGGLESGRAPHICSBEHAVIOR_H
