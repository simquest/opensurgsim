// This file is a part of the OpenSurgSim project.
// Copyright 2015, SimQuest Solutions Inc.
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

#ifndef SURGSIM_BLOCKS_KEYBOARDCALLBACKBEHAVIOR_H
#define SURGSIM_BLOCKS_KEYBOARDCALLBACKBEHAVIOR_H

#include <functional>

#include "SurgSim/Devices/Keyboard/KeyCode.h"
#include "SurgSim/Framework/Behavior.h"

namespace SurgSim
{
namespace Input
{
class InputComponent;
}

namespace Blocks
{
SURGSIM_STATIC_REGISTRATION(KeyboardCallbackBehavior);

/// This behavior will call the callback function registered when the registered key is pressed.
class KeyboardCallbackBehavior : public Framework::Behavior
{
public:
	typedef std::function<void()> CallbackType;

	/// Constructor
	/// \param	name	Name of the behavior
	explicit KeyboardCallbackBehavior(const std::string& name);

	SURGSIM_CLASSNAME(SurgSim::Blocks::KeyboardCallbackBehavior);

	/// Set the input component from which pressed key comes.
	/// \param	inputComponent	The input component which contains the pressed key(s).
	void setInputComponent(std::shared_ptr<Framework::Component> inputComponent);

	/// Get the input component of this behavior, from which the pressed key comes.
	/// \return The input component which sends key press to this behavior.
	std::shared_ptr<Input::InputComponent> getInputComponent() const;

	/// Register a key, so that when such key is pressed, this behavior will call the callback.
	/// \param key The controlling key.
	void registerKey(Devices::KeyCode key);

	/// Register a callback function.
	/// This function will be called when the registered key is pressed.
	/// \param func The callback function.
	void registerCallback(CallbackType func);

	void update(double dt) override;

protected:
	/// Set the controlling key.
	/// \param key The key to trigger the callback.
	/// \note This is used for serialization only.
	void setKey(int key);

	/// \return The key which will make this behavior to call the callback.
	/// \note This is used for serialization only.
	int getKey() const;

	bool doInitialize() override;

	bool doWakeUp() override;

private:
	/// Record if any key is pressed in last update() call.
	bool m_keyPressedLastUpdate;

	/// The registered key, when which is pressed, the registered callback in this behavior will be called.
	int m_actionKey;

	/// Callback function.
	CallbackType m_callback;

	/// Input component from which pressed key comes.
	std::shared_ptr<Input::InputComponent> m_inputComponent;
};

}; // namespace Blocks
}; // namespace SurgSim

#endif //SURGSIM_BLOCKS_KEYBOARDCALLBACKBEHAVIOR_H
