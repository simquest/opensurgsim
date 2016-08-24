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

#ifndef SURGSIM_BLOCKS_KEYBEHAVIOR_H
#define SURGSIM_BLOCKS_KEYBEHAVIOR_H

#include <memory>
#include <string>

#include "SurgSim/Framework/Behavior.h"

namespace SurgSim
{
namespace Input
{
class InputComponent;
}

namespace Blocks
{

/// Behavior to abstract the functionality of keyboard driven behaviors, can be programmed to react to a
/// single keystroke
class KeyBehavior : public Framework::Behavior
{
public:
	/// Constructor
	explicit KeyBehavior(const std::string& name);

	/// Destructor
	~KeyBehavior();

	void update(double dt) override;
	bool doInitialize() override;
	bool doWakeUp() override;

	/// Set the input component from which pressed key comes.
	/// \param	inputComponent	The input component which contains the pressed key(s).
	void setInputComponent(std::shared_ptr<Framework::Component> inputComponent);

	/// Get the input component of this behavior, from which the pressed key comes.
	/// \return The input component which sends key press to this behavior.
	std::shared_ptr<Input::InputComponent> getInputComponent() const;

	/// Register the key, this let's the system keep track of keys used in the application
	/// \param keycode the key that is being used
	/// \param description description of functionality that the key triggers
	/// \return true if the key is available, false if another key has already been register
	static bool registerKey(int keycode, const std::string& description);

	/// Remove a key from the registry
	/// \param keycode the key to be removed
	/// \return true if the key was actually removed
	static bool unregisterKey(int keycode);

	/// Write the keymap out to the logger
	static void logMap();

protected:
	/// Implement to execute functionality on key press
	/// \param key the value of the key hit
	virtual void onKeyDown(int key) = 0;

	/// Implement to execute functionality on key release
	/// \param key the value of the key hit
	virtual void onKeyUp(int key) = 0;

	/// Input component needs to provide key
	std::shared_ptr<Input::InputComponent> m_inputComponent;

	/// Keep track if the key was pressed the last time around
	int m_lastKey;

	///@{
	/// Handle the key map
	static boost::mutex m_keyMapMutex;
	static std::unordered_map<int, std::string> m_keyMap;
	///@}
};

}; // namespace Blocks
}; // namespace SurgSim

#endif
