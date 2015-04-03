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

#ifndef EXAMPLES_STAPLING_KEYTOQUITBEHAVIOR_H
#define EXAMPLES_STAPLING_KEYTOQUITBEHAVIOR_H

#include <SurgSim/Framework/Behavior.h>

#include <SurgSim/Devices/Keyboard/KeyCode.h>

#include <functional>

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
}

class KeyToQuitBehavior : public SurgSim::Framework::Behavior
{
public:

	typedef std::function<void(int)> CallbackType;

	/// Constructor
	KeyToQuitBehavior(const std::string& name);

	/// Destructor
	~KeyToQuitBehavior();

	virtual void update(double dt) override;

	virtual bool doInitialize() override;

	virtual bool doWakeUp() override;

	/// Set the input component from which pressed keys come.
	/// \param	inputComponent	The input component which contains the pressed key(s).
	void setInputComponent(std::shared_ptr<SurgSim::Framework::Component> inputComponent);

	/// Get the input component of this behavior
	/// \return The input component which sends signals to this behavior.
	std::shared_ptr<SurgSim::Input::InputComponent> getInputComponent() const;

	char getQuitKey() const;

	void setQuitKey(int val);

	void setCallback(CallbackType callback);

private:

	SurgSim::Device::KeyCode m_quitKey;

	/// Input component from which pressed keys come.
	std::shared_ptr<SurgSim::Input::InputComponent> m_inputComponent;

	CallbackType m_callback;
};

#endif
