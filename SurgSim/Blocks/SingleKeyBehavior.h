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

#ifndef SURGSIM_BLOCKS_SINGLEKEYBEHAVIOR_H
#define SURGSIM_BLOCKS_SINGLEKEYBEHAVIOR_H

#include <memory>
#include <string>

#include "SurgSim/Blocks/KeyBehavior.h"

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
class SingleKeyBehavior : public KeyBehavior
{
public:
	/// Constructor
	explicit SingleKeyBehavior(const std::string& name);

	/// Destructor
	~SingleKeyBehavior();

	bool doWakeUp() override;

	/// Sets the current key value used to trigger this behavior
	/// \param val the key code to use to trigger this behavior
	void setKey(int val);

	/// \return the key code that triggers this behavior
	int getKey() const;

protected:
	void setDescription(const std::string& description);

	virtual void onKey() = 0;

	void onKeyDown(int actualKey) override;

	void onKeyUp(int actualKey) override;

	std::shared_ptr<Input::InputComponent> m_inputComponent;

	/// Registered key to trigger action
	int m_actionKey;

	std::string m_description;
};

}; // namespace Blocks
}; // namespace SurgSim

#endif // SURGSIM_BLOCKS_SINGLEKEYBEHAVIOR_H
