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

#ifndef EXAMPLES_ADDSPHEREFROMINPUT_ADDSPHEREBEHAVIOR_H
#define EXAMPLES_ADDSPHEREFROMINPUT_ADDSPHEREBEHAVIOR_H

#include "SurgSim/Framework/Behavior.h"
#include "SurgSim/Input/InputComponent.h"

class AddSphereFromInputBehavior: public SurgSim::Framework::Behavior
{
public:
	/// Constructor
	/// \param	name	Name of the behavior
	explicit AddSphereFromInputBehavior(const std::string& name);

	/// Set input component to get the pose
	/// \param	sender	Input component to get the pose
	void setInputComponent(std::shared_ptr<SurgSim::Input::InputComponent> sender);

	/// Update the behavior
	/// \param dt	The length of time (seconds) between update calls.
	void update(double dt) override;

	/// Return the type of manager that should be responsible for this behavior
	int getTargetManagerType() const override;

protected:
	/// Initialize the behavior
	bool doInitialize() override;
	/// Wakeup the behavior
	bool doWakeUp() override;

private:
	/// Input component to get the pose
	std::shared_ptr<SurgSim::Input::InputComponent> m_from;

	/// Used to record the number of spheres added
	int m_numElements;

	/// Used to record if button was previously pressed
	bool m_buttonPreviouslyPressed;
};

#endif  // EXAMPLES_ADDSPHEREFROMINPUT_ADDSPHEREBEHAVIOR_H
