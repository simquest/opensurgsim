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

#ifndef EXAMPLES_ADDSPHEREFROMINPUT_INPUTGRAPHICSBEHAVIOR_H
#define EXAMPLES_ADDSPHEREFROMINPUT_INPUTGRAPHICSBEHAVIOR_H

#include <SurgSim/Framework/Behavior.h>
#include <SurgSim/Framework/Representation.h>
#include <SurgSim/Input/InputComponent.h>

namespace SurgSim
{

namespace Input
{

class InputGraphicsBehavior: public SurgSim::Framework::Behavior
{
public:
	/// Constructor
	/// \param	name	Name of the behavior
	/// \param	from	Input component to get the pose
	/// \param	to	Representation to set the pose
	InputGraphicsBehavior(const std::string& name,
		std::shared_ptr<SurgSim::Input::InputComponent> from,
		std::shared_ptr<SurgSim::Framework::Representation> to);

	/// Update the behavior
	/// \param dt	The length of time (seconds) between update calls.
	virtual void update(double dt) override;

	/// Return the type of manager that should be responsible for this behavior
	virtual int getTargetManagerType() const override;

protected:
	/// Initialize the behavior
	virtual bool doInitialize() override;

	/// Wakeup the behavior, which copies the initial pose
	virtual bool doWakeUp() override;

private:
	/// Representation to get the pose
	std::shared_ptr<SurgSim::Input::InputComponent> m_from;
	/// Representation to set the pose
	std::shared_ptr<SurgSim::Framework::Representation> m_to;
};


};  // namespace Input

};  // namespace SurgSim

#endif  // EXAMPLES_ADDSPHEREFROMINPUT_INPUTGRAPHICSBEHAVIOR_H
