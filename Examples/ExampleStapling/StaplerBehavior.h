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

#ifndef EXAMPLES_EXAMPLESTAPLING_STAPLERBEHAVIOR_H
#define EXAMPLES_EXAMPLESTAPLING_STAPLERBEHAVIOR_H

#include <array>
#include <memory>
#include <string>

#include "SurgSim/Framework/Behavior.h"

namespace SurgSim
{
namespace Framework
{
class Representation;
}

namespace Graphics
{
class SceneryRepresentation;
}

namespace Collision
{
class Representation;
}

namespace Input
{
class InputComponent;
}

}

/// This behavior is used to add staples.
/// The stapler is controlled by an input device and when
/// the user pushes a button on the device, a stapler will be deployed from the stapler.
class StaplerBehavior: public SurgSim::Framework::Behavior
{
public:
	/// Constructor
	/// \param	name	Name of the behavior
	explicit StaplerBehavior(const std::string& name);

	/// Set the input component from which to get the pose
	/// \param	inputComponent	The input component which sends the pose.
	void setInputComponent(std::shared_ptr<SurgSim::Input::InputComponent> inputComponent);

	/// Set the representation of the stapler
	/// \param	staplerRepresentation The representation of a stapler
	void setRepresentation(std::shared_ptr<SurgSim::Framework::Representation> staplerRepresentation);

	/// Update the behavior
	/// \param dt	The length of time (seconds) between update calls.
	virtual void update(double dt) override;

	/// Return the type of manager that should be responsible for this behavior
	/// \return An integer indicating which manger should be responsible for this behavior.
	virtual int getTargetManagerType() const override;

	/// Sets the virtual teeth for the virtual staple
	/// \param virtualTooth1 collision representation for the virtual staple tooth
	/// \param virtualTooth2 collision representation for the virtual staple tooth
	void setVirtualStaple(std::shared_ptr<SurgSim::Collision::Representation> virtualTooth1,
						  std::shared_ptr<SurgSim::Collision::Representation> virtualTooth2);

protected:
	/// Initialize this behavior
	/// \return True on success, otherwise false.
	/// \note: In current implementation, this method always returns "true".
	virtual bool doInitialize() override;

	/// Wakeup this behavior
	/// \return True on success, otherwise false.
	/// \note: In current implementation, this method always returns "true".
	virtual bool doWakeUp() override;

private:
	/// Input component from which to get the pose.
	std::shared_ptr<SurgSim::Input::InputComponent> m_from;

	/// The representation of the stapler.
	std::shared_ptr<SurgSim::Framework::Representation> m_representation;

	/// The number of staples added
	int m_numElements;

	/// Used to record if a button was previously pressed
	bool m_buttonPreviouslyPressed;

	/// Contains the teeth for detecting collisions
	std::array<std::shared_ptr<SurgSim::Collision::Representation>, 2> m_virtualTeeth;
};

#endif  // EXAMPLES_EXAMPLESTAPLING_STAPLERBEHAVIOR_H
