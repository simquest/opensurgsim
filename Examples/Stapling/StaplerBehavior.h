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

#ifndef EXAMPLES_STAPLING_STAPLERBEHAVIOR_H
#define EXAMPLES_STAPLING_STAPLERBEHAVIOR_H

#include <array>
#include <memory>
#include <string>
#include <unordered_map>

#include "SurgSim/Framework/Behavior.h"
#include "SurgSim/Framework/ObjectFactory.h"
#include "SurgSim/Collision/Representation.h"

namespace SurgSim
{

namespace DataStructures
{
struct Location;
}

namespace Framework
{
class Representation;
}

namespace Graphics
{
class SceneryRepresentation;
}

namespace Input
{
class InputComponent;
}

namespace Physics
{
class Constraint;
}

}

SURGSIM_STATIC_REGISTRATION(StaplerBehavior);

/// This behavior is used to add staples.
/// The stapler is controlled by an input device and when
/// the user pushes a button on the device, a stapler will be deployed from the stapler.
class StaplerBehavior: public SurgSim::Framework::Behavior
{
public:
	/// Constructor
	/// \param	name	Name of the behavior
	explicit StaplerBehavior(const std::string& name);

	SURGSIM_CLASSNAME(StaplerBehavior);

	/// Set the input component from which to get the pose
	/// \param	inputComponent	The input component which sends the pose.
	void setInputComponent(std::shared_ptr<SurgSim::Framework::Component> inputComponent);

	/// \return	The input component which sends the pose.
	std::shared_ptr<SurgSim::Input::InputComponent> getInputComponent();

	/// Set the representation of the stapler
	/// \param	staplerRepresentation The representation of a stapler
	void setRepresentation(std::shared_ptr<SurgSim::Framework::Component> staplerRepresentation);

	/// \return The representation of a stapler
	std::shared_ptr<SurgSim::Framework::Representation> getRepresentation();

	/// Update the behavior
	/// \param dt	The length of time (seconds) between update calls.
	void update(double dt) override;

	/// Return the type of manager that should be responsible for this behavior
	/// \return An integer indicating which manger should be responsible for this behavior.
	int getTargetManagerType() const override;

	/// Sets the virtual teeth for the virtual staple
	/// \param virtualTeeth Array of collision representations for the virtual stapler teeth.
	void setVirtualTeeth(const std::array<std::shared_ptr<SurgSim::Collision::Representation>, 2>& virtualTeeth);

	/// \return Array of collision representations for the virtual stapler teeth.
	const std::array<std::shared_ptr<SurgSim::Collision::Representation>, 2>& getVirtualTeeth();

	/// Add a scene element (name) for which stapling is enabled within this behaviour.
	/// \param sceneElementName The name of the scene element that this behaviour can staple.
	void enableStaplingForSceneElement(std::string sceneElementName);

	/// Sets the list of scene element names that this behaviour can staple
	/// \param stapleEnabledSceneElements List of scene element names that this behaviour can staple.
	void setStapleEnabledSceneElements(const std::list<std::string>& stapleEnabledSceneElements);

	/// \return List of scene element names that this behaviour can staple.
	const std::list<std::string>& getStapleEnabledSceneElements();

protected:
	/// Initialize this behavior
	/// \return True on success, otherwise false.
	/// \note: In current implementation, this method always returns "true".
	bool doInitialize() override;

	/// Wakeup this behavior
	/// \return True on success, otherwise false.
	/// \note: In current implementation, this method always returns "true".
	bool doWakeUp() override;

private:
	/// Given a collision map, remove entries whose representations are not part of
	/// enabled scene element lists.
	/// \param [in,out] collisionsMap The collision map to be filtered.
	void filterCollisionMapForStapleEnabledRepresentations(SurgSim::Collision::ContactMapType *collisionsMap);

	/// Given a Collision::Representation, get the corresponding Physics::Representation.
	/// \param collisionRepresentation shared_ptr to the collision representation.
	/// \return The shared_ptr of the Physics::Representation. Can be nullptr.
	std::shared_ptr<SurgSim::Physics::Representation> findCorrespondingPhysicsRepresentation(
		std::shared_ptr<SurgSim::Collision::Representation> collisionRepresentation);

	/// Given a collision map, remove entries whose representations are not supported to be stapled to.
	/// \param [in,out] collisionsMap The collision map to be filtered.
	void filterCollisionMapForSupportedRepresentationTypes(SurgSim::Collision::ContactMapType* collisionsMap);

	/// Function to create the staple element.
	/// \note This function also checks for collision with stapling enabled objects in the scene to create
	/// bilateral constraint between the staple element and the object.
	void createStaple();

	/// Input component from which to get the pose.
	std::shared_ptr<SurgSim::Input::InputComponent> m_from;

	/// The representation of the stapler.
	std::shared_ptr<SurgSim::Framework::Representation> m_representation;

	/// The number of staples added
	int m_numElements;

	/// The NamedData index for the button1.
	int m_button1Index;

	/// Flag for caching the the NamedData button1Index.
	bool m_button1IndexCached;

	/// Used to record if a button was previously pressed
	bool m_buttonPreviouslyPressed;

	/// Contains the teeth for detecting collisions
	std::array<std::shared_ptr<SurgSim::Collision::Representation>, 2> m_virtualTeeth;

	/// The list of scene element names that this behavior can staple.
	std::list<std::string> m_stapleEnabledSceneElements;
};

#endif  // EXAMPLES_STAPLING_STAPLERBEHAVIOR_H
