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

#include <boost/exception/to_string.hpp>

#include "Examples/ExampleStapling/StapleElement.h"
#include "Examples/ExampleStapling/StaplerBehavior.h"
#include "SurgSim/Collision/CollisionPair.h"
#include "SurgSim/Collision/Representation.h"
#include "SurgSim/DataStructures/DataGroup.h"
#include "SurgSim/Framework/Scene.h"
#include "SurgSim/Graphics/SceneryRepresentation.h"
#include "SurgSim/Input/InputComponent.h"
#include "SurgSim/Physics/Constraint.h"
#include "SurgSim/Physics/ConstraintComponent.h"
#include "SurgSim/Physics/Localization.h"
#include "SurgSim/Physics/RigidCollisionRepresentation.h"
#include "SurgSim/Physics/RigidRepresentationBilateral3D.h"

using SurgSim::Physics::ConstraintImplementation;
using SurgSim::Physics::RigidRepresentationBilateral3D;
using SurgSim::Physics::Localization;

StaplerBehavior::StaplerBehavior(const std::string& name):
	SurgSim::Framework::Behavior(name),
	m_numElements(0),
	m_buttonPreviouslyPressed(false)
{
}

void StaplerBehavior::setInputComponent(std::shared_ptr<SurgSim::Input::InputComponent> inputComponent)
{
	m_from = inputComponent;
}

void StaplerBehavior::setCollisionRepresentation(
	 std::shared_ptr<SurgSim::Collision::Representation> staplerRepresentation)
{
	m_collisionRepresentation = staplerRepresentation;
}

void StaplerBehavior::setVirtualStaple(std::shared_ptr<SurgSim::Collision::Representation> virtualTooth1,
									   std::shared_ptr<SurgSim::Collision::Representation> virtualTooth2)
{
	m_virtualTeeth[0] = virtualTooth1;
	m_virtualTeeth[1] = virtualTooth2;
}

static std::shared_ptr<SurgSim::Collision::Representation> findMostCollidedRepresentation(
	const std::shared_ptr<SurgSim::Collision::Representation>& object)
{
	typedef std::unordered_map<std::shared_ptr<SurgSim::Collision::Representation>,
							   std::list<std::shared_ptr<SurgSim::Collision::Contact>>> MapType;

	MapType collisionsMap = object->getCollisions();

	if (collisionsMap.empty())
	{
		return nullptr;
	}

	auto result = std::max_element(collisionsMap.begin(),
								   collisionsMap.end(),
								   [](const MapType::value_type& lhs, const MapType::value_type& rhs)
								   { return lhs.second.size() < rhs.second.size(); });

	return (*result).first;
}

static std::shared_ptr<SurgSim::Collision::Contact> findDeepestContact(
	const std::shared_ptr<SurgSim::Collision::Representation>& object,
	const std::shared_ptr<SurgSim::Collision::Representation>& intersectingObject)
{
	const std::list<std::shared_ptr<SurgSim::Collision::Contact>>& contacts
		= object->getCollisions().at(intersectingObject);

	if (contacts.empty())
	{
		return nullptr;
	}

	auto result = std::max_element(contacts.begin(),
								   contacts.end(),
								   [](const std::shared_ptr<SurgSim::Collision::Contact>& lhs,
									  const std::shared_ptr<SurgSim::Collision::Contact>& rhs)
								   { return lhs->depth < rhs->depth; });

	return *result;
}

void StaplerBehavior::update(double dt)
{
	SurgSim::DataStructures::DataGroup dataGroup;
	m_from->getData(&dataGroup);

	// Check if the stapler is being pressed.
	bool button1 = false;
	dataGroup.booleans().get("button1", &button1);

	bool processButtonPush = button1 && m_buttonPreviouslyPressed;
	m_buttonPreviouslyPressed = button1;

	if (!processButtonPush)
	{
		return;
	}

	// Create the staple (not added to the scene right now).
	std::string stapleName = "stapleId_" + boost::to_string(m_numElements++);
	auto staple = std::make_shared<StapleElement>(stapleName);
	staple->setPose(m_collisionRepresentation->getPose());

	int toothId = 0;
	bool stapleAdded = false;
	for (auto virtualTooth = m_virtualTeeth.begin(); virtualTooth != m_virtualTeeth.end(); ++virtualTooth)
	{
		if (*virtualTooth == nullptr)
		{
			continue;
		}

		// The virtual tooth could be in contact with any number of objects in the scene.
		// Find the object it has most collision pairs with.
		std::shared_ptr<SurgSim::Collision::Representation> targetRepresentation
			= findMostCollidedRepresentation(*virtualTooth);

		if (targetRepresentation == nullptr)
		{
			continue;
		}

		// Iterate through the list of collision pairs to find a point of constraint with the deepest contact.
		std::shared_ptr<SurgSim::Collision::Contact> targetContact
			= findDeepestContact(*virtualTooth, targetRepresentation);

		if (targetContact == nullptr)
		{
			continue;
		}

		// Get the physics representation of the targetRepresentation.
		// Done by dynamic type casting.
		std::shared_ptr<SurgSim::Physics::RigidCollisionRepresentation> rigidCollisionRepresentation = 
			std::dynamic_pointer_cast<SurgSim::Physics::RigidCollisionRepresentation>(targetRepresentation);

		if (rigidCollisionRepresentation == nullptr)
		{
			SURGSIM_LOG_WARNING(SurgSim::Framework::Logger::getDefaultLogger())
				<< "Failed to attach staple.  Collision only supported with RigidCollisionRepresentation.";
			continue;
		}

		std::shared_ptr<SurgSim::Physics::RigidRepresentationBase> physicsRepresentation =
			rigidCollisionRepresentation->getRigidRepresentation();

		if (physicsRepresentation == nullptr)
		{
			SURGSIM_LOG_WARNING(SurgSim::Framework::Logger::getDefaultLogger())
				<< "Failed to attach staple.  RigidCollisionRepresentation not attached to RigidRepresentationBase.";
			continue;
		}

		// Create the staple with no collision representation.
		if (!stapleAdded)
		{
			staple->setHasCollisionRepresentation(false);
			getScene()->addSceneElement(staple);
			stapleAdded = true;
		}

		// Create a bilateral constraint between the physicsRepresentation and staple.
		// First find the points where the constraint is going to be applied.
		std::shared_ptr<Localization> stapleLocalization
			= staple->getPhysicsRepresentation()->createLocalization(targetContact->penetrationPoints.first);
		stapleLocalization->setRepresentation(staple->getPhysicsRepresentation());

		std::shared_ptr<Localization> otherLocalization
			= physicsRepresentation->createLocalization(targetContact->penetrationPoints.second);
		otherLocalization->setRepresentation(physicsRepresentation);

		// Create the Constraint.
		std::shared_ptr<SurgSim::Physics::Constraint> constraint =
			std::make_shared<SurgSim::Physics::Constraint>(
				std::make_shared<SurgSim::Physics::ConstraintData>(),
				std::make_shared<RigidRepresentationBilateral3D>(),
				stapleLocalization,
				std::make_shared<RigidRepresentationBilateral3D>(),
				otherLocalization);

		std::shared_ptr<SurgSim::Physics::ConstraintComponent> constraintComponent =
			std::make_shared<SurgSim::Physics::ConstraintComponent>(
				stapleName + "_bilateral_constraint_" + boost::to_string(toothId++));

		constraintComponent->setConstraint(constraint);
		staple->addComponent(constraintComponent);
	}

	if (!stapleAdded)
	{
		// Create the staple element.
		getScene()->addSceneElement(staple);
	}
}

int StaplerBehavior::getTargetManagerType() const
{
	return SurgSim::Framework::MANAGER_TYPE_INPUT;
}

bool StaplerBehavior::doInitialize()
{
	SURGSIM_ASSERT(m_from) << "StaplerBehavior: no InputComponent held.";
	return true;
}

bool StaplerBehavior::doWakeUp()
{
	return true;
}
