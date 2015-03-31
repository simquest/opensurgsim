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

#include "Examples/Stapling/StaplerBehavior.h"

#include <boost/exception/to_string.hpp>

#include "Examples/Stapling/StapleElement.h"
#include "SurgSim/Collision/CollisionPair.h"
#include "SurgSim/Collision/Representation.h"
#include "SurgSim/DataStructures/DataStructuresConvert.h"
#include "SurgSim/DataStructures/DataGroup.h"
#include "SurgSim/DataStructures/Location.h"
#include "SurgSim/Framework/FrameworkConvert.h"
#include "SurgSim/Framework/Log.h"
#include "SurgSim/Framework/Scene.h"
#include "SurgSim/Framework/SceneElement.h"
#include "SurgSim/Graphics/SceneryRepresentation.h"
#include "SurgSim/Graphics/Model.h"
#include "SurgSim/Math/MlcpConstraintType.h"
#include "SurgSim/Input/InputComponent.h"
#include "SurgSim/Physics/Constraint.h"
#include "SurgSim/Physics/ConstraintComponent.h"
#include "SurgSim/Physics/ConstraintDataRotationVector.h"
#include "SurgSim/Physics/ConstraintDataFem3DDistancePoints.h"
#include "SurgSim/Physics/ConstraintImplementation.h"
#include "SurgSim/Physics/DeformableCollisionRepresentation.h"
#include "SurgSim/Physics/DeformableRepresentation.h"
#include "SurgSim/Physics/FixedRepresentationBilateral3D.h"
#include "SurgSim/Physics/Fem3DRepresentationBilateral3D.h"
#include "SurgSim/Physics/Fem3DRepresentationConstraint2Fem3DPoints.h"
#include "SurgSim/Physics/Fem3DRepresentation.h"
#include "SurgSim/Physics/FemElement.h"
#include "SurgSim/Physics/Localization.h"
#include "SurgSim/Physics/RigidCollisionRepresentation.h"
#include "SurgSim/Physics/RigidRepresentation.h"
#include "SurgSim/Physics/RigidRepresentationBilateral3D.h"
#include "SurgSim/Physics/RigidRepresentationConstraintRotationVector.h"

using SurgSim::Collision::ContactMapType;
using SurgSim::Physics::ConstraintDataFem3DDistancePoints;
using SurgSim::Physics::ConstraintImplementation;
using SurgSim::Physics::FixedRepresentationBilateral3D;
using SurgSim::Physics::RigidRepresentationBilateral3D;
using SurgSim::Physics::RigidRepresentationConstraintRotationVector;
using SurgSim::Physics::Fem3DRepresentation;
using SurgSim::Physics::Fem3DRepresentationBilateral3D;
using SurgSim::Physics::Fem3DRepresentationConstraint2Fem3DPoints;
using SurgSim::Physics::FemElement;
using SurgSim::Physics::Localization;
using SurgSim::Framework::checkAndConvert;

SURGSIM_REGISTER(SurgSim::Framework::Component, StaplerBehavior, StaplerBehavior);

StaplerBehavior::StaplerBehavior(const std::string& name):
	SurgSim::Framework::Behavior(name),
	m_numElements(0),
	m_button1Index(-1),
	m_button1IndexCached(false),
	m_buttonPreviouslyPressed(false)
{
	typedef std::array<std::shared_ptr<SurgSim::Collision::Representation>, 2> VirtualTeethArray;
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(StaplerBehavior, std::shared_ptr<SurgSim::Framework::Component>,
									  InputComponent, getInputComponent, setInputComponent);
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(StaplerBehavior, std::shared_ptr<SurgSim::Framework::Component>,
									  Representation, getRepresentation, setRepresentation);
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(StaplerBehavior, VirtualTeethArray, VirtualTeeth,
									  getVirtualTeeth, setVirtualTeeth);
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(StaplerBehavior, std::list<std::string>, StapleEnabledSceneElements,
									  getStapleEnabledSceneElements, setStapleEnabledSceneElements);
}

void StaplerBehavior::setInputComponent(std::shared_ptr<SurgSim::Framework::Component> inputComponent)
{
	SURGSIM_ASSERT(nullptr != inputComponent) << "'inputComponent' cannot be set to 'nullptr'";
	m_from = checkAndConvert<SurgSim::Input::InputComponent>(inputComponent, "SurgSim::Input::InputComponent");
}

std::shared_ptr<SurgSim::Input::InputComponent> StaplerBehavior::getInputComponent()
{
	return m_from;
}

void StaplerBehavior::setRepresentation(std::shared_ptr<SurgSim::Framework::Component> staplerRepresentation)
{
	SURGSIM_ASSERT(nullptr != staplerRepresentation) << "'staplerRepresentation' cannot be set to 'nullptr'";
	m_representation = checkAndConvert<SurgSim::Framework::Representation>(
						   staplerRepresentation, "SurgSim::Framework::Representation");
}

std::shared_ptr<SurgSim::Framework::Representation> StaplerBehavior::getRepresentation()
{
	return m_representation;
}

void StaplerBehavior::setVirtualTeeth(
	const std::array<std::shared_ptr<SurgSim::Collision::Representation>, 2>& virtualTeeth)
{
	m_virtualTeeth = virtualTeeth;
}

const std::array<std::shared_ptr<SurgSim::Collision::Representation>, 2>& StaplerBehavior::getVirtualTeeth()
{
	return m_virtualTeeth;
}

void StaplerBehavior::enableStaplingForSceneElement(std::string sceneElementName)
{
	m_stapleEnabledSceneElements.push_back(sceneElementName);
}

void StaplerBehavior::setStapleEnabledSceneElements(const std::list<std::string>& stapleEnabledSceneElements)
{
	m_stapleEnabledSceneElements = stapleEnabledSceneElements;
}

const std::list<std::string>& StaplerBehavior::getStapleEnabledSceneElements()
{
	return m_stapleEnabledSceneElements;
}

void StaplerBehavior::filterCollisionMapForStapleEnabledRepresentations(ContactMapType* collisionsMap)
{
	for (auto it = collisionsMap->begin(); it != collisionsMap->end();)
	{
		if (std::find(m_stapleEnabledSceneElements.begin(),
					  m_stapleEnabledSceneElements.end(),
					  (*it).first->getSceneElement()->getName()) == m_stapleEnabledSceneElements.end())
		{
			// Representation's scene element is not in the m_stapleEnabledSceneElements.
			it = collisionsMap->erase(it);
		}
		else
		{
			++it;
		}
	}
}

std::shared_ptr<SurgSim::Physics::Representation> StaplerBehavior::findCorrespondingPhysicsRepresentation(
	std::shared_ptr<SurgSim::Collision::Representation> collisionRepresentation)
{
	std::shared_ptr<SurgSim::Physics::Representation> physicsRepresentation = nullptr;

	// Check if the collisionRepresenation is for a Rigid body.
	std::shared_ptr<SurgSim::Physics::RigidCollisionRepresentation> rigidCollisionRepresentation =
		std::dynamic_pointer_cast<SurgSim::Physics::RigidCollisionRepresentation>(collisionRepresentation);

	if (rigidCollisionRepresentation != nullptr)
	{
		physicsRepresentation = rigidCollisionRepresentation->getRigidRepresentation();
	}
	else
	{
		// Check if the collisionRepresenation is for a deformable body.
		std::shared_ptr<SurgSim::Physics::DeformableCollisionRepresentation> deformableCollisionRepresentation =
			std::dynamic_pointer_cast<SurgSim::Physics::DeformableCollisionRepresentation>(collisionRepresentation);

		if (deformableCollisionRepresentation != nullptr)
		{
			physicsRepresentation = deformableCollisionRepresentation->getDeformableRepresentation();
		}
	}

	return physicsRepresentation;
}

void StaplerBehavior::filterCollisionMapForSupportedRepresentationTypes(ContactMapType* collisionsMap)
{
	for (auto it = collisionsMap->begin(); it != collisionsMap->end();)
	{
		if (findCorrespondingPhysicsRepresentation((*it).first) == nullptr)
		{
			// Representation type is not supported to be stapled.
			it = collisionsMap->erase(it);
		}
		else
		{
			++it;
		}
	}
}

void StaplerBehavior::createStaple()
{
	// Create the staple (not added to the scene right now).
	auto staple = std::make_shared<StapleElement>("staple_" + boost::to_string(m_numElements++));
	staple->setPose(m_representation->getPose());

	std::array<std::shared_ptr<SurgSim::Physics::Representation>, 2> reps;
	std::array<SurgSim::DataStructures::Location, 2> locations;

	std::shared_ptr<SurgSim::Physics::Representation> stapleRep;
	std::array<SurgSim::DataStructures::Location, 2> stapleLocations;

	int toothId = 0;
	bool stapleAdded = false;
	for (auto virtualTooth = m_virtualTeeth.cbegin(); virtualTooth != m_virtualTeeth.cend(); ++virtualTooth)
	{
		// The virtual tooth could be in contact with any number of objects in the scene.
		// Get its collisionMap.
		ContactMapType collisionsMap = *((*virtualTooth)->getCollisions().safeGet());

		// If the virtualTooth has no collision, continue to next loop iteration.
		if (collisionsMap.empty())
		{
			continue;
		}

		// Remove representations from the collision map that are not enabled to be stapled.
		filterCollisionMapForStapleEnabledRepresentations(&collisionsMap);

		// If the collision map is emptied after filtering, continue to next loop iteration.
		if (collisionsMap.empty())
		{
			continue;
		}

		// Filter the map based on supported Physics::Represention types.
		filterCollisionMapForSupportedRepresentationTypes(&collisionsMap);

		// If the collision map is emptied after filtering, continue to next loop iteration.
		if (collisionsMap.empty())
		{
			continue;
		}

		// Find the row (representation, list of contacts) in the map that the virtualTooth has most
		// collision pairs with.
		ContactMapType::value_type targetRepresentationContacts
			= *std::max_element(collisionsMap.begin(), collisionsMap.end(),
								[](const ContactMapType::value_type & lhs, const ContactMapType::value_type & rhs)
		{
			return lhs.second.size() < rhs.second.size();
		});

		// Iterate through the list of collision pairs to find a contact with the deepest penetration.
		std::shared_ptr<SurgSim::Collision::Contact> targetContact
			= *std::max_element(targetRepresentationContacts.second.begin(), targetRepresentationContacts.second.end(),
								[](const std::shared_ptr<SurgSim::Collision::Contact>& lhs,
								   const std::shared_ptr<SurgSim::Collision::Contact>& rhs)
		{
			return lhs->depth < rhs->depth;
		});

		// Create the staple, before creating the constraint with the staple.
		// The staple is created with no collision representation, because it is going to be constrained.
		if (!stapleAdded)
		{
			staple->setHasCollisionRepresentation(false);
			getScene()->addSceneElement(staple);
			// The gravity of the staple is disabled to prevent it from rotating about the line
			// connecting the two points of constraints on the staple.
			staple->getComponents<SurgSim::Physics::Representation>()[0]->setIsGravityEnabled(false);
			stapleAdded = true;
		}

		// Find the corresponding Phsyics::Representation for the target Collision::Representation.
		// Note that the targetPhysicsRepresentation will NOT be a nullptr, because the
		// collisionsMap was filtered earlier to remove Representations that returned nullptr
		// when the function findCorrespondingPhysicsRepresentation was called.
		// (see filterCollisionMapForStapleEnabledRepresentations above).
		std::shared_ptr<SurgSim::Physics::Representation> targetPhysicsRepresentation =
			findCorrespondingPhysicsRepresentation(targetRepresentationContacts.first);

		// The constraint is created at the contact point in targetContact->penetrationPoints.second.
		// Convert this location to stapleRepresentation.
		auto stapleRepresentation = staple->getComponents<SurgSim::Physics::Representation>()[0];
		SurgSim::DataStructures::Location stapleConstraintLocation(SurgSim::Math::Vector3d(
					stapleRepresentation->getPose().inverse() * targetPhysicsRepresentation->getPose() *
					targetContact->penetrationPoints.second.rigidLocalPosition.getValue()));

		reps[toothId] = targetPhysicsRepresentation;
		locations[toothId] = targetContact->penetrationPoints.second;

		stapleRep = stapleRepresentation;
		stapleLocations[toothId] = stapleConstraintLocation;

		toothId++;
	}

	// If we have 2 collisions against the same Fem3DRepresentation, let's create a distance constraint
	if (toothId >= 2 && std::dynamic_pointer_cast<Fem3DRepresentation>(reps[0]) != nullptr && reps[0] == reps[1])
	{
		auto data = std::make_shared<SurgSim::Physics::ConstraintDataFem3DDistancePoints>();

		data->setPoint(0, std::static_pointer_cast<SurgSim::Physics::Fem3DRepresentationLocalization>(reps[0]->createLocalization(locations[0])));
		data->setPoint(1, std::static_pointer_cast<SurgSim::Physics::Fem3DRepresentationLocalization>(reps[1]->createLocalization(locations[1])));
		auto P0 = data->getPoint(0)->calculatePosition();
		auto P1 = data->getPoint(1)->calculatePosition();
		data->setDistance((P1-P0).norm());

		auto constraint = std::make_shared<SurgSim::Physics::Constraint>(
			SurgSim::Math::MLCP_BILATERAL_1D_CONSTRAINT, data,
			reps[0], locations[0],
			reps[1], locations[1]);

		// Create a component to store this constraint.
		std::shared_ptr<SurgSim::Physics::ConstraintComponent> constraintComponent =
			std::make_shared<SurgSim::Physics::ConstraintComponent>(
			"DistanceConstraint" + boost::to_string(toothId));

		constraintComponent->setConstraint(constraint);
		staple->addComponent(constraintComponent);
	}

	// If we have 2 collisions against the same Fem3DRepresentation, let's constrained the staple to the deforming Fem3D
	if (toothId >= 2 && std::dynamic_pointer_cast<Fem3DRepresentation>(reps[0]) != nullptr && reps[0] == reps[1])
	{
		// Create a bilateral constraint between the targetPhysicsRepresentation and the staple.
		{
			auto constraint = std::make_shared<SurgSim::Physics::Constraint>(SurgSim::Math::MLCP_BILATERAL_3D_CONSTRAINT,
				std::make_shared<SurgSim::Physics::ConstraintData>(),
				stapleRep, stapleLocations[0],
				reps[0], locations[0]);

			// Create a component to store this constraint.
			std::shared_ptr<SurgSim::Physics::ConstraintComponent> constraintComponent =
				std::make_shared<SurgSim::Physics::ConstraintComponent>(
				"Bilateral3DConstraint" + boost::to_string(0));

			constraintComponent->setConstraint(constraint);
			staple->addComponent(constraintComponent);
		}

		// Create a rotation vector constraint for the staple.
		{
			SurgSim::Math::Vector3d axis, rotationVector;
			double angle;
			SurgSim::Math::computeAngleAndAxis(SurgSim::Math::Quaterniond(stapleRep->getPose().linear()), &angle, &axis);
			rotationVector = angle * axis;

			auto constraintData = std::make_shared<SurgSim::Physics::ConstraintDataRotationVector>();
			constraintData->setInitialRotationVector(rotationVector);

			std::shared_ptr<Fem3DRepresentation> fem3d = std::dynamic_pointer_cast<Fem3DRepresentation>(reps[0]);
			if (fem3d != nullptr && locations[0].meshLocalCoordinate.hasValue())
			{
				// We create a temporary localization to translate the triangleId to an elementId, this feature is not directly accessible
				auto localization = std::static_pointer_cast<SurgSim::Physics::Fem3DRepresentationLocalization>(fem3d->createLocalization(locations[0]));
				auto element = fem3d->getFemElement(localization->getLocalPosition().index);
				constraintData->setInitialOrientation(element->getOrientation(*fem3d->getCurrentState()));
			}

			auto constraintRotationVector = std::make_shared<SurgSim::Physics::Constraint>(SurgSim::Math::MLCP_BILATERAL_3D_ROTATION_VECTOR_CONSTRAINT,
				constraintData,
				stapleRep, stapleLocations[0],
				reps[0], locations[0]);

			// Create a component to store this constraint.
			std::shared_ptr<SurgSim::Physics::ConstraintComponent> constraintRotationVectorComponent =
				std::make_shared<SurgSim::Physics::ConstraintComponent>(
				"RotationVectorConstraint" + boost::to_string(toothId));

			constraintRotationVectorComponent->setConstraint(constraintRotationVector);
			staple->addComponent(constraintRotationVectorComponent);
		}
	}

	if (!stapleAdded)
	{
		// Create the staple element.
		staple->setHasCollisionRepresentation(true);
		getScene()->addSceneElement(staple);
	}
}

void StaplerBehavior::update(double dt)
{
	SurgSim::DataStructures::DataGroup dataGroup;
	m_from->getData(&dataGroup);

	// Get the button1 index.
	if (!m_button1IndexCached)
	{
		m_button1Index = dataGroup.booleans().getIndex("button1");
		m_button1IndexCached = true;
	}

	// Check if the stapler is being pressed.
	bool button1 = false;
	dataGroup.booleans().get(m_button1Index, &button1);

	if (button1 && !m_buttonPreviouslyPressed)
	{
		createStaple();
	}

	m_buttonPreviouslyPressed = button1;
}

int StaplerBehavior::getTargetManagerType() const
{
	return SurgSim::Framework::MANAGER_TYPE_PHYSICS;
}

bool StaplerBehavior::doInitialize()
{
	SURGSIM_ASSERT(m_from) << "StaplerBehavior: no InputComponent held.";
	SURGSIM_ASSERT((m_virtualTeeth[0] != nullptr) && (m_virtualTeeth[1] != nullptr))
			<< "StaplerBehavior: setVirtualStaple was not called, "
			<< "or it was passed nullptr for a Collision Representation.";
	return true;
}

bool StaplerBehavior::doWakeUp()
{
	return true;
}
