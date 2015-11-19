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

#include "SurgSim/Blocks/PunctureBehavior.h"

#include "SurgSim/Collision/CollisionPair.h"
#include "SurgSim/Collision/Representation.h"
#include "SurgSim/Framework/FrameworkConvert.h"
#include "SurgSim/Framework/Log.h"
#include "SurgSim/Framework/SceneElement.h"
#include "SurgSim/Math/RigidTransform.h"
#include "SurgSim/Physics/SlidingConstraintData.h"

using SurgSim::Math::RigidTransform3d;

namespace SurgSim
{
namespace Blocks
{
SURGSIM_REGISTER(SurgSim::Framework::Component, SurgSim::Blocks::PunctureBehavior,
	PunctureBehavior);

PunctureBehavior::PunctureBehavior(const std::string& name) :
	SurgSim::Framework::Behavior(name), m_proximity(0.001)
{
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(PunctureBehavior, std::shared_ptr<SurgSim::Physics::Fem1DRepresentation>,
		Suture, getSuture, setSuture);
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(PunctureBehavior, std::shared_ptr<SurgSim::Physics::Fem2DRepresentation>,
		Tissue, getTissue, setTissue);
}

void PunctureBehavior::setSuture(std::shared_ptr<Physics::Fem1DRepresentation> suture)
{
	SURGSIM_ASSERT(!isInitialized()) << "Cannot set the suture on a behavior that has been initialized";
	m_suture = suture;
}

std::shared_ptr<Physics::Fem1DRepresentation> PunctureBehavior::getSuture()
{
	return m_suture;
}

void PunctureBehavior::setTissue(std::shared_ptr<Physics::Fem2DRepresentation> tissue)
{
	SURGSIM_ASSERT(!isInitialized()) << "Cannot set the tissue on a behavior that has been initialized";
	m_tissue = tissue;
}

std::shared_ptr<Physics::Fem2DRepresentation> PunctureBehavior::getTissue()
{
	return m_tissue;
}

void PunctureBehavior::setProximity(double proximity)
{
	SURGSIM_ASSERT(!isInitialized()) << "Cannot set the proximity on a behavior that has been initialized";
	m_proximity = proximity;
}

/// \return The proximity from the needleEnd, within which a contact is searched for.
double PunctureBehavior::getProximity()
{
	return m_proximity;
}

void PunctureBehavior::update(double dt)
{
	if (m_puncturePoint)
	{
		return;
	}

	// Check for collisions and see if a contact needs to be created or not.
	auto collisions = m_suture->getCollisionRepresentation()->getCollisions().safeGet();
	bool needleDriven = false;
	std::shared_ptr<Collision::Contact> driveContact;
	double minimumProximity = m_proximity * m_proximity;
	if (collisions->find(m_tissue->getCollisionRepresentation()) != collisions->end())
	{
		// Suture and tissue collided. Find the collision pair near the needle end.
		for (auto& collisionMap : *collisions)
		{
			if (collisionMap.first == m_tissue->getCollisionRepresentation())
			{
				// List of contacts between suture and tissue
				auto& contacts = collisionMap.second;
				for (auto& contact : contacts)
				{
					auto& sutureContactLocation = contact->penetrationPoints.first;
					auto proximityVector =
						m_needleEnd->calculatePosition(0.0) - sutureContactLocation.rigidLocalPosition.getValue();
					double proximity = proximityVector.squaredNorm();
					if (proximity < minimumProximity)
					{
						needleDriven = true;
						minimumProximity = proximity;
						driveContact = contact;
					}
				}
			}
		}
	}

	if (needleDriven)
	{
		m_puncturePoint = std::make_shared<Graphics::OsgAxesRepresentation>("Puncture Point");
		Math::Vector3d punctureDirection =
			m_needleEnd->calculatePosition(0.0) - m_needleEndAdjacentNode->calculatePosition(0.0);
		Math::Vector3d binormal, tangent;
		Math::buildOrthonormalBasis(&punctureDirection, &binormal, &tangent);
		Math::Matrix33d rotation;
		rotation.col(0) = punctureDirection;
		rotation.col(1) = binormal;
		rotation.col(2) = tangent;
		m_puncturePoint->setLocalPose(Math::makeRigidTransform(rotation,
			driveContact->penetrationPoints.first.rigidLocalPosition.getValue()));

		m_tissue->getSceneElement()->addComponent(m_puncturePoint);
	}
}

bool PunctureBehavior::doInitialize()
{
	return true;
}

bool PunctureBehavior::doWakeUp()
{
	bool result = true;

	if (m_suture == nullptr)
	{
		SURGSIM_LOG_SEVERE(SurgSim::Framework::Logger::getDefaultLogger()) << getClassName() << " named '" +
			getName() + "' must have a suture to do anything.";
		result = false;
	}

	if (m_tissue == nullptr)
	{
		SURGSIM_LOG_SEVERE(SurgSim::Framework::Logger::getDefaultLogger()) << getClassName() << " named '" +
			getName() + "' must have a tissue to drive the suture into.";
		result = false;
	}

	if (result)
	{
		DataStructures::Location location;
		location.elementMeshLocalCoordinate = DataStructures::IndexedLocalCoordinate(0, Math::Vector2d(1.0, 0.0));
		m_needleEnd = std::dynamic_pointer_cast<Physics::Fem1DLocalization>(m_suture->createLocalization(location));
		location.elementMeshLocalCoordinate = DataStructures::IndexedLocalCoordinate(1, Math::Vector2d(1.0, 0.0));
		m_needleEndAdjacentNode =
			std::dynamic_pointer_cast<Physics::Fem1DLocalization>(m_suture->createLocalization(location));
		result = m_needleEnd != nullptr && m_needleEndAdjacentNode != nullptr;
	}

	return result;
}

}; //namespace Blocks
}; //namespace SurgSim
