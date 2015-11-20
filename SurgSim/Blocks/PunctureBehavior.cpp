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
#include "SurgSim/Graphics/OsgAxesRepresentation.h"
#include "SurgSim/Physics/Fem1DLocalization.h"
#include "SurgSim/Physics/Fem1DRepresentation.h"
#include "SurgSim/Physics/Fem2DRepresentation.h"

namespace SurgSim
{
namespace Blocks
{
SURGSIM_REGISTER(SurgSim::Framework::Component, SurgSim::Blocks::PunctureBehavior, PunctureBehavior);

PunctureBehavior::PunctureBehavior(const std::string& name) :
	Framework::Behavior(name), m_proximity(0.001)
{
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(PunctureBehavior, std::shared_ptr<SurgSim::Physics::Fem1DRepresentation>,
		Suture, getSuture, setSuture);
	SURGSIM_ADD_SERIALIZABLE_PROPERTY(PunctureBehavior, std::shared_ptr<SurgSim::Physics::Fem2DRepresentation>,
		Tissue, getTissue, setTissue);
}

void PunctureBehavior::setSuture(const std::shared_ptr<Physics::Fem1DRepresentation>& suture)
{
	SURGSIM_ASSERT(!isInitialized()) << "Cannot set the suture on a behavior that has been initialized";
	m_suture = suture;
}

const std::shared_ptr<Physics::Fem1DRepresentation>& PunctureBehavior::getSuture() const
{
	return m_suture;
}

void PunctureBehavior::setTissue(const std::shared_ptr<Physics::Fem2DRepresentation>& tissue)
{
	SURGSIM_ASSERT(!isInitialized()) << "Cannot set the tissue on a behavior that has been initialized";
	m_tissue = tissue;
}

const std::shared_ptr<Physics::Fem2DRepresentation>& PunctureBehavior::getTissue() const
{
	return m_tissue;
}

void PunctureBehavior::setProximity(double proximity)
{
	SURGSIM_ASSERT(!isInitialized()) << "Cannot set the proximity on a behavior that has been initialized";
	m_proximity = proximity;
}

double PunctureBehavior::getProximity() const
{
	return m_proximity;
}

void PunctureBehavior::update(double dt)
{
	if (m_puncturePoint != nullptr)
	{
		return;
	}

	// Check for collisions and see if a contact needs to be created or not.
	auto collisions = m_suture->getCollisionRepresentation()->getCollisions().safeGet();
	bool needleDriven = false;
	std::shared_ptr<Collision::Contact> driveContact;
	double minimumProximity = m_proximity * m_proximity;
	auto collisionMap = collisions->find(m_tissue->getCollisionRepresentation());
	if (collisionMap != collisions->end())
	{
		SURGSIM_ASSERT(collisionMap->first == m_tissue->getCollisionRepresentation());

		// List of contacts between suture and tissue
		const auto& contacts = collisionMap->second;
		for (const auto& contact : contacts)
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

	if (needleDriven)
	{
		m_puncturePoint = std::make_shared<Graphics::OsgAxesRepresentation>("Puncture Point");
		Math::Vector3d punctureDirection =
			m_needleEnd->calculatePosition(0.0) - m_needleEndAdjacentNode->calculatePosition(0.0);
		Math::Vector3d binormal, tangent;
		Math::buildOrthonormalBasis(&punctureDirection, &binormal, &tangent);
		Math::Matrix33d rotation;
		rotation << punctureDirection, binormal, tangent;
		m_puncturePoint->setLocalPose(Math::makeRigidTransform(rotation,
			driveContact->penetrationPoints.first.rigidLocalPosition.getValue()));

		m_tissue->getSceneElement()->addComponent(m_puncturePoint);
	}
}

int PunctureBehavior::getTargetManagerType() const
{
	return Framework::MANAGER_TYPE_PHYSICS;
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
		SURGSIM_LOG_SEVERE(Framework::Logger::getLogger("Blocks/PunctureBehavior")) <<
			getFullName() + "' must have a suture to do anything.";
		result = false;
	}

	if (m_tissue == nullptr)
	{
		SURGSIM_LOG_SEVERE(Framework::Logger::getLogger("Blocks/PunctureBehavior")) <<
			getFullName() << "must have a tissue to drive the suture into.";
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
