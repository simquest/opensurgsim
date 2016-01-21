// This file is a part of the OpenSurgSim project.
// Copyright 2013-2015, SimQuest Solutions Inc.
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

#include <vector>

#include "SurgSim/Physics/ContactFiltering.h"

#include "SurgSim/Collision/CollisionPair.h"
#include "SurgSim/Collision/Representation.h"
#include "SurgSim/Physics/PhysicsManagerState.h"

using SurgSim::Collision::CollisionPair;

namespace SurgSim
{
namespace Physics
{

ContactFiltering::ContactFiltering(bool doCopyState) : Computation(doCopyState), m_angleLimit(M_PI_2)
{
	m_logger = SurgSim::Framework::Logger::getLogger("ContactFiltering");
}

ContactFiltering::~ContactFiltering()
{
}

void ContactFiltering::setAngleLimit(double angleLimit)
{
	m_angleLimit = angleLimit;
}

double ContactFiltering::getAngleLimit() const
{
	return m_angleLimit;
}

std::shared_ptr<PhysicsManagerState> ContactFiltering::doUpdate(
	const double& dt,
	const std::shared_ptr<PhysicsManagerState>& state)
{
	std::shared_ptr<PhysicsManagerState> result = state;

	for (auto& pair : result->getCollisionPairs())
	{
		auto collisionRepresentations = pair->getRepresentations();

		auto collisionToPhysicsMap = state->getCollisionToPhysicsMap();
		auto foundFirst = collisionToPhysicsMap.find(collisionRepresentations.first);
		auto foundSecond = collisionToPhysicsMap.find(collisionRepresentations.second);
		if (foundFirst == collisionToPhysicsMap.end() || foundSecond == collisionToPhysicsMap.end())
		{
			continue;
		}
		std::pair<std::shared_ptr<Representation>, std::shared_ptr<Representation>> physicsRepresentations;
		physicsRepresentations.first = foundFirst->second;
		physicsRepresentations.second = foundSecond->second;

		if (!(physicsRepresentations.first->isActive() && physicsRepresentations.second->isActive()))
		{
			continue;
		}

		for (auto& contact : pair->getContacts())
		{
			// By the contact definition, normal is pointing "in" to body1.
			// Moving body1 by normal * depth would solve the contact with body2.
			Math::Vector3d normal = contact->normal;
			auto localization1 = physicsRepresentations.first->createLocalization(contact->penetrationPoints.first);
			auto localization2 = physicsRepresentations.second->createLocalization(contact->penetrationPoints.second);
			Math::Vector3d velocity1 = localization1->calculateVelocity();
			Math::Vector3d velocity2 = localization2->calculateVelocity();
			Math::Vector3d relativeVelocity = (velocity2 - velocity1);

			// If the relative velocity is small, it means that the motion is not strong enough to drive the
			// contact filtering...i.e. all contacts are potentially valid and can push the object in any direction.
			double relativeVelocityNorm = relativeVelocity.norm();
			if (relativeVelocityNorm < 1e-4)
			{
				continue;
			}
			relativeVelocity /= relativeVelocityNorm;

			// Filter contacts that are too orthogonal to the motion and would produce inconsitant forces wrt motion
			double criteria = std::abs(normal.dot(relativeVelocity));
			if (criteria < std::cos(m_angleLimit))
			{
				contact->active = false;
				SURGSIM_LOG_DEBUG(m_logger) << "Contact filtered [|normal.relativeVelocity| = "<<
					criteria << "] < cos(" << m_angleLimit << ") = " << std::cos(m_angleLimit) << std::endl <<
					" > normal = " << normal.transpose() << std::endl <<
					" > relativeVelocity = " << relativeVelocity.transpose() << std::endl;
			}
		}
	}

	return result;
}

}; // Physics
}; // SurgSim
