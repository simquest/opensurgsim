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

#include "SurgSim/Physics/ContactConstraintGeneration.h"

#include <utility>
#include <vector>

#include "SurgSim/Collision/CollisionPair.h"
#include "SurgSim/Collision/Representation.h"
#include "SurgSim/DataStructures/Location.h"
#include "SurgSim/Framework/Log.h"
#include "SurgSim/Physics/Constraint.h"
#include "SurgSim/Physics/ContactConstraintData.h"
#include "SurgSim/Physics/ConstraintImplementation.h"
#include "SurgSim/Physics/PhysicsManagerState.h"
#include "SurgSim/Physics/Representation.h"

namespace SurgSim
{
namespace Physics
{

ContactConstraintGeneration::ContactConstraintGeneration(bool doCopyState) : Computation(doCopyState)
{
	m_logger = SurgSim::Framework::Logger::getLogger("ContactConstraintGeneration");
}

ContactConstraintGeneration::~ContactConstraintGeneration()
{

}

std::shared_ptr<PhysicsManagerState> ContactConstraintGeneration::doUpdate(
		const double& dt,
		const std::shared_ptr<PhysicsManagerState>& state)
{
	using SurgSim::DataStructures::Location;

	auto result = state;
	auto& pairs = result->getCollisionPairs();

	auto pairsIt = std::begin(pairs);
	auto pairsEnd = std::end(pairs);

	std::vector<std::shared_ptr<Constraint>> constraints;

	// This will check all collision pairs for contacts, then iterate over all
	// the contacts and for each contact will create a constraint between the
	// sides of the collisionpair and the localizations created from the contact
	// point. The list of all the constraints will be added back into the
	// Physics state as a result of this computation
	for (; pairsIt != pairsEnd; ++pairsIt)
	{
		if ((*pairsIt)->hasContacts())
		{
			auto contactsIt = std::begin((*pairsIt)->getContacts());
			auto contactsEnd = std::end((*pairsIt)->getContacts());
			for (; contactsIt != contactsEnd; ++contactsIt)
			{
				std::pair<std::shared_ptr<Location>, std::shared_ptr<Location>> locations;

				auto collisionRepresentations = (*pairsIt)->getRepresentations();

				auto collisionToPhysicsMap = state->getCollisionToPhysicsMap();
				auto foundFirst = collisionToPhysicsMap.find(collisionRepresentations.first);
				auto foundSecond = collisionToPhysicsMap.find(collisionRepresentations.second);
				if (foundFirst == collisionToPhysicsMap.end() || foundSecond == collisionToPhysicsMap.end())
				{
					SURGSIM_LOG_DEBUG(m_logger) << __FUNCTION__ << " Not creating a constraint. " <<
						collisionRepresentations.first->getName() << " and/or " <<
						collisionRepresentations.second->getName() << " does not have a physics representation";
					continue;
				}
				std::pair<std::shared_ptr<Representation>, std::shared_ptr<Representation>> physicsRepresentations;
				physicsRepresentations.first = foundFirst->second;
				physicsRepresentations.second = foundSecond->second;

				if (!(physicsRepresentations.first->isActive() && physicsRepresentations.second->isActive()))
				{
					SURGSIM_LOG_DEBUG(m_logger) << __FUNCTION__ << " Not creating a constraint. " <<
						physicsRepresentations.first->getName() << " and/or " <<
						physicsRepresentations.second->getName() << " is not an active physics representation";
					continue;
				}

				locations.first = makeLocation(physicsRepresentations.first, collisionRepresentations.first,
						(*contactsIt)->penetrationPoints.first);
				locations.second = makeLocation(physicsRepresentations.second, collisionRepresentations.second,
						(*contactsIt)->penetrationPoints.second);

				// HS-2013-jul-12 The type of constraint is fixed here right now, to get to a constraint
				// that we can change we probably will need to predefine collision pairs and their appropriate
				// contact constraints so we can look up which constraint to use here
				auto data = std::make_shared<ContactConstraintData>();
				data->setPlaneEquation((*contactsIt)->normal, (*contactsIt)->depth);

				constraints.push_back(std::make_shared<Constraint>(
					SurgSim::Physics::FRICTIONLESS_3DCONTACT, data,
					physicsRepresentations.first, *locations.first,
					physicsRepresentations.second, *locations.second));
			}
		}
	}

	result->setConstraintGroup(CONSTRAINT_GROUP_TYPE_CONTACT, constraints);

	return std::move(result);
}

std::shared_ptr<SurgSim::DataStructures::Location> ContactConstraintGeneration::makeLocation(
	std::shared_ptr<SurgSim::Physics::Representation> physicsRepresentation,
	std::shared_ptr<SurgSim::Collision::Representation> collisionRepresentation,
	const SurgSim::DataStructures::Location& location)
{
	auto physicsLocation = std::make_shared<SurgSim::DataStructures::Location>(location);
	SURGSIM_ASSERT(physicsLocation != nullptr) << "Cannot create Location object.";

	if (location.rigidLocalPosition.hasValue())
	{
		// Move the local position from the collision representation that created the location
		// to local coordinates of the physics representation that is creating a localization
		physicsLocation->rigidLocalPosition.setValue(
				physicsRepresentation->getLocalPose().inverse() *
				collisionRepresentation->getLocalPose() *
				location.rigidLocalPosition.getValue());
	}
	return physicsLocation;
}

}; // Physics
}; // SurgSim
