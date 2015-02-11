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
#include "SurgSim/Physics/ConstraintImplementation.h"
#include "SurgSim/Physics/ContactConstraintData.h"
#include "SurgSim/Physics/Localization.h"
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
	auto result = state;
	auto pairs = result->getCollisionPairs();

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
				std::pair<std::shared_ptr<Localization>, std::shared_ptr<Localization>> localizations;

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

				localizations.first = makeLocalization(physicsRepresentations.first, collisionRepresentations.first,
						(*contactsIt)->penetrationPoints.first);
				localizations.second = makeLocalization(physicsRepresentations.second, collisionRepresentations.second,
						(*contactsIt)->penetrationPoints.second);
				if (localizations.first != nullptr && localizations.second != nullptr)
				{
					std::pair< std::shared_ptr<ConstraintImplementation>, std::shared_ptr<ConstraintImplementation>>
						implementations;

					// HS-2013-jul-12 The type of constraint is fixed here right now, to get to a constraint
					// that we can change we probably will need to predefine collision pairs and their appropriate
					// contact constraints so we can look up which constraint to use here

					implementations.first = localizations.first->getRepresentation()->createConstraint(
						SurgSim::Math::MLCP_UNILATERAL_3D_FRICTIONLESS_CONSTRAINT);
					implementations.second = localizations.second->getRepresentation()->createConstraint(
						SurgSim::Math::MLCP_UNILATERAL_3D_FRICTIONLESS_CONSTRAINT);

					if (implementations.first != nullptr && implementations.second != nullptr)
					{
						std::shared_ptr<ContactConstraintData> data = std::make_shared<ContactConstraintData>();
						data->setPlaneEquation((*contactsIt)->normal, (*contactsIt)->depth);

						constraints.push_back(std::make_shared<Constraint>(
							data,
							implementations.first,
							localizations.first,
							implementations.second,
							localizations.second));
					}
				}
			}
		}
	}

	result->setConstraintGroup(CONSTRAINT_GROUP_TYPE_CONTACT, constraints);

	return std::move(result);
}

std::shared_ptr<Localization> ContactConstraintGeneration::makeLocalization(
	std::shared_ptr<SurgSim::Physics::Representation> physicsRepresentation,
	std::shared_ptr<SurgSim::Collision::Representation> collisionRepresentation,
	const SurgSim::DataStructures::Location& location)
{
	std::shared_ptr<Localization> result;
	if (physicsRepresentation != nullptr)
	{
		if (location.rigidLocalPosition.hasValue())
		{
			// Move the local position from the collision representation that created the location
			// to local coordinates of the physics representation that is creating a localization
			SurgSim::DataStructures::Location physicsLocation = location;
			physicsLocation.rigidLocalPosition.setValue(
					physicsRepresentation->getLocalPose().inverse() *
					collisionRepresentation->getLocalPose() *
					location.rigidLocalPosition.getValue());
			result = physicsRepresentation->createLocalization(physicsLocation);
		}
		else
		{
			result = physicsRepresentation->createLocalization(location);
		}

		if (result != nullptr)
		{
			// HS 2013-jun-20 this is not quite right, we are passing in the pointer to the representation
			// that we just asked about, but I don't know if we can use shared_from_this from the inside and
			// still get the same reference count as with this  shared_ptr
			result->setRepresentation(physicsRepresentation);
		}
		else
		{
			SURGSIM_LOG_WARNING(m_logger) << __FUNCTION__ <<
				" Cannot create Localization from " << physicsRepresentation->getName();
		}
	}
	else
	{
		SURGSIM_LOG_WARNING(m_logger) << __FUNCTION__ <<
			" Cannot create Localization from nullptr.";
	}
	return result;
}

}; // Physics
}; // SurgSim
