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

#include <SurgSim/Physics/ContactConstraintGeneration.h>

#include <utility>
#include <vector>

#include <SurgSim/Physics/Representation.h>
#include <SurgSim/Physics/Localization.h>
#include <SurgSim/Physics/CollisionPair.h>
#include <SurgSim/Physics/CollisionRepresentation.h>

namespace SurgSim
{
namespace Physics
{

ContactConstraintGeneration::ContactConstraintGeneration()
{

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

	for (; pairsIt != pairsEnd; ++pairsIt)
	{
		if ((*pairsIt)->hasContacts())
		{
			auto firstContact  = (*pairsIt)->getContacts().front();
			std::pair<std::shared_ptr<Localization>, std::shared_ptr<Localization>> localizations;
			auto representations = (*pairsIt)->getRepresentations();
			makeLocalization(
				representations.first,
				firstContact->penetrationPoints.first,
				localizations.first);

			if (localizations.first != nullptr)
			{
				makeLocalization(
					representations.second,
					firstContact->penetrationPoints.second ,
					localizations.second);
			}

			if (localizations.first != nullptr && localizations.second != nullptr)
			{
				// Make Constraint
			}
		}
	}

	return std::move(result);
}

bool ContactConstraintGeneration::makeLocalization(
	const std::shared_ptr<CollisionRepresentation>& representation,
	const Location& location,
	std::shared_ptr<Localization> localization)
{
	bool result = false;
	auto physicsRepresenation = representation->getPhysicsRepresentation();
	if (physicsRepresenation != nullptr)
	{
		std::shared_ptr<Localization> localization = physicsRepresenation->createLocalization(location);

		// HS 2013-jun-20 this is not quite right, we are passing in the pointer to the representation
		// that we just asked about, but I don't know if we can use shared_from_this from the inside and
		// still get the same reference count as with this  shared_ptr
		localization->setRepresentation(physicsRepresenation);
		result = true;
	}
	return result;
}

}; // Physics
}; // SurgSim
