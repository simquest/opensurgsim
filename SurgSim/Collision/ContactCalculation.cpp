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

#include <SurgSim/Collision/ContactCalculation.h>

namespace SurgSim
{
namespace Collision
{

void ContactCalculation::calculateContact(std::shared_ptr<CollisionPair> pair)
{
	std::pair<int,int> shapeTypes = getShapeTypes();
	int firstShapeType = pair->getFirst()->getShapeType();
	int secondShapeType = pair->getSecond()->getShapeType();

	if (firstShapeType != secondShapeType && firstShapeType == shapeTypes.second &&
			secondShapeType == shapeTypes.first)
	{
		pair->swapRepresentations();
	}

	if(shapeTypes.first != SurgSim::Physics::RIGID_SHAPE_TYPE_NONE)
	{
		SURGSIM_ASSERT(firstShapeType == shapeTypes.first) <<
			"First Object, wrong type of object" << firstShapeType;
	}

	if(shapeTypes.second != SurgSim::Physics::RIGID_SHAPE_TYPE_NONE)
	{
		SURGSIM_ASSERT(secondShapeType == shapeTypes.second) <<
			"Second Object, wrong type of object" << secondShapeType;
	}

	doCalculateContact(pair);
}

}; // namespace Collision
}; // namespace SurgSim