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

#include "SurgSim/Collision/SphereSphereDcdContact.h"

#include "SurgSim/Collision/CollisionPair.h"
#include "SurgSim/Collision/Representation.h"
#include "SurgSim/Math/SphereShape.h"
#include "SurgSim/Math/Vector.h"

using SurgSim::DataStructures::Location;
using SurgSim::Math::SphereShape;
using SurgSim::Math::Vector3d;

namespace SurgSim
{
namespace Collision
{

SphereSphereDcdContact::SphereSphereDcdContact()
{
}

std::pair<int, int> SphereSphereDcdContact::getShapeTypes()
{
	return std::pair<int, int>(SurgSim::Math::SHAPE_TYPE_SPHERE, SurgSim::Math::SHAPE_TYPE_SPHERE);
}

void SphereSphereDcdContact::doCalculateContact(std::shared_ptr<CollisionPair> pair)
{
	std::shared_ptr<SphereShape> firstSphere = std::static_pointer_cast<SphereShape>(pair->getFirst()->getShape());
	std::shared_ptr<SphereShape> secondSphere = std::static_pointer_cast<SphereShape>(pair->getSecond()->getShape());

	Vector3d firstCenter = pair->getFirst()->getPose().translation();
	Vector3d secondCenter = pair->getSecond()->getPose().translation();

	Vector3d normal = firstCenter - secondCenter;
	double dist = normal.norm();
	double maxDist = firstSphere->getRadius() + secondSphere->getRadius();
	if (dist < maxDist)
	{
		std::pair<Location, Location> penetrationPoints;
		normal.normalize();
		penetrationPoints.first.rigidLocalPosition.setValue(
			(pair->getFirst()->getPose().linear().inverse() * -normal) * firstSphere->getRadius());
		penetrationPoints.second.rigidLocalPosition.setValue(
			(pair->getSecond()->getPose().linear().inverse() * normal) * secondSphere->getRadius());

		pair->addContact(maxDist - dist, normal, penetrationPoints);
	}
}

}; // namespace Collision
}; // namespace SurgSim
