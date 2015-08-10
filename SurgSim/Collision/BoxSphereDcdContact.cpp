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

#include "SurgSim/Collision/BoxSphereDcdContact.h"

#include "SurgSim/Collision/CollisionPair.h"
#include "SurgSim/Collision/Representation.h"
#include "SurgSim/Math/Geometry.h"
#include "SurgSim/Math/Vector.h"

using SurgSim::DataStructures::Location;
using SurgSim::Math::BoxShape;
using SurgSim::Math::Geometry::DistanceEpsilon;
using SurgSim::Math::Geometry::SquaredDistanceEpsilon;
using SurgSim::Math::SphereShape;
using SurgSim::Math::Vector3d;

namespace SurgSim
{
namespace Collision
{

BoxSphereDcdContact::BoxSphereDcdContact()
{
}

std::pair<int, int> BoxSphereDcdContact::getShapeTypes()
{
	return std::pair<int, int>(SurgSim::Math::SHAPE_TYPE_BOX, SurgSim::Math::SHAPE_TYPE_SPHERE);
}

void BoxSphereDcdContact::doCalculateContact(std::shared_ptr<CollisionPair> pair)
{
	std::shared_ptr<Representation> box = pair->getFirst();
	std::shared_ptr<Representation> sphere = pair->getSecond();

	auto boxShape = std::static_pointer_cast<BoxShape>(box->getShape());
	auto sphereShape = std::static_pointer_cast<SphereShape>(sphere->getShape());

	auto contacts = calculateContact(*boxShape, box->getPose(), *sphereShape, sphere->getPose());
	for (auto& contact : contacts)
	{
		pair->addContact(contact);
	}
}

std::list<std::shared_ptr<Contact>> BoxSphereDcdContact::calculateContact(
									 const SurgSim::Math::BoxShape& boxShape,
									 const SurgSim::Math::RigidTransform3d& boxPose,
									 const SurgSim::Math::SphereShape& sphereShape,
									 const SurgSim::Math::RigidTransform3d& spherePose)
{
	std::list<std::shared_ptr<Contact>> contacts;

	// Sphere center...
	Vector3d sphereCenter = spherePose.translation();
	// ... in Box coordinate system.
	Vector3d boxLocalSphereCenter =  boxPose.inverse() * sphereCenter;

	// Box half size.
	Vector3d boxSize = boxShape.getSize() * 0.5;

	// Determine the closest point to the sphere center in the box
	Vector3d closestPoint = boxLocalSphereCenter;
	closestPoint.x() = std::min(boxSize.x(), closestPoint.x());
	closestPoint.x() = std::max(-boxSize.x(), closestPoint.x());
	closestPoint.y() = std::min(boxSize.y(), closestPoint.y());
	closestPoint.y() = std::max(-boxSize.y(), closestPoint.y());
	closestPoint.z() = std::min(boxSize.z(), closestPoint.z());
	closestPoint.z() = std::max(-boxSize.z(), closestPoint.z());

	// Distance between the closestPoint and boxLocalSphereCenter.  Normal points into first representation, the box.
	Vector3d normal = closestPoint - boxLocalSphereCenter;
	double distanceSquared = normal.squaredNorm();
	if (distanceSquared - (sphereShape.getRadius() * sphereShape.getRadius()) > SquaredDistanceEpsilon)
	{
		// There is no collision.
		return std::move(contacts);
	}

	double distance = 0.0;

	// If sphere center is inside box, it is handled differently.
	if (distanceSquared <= SquaredDistanceEpsilon)
	{
		// Sphere center is inside the box.
		// In this case closestPoint is equal to boxLocalSphereCenter.
		// Find which face of the box is closest to the closestPoint.
		// abs(boxSize.x - closestPoint.x) and abs(-boxSize.x - closestPoint.x) are the distances between the
		// closestPoint and the two faces (along x-axis) of the box.
		// But since the abs(closestPoint.x) will always <= boxSize.x (because the point is inside box),
		// (boxSize.x() - abs(closestPoint.x())) gives the distance from the closestPoint to whichever x-axis face is
		// closest. This value is calculated for all the axes. The axis with the minimum value contains the
		// colliding face.
		Vector3d distancesFromFaces = boxSize - closestPoint.cwiseAbs();
		int minimumDistanceId;
		distancesFromFaces.minCoeff(&minimumDistanceId);
		// The mininumDistanceId is the index of the non-zero component of the normal of the closest face.
		// The normal points toward the first representation, the box.  So the sign (or direction) of that entry is +1
		// if the closestPoint component is negative and vice versa.
		double direction = closestPoint[minimumDistanceId] > -DistanceEpsilon ? -1.0 : 1.0;
		normal.setZero();
		normal[minimumDistanceId] = direction;
		// The closestPoint should be on the closest box face, so the negative of the normal direction.
		closestPoint[minimumDistanceId] = boxSize[minimumDistanceId] * (-direction);
		distance = -std::abs(distancesFromFaces[minimumDistanceId]);
	}
	else
	{
		// Sphere center is outside box.
		distance = normal.norm();
		normal /= distance;
	}

	double depth = std::abs(distance - sphereShape.getRadius());
	normal = boxPose.linear() * normal;
	std::pair<Location, Location> penetrationPoints = std::make_pair(Location(closestPoint),
			Location(spherePose.inverse() * (sphereCenter + (normal * sphereShape.getRadius()))));

	// Create the contact.
	contacts.push_back(std::make_shared<Contact>(
						   CollisionDetectionAlgorithmType::DISCRETE_COLLISION_DETECTION, depth, 0.0,
						   Vector3d::Zero(), normal, penetrationPoints));
	return std::move(contacts);
}

}; // namespace Collision
}; // namespace SurgSim
