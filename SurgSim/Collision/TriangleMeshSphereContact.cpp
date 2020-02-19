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

#include "SurgSim/Collision/TriangleMeshSphereContact.h"

#include "SurgSim/Collision/Representation.h"
#include "SurgSim/DataStructures/AabbTree.h"
#include "SurgSim/DataStructures/AabbTreeNode.h"
#include "SurgSim/DataStructures/IndexedLocalCoordinate.h"
#include "SurgSim/DataStructures/Location.h"
#include "SurgSim/Math/Geometry.h"
#include "SurgSim/Math/Vector.h"

using SurgSim::DataStructures::IndexedLocalCoordinate;
using SurgSim::DataStructures::Location;
using SurgSim::Math::barycentricCoordinates;
using SurgSim::Math::distancePointTriangle;
using SurgSim::Math::Vector;
using SurgSim::Math::Vector3d;


namespace SurgSim
{
namespace Collision
{

std::pair<int, int> TriangleMeshSphereContact::getShapeTypes()
{
	return std::pair<int, int>(Math::SHAPE_TYPE_MESH, Math::SHAPE_TYPE_SPHERE);
}

std::list<std::shared_ptr<Contact>> TriangleMeshSphereContact::calculateDcdContact(
									 const Math::MeshShape& mesh,
									 const Math::RigidTransform3d& meshPose,
									 const Math::SphereShape& sphere,
									 const Math::RigidTransform3d& spherePose) const
{

	std::list<std::shared_ptr<Contact>> contacts;

	Vector3d center = spherePose.translation();
	auto sphereAabbTree = std::make_shared<DataStructures::AabbTree>();
	Math::Aabbd posedBoundingBox = Math::transformAabb(Math::makeRigidTranslation(center), sphere.getBoundingBox());
	sphereAabbTree->add(posedBoundingBox, 0);
	Vector3d closestPoint;
	Vector3d coordinates;
	const double radius = sphere.getRadius();

	std::vector<size_t> candidateTriangles;

	auto intersections = mesh.getAabbTree()->spatialJoin(*sphereAabbTree);
	for (auto& intersection : intersections)
	{
		candidateTriangles.clear();

		intersection.first->getIntersections(intersection.second->getAabb(), &candidateTriangles);
		for (auto& triangle : candidateTriangles)
		{
			const Vector3d& normal = mesh.getNormal(triangle);
			if (normal.isZero())
			{
				continue;
			}
			
			auto vertices = mesh.getTrianglePositions(triangle);
			double distance = distancePointTriangle(center, vertices[0], vertices[1], vertices[2], &closestPoint);
			if (distance < radius)
			{
				double depth = radius - normal.dot(center - closestPoint);
				barycentricCoordinates(closestPoint, vertices[0], vertices[1], vertices[2], normal, &coordinates);

				std::pair<Location, Location> penetrationPoints;
				penetrationPoints.first.triangleMeshLocalCoordinate.setValue(
					DataStructures::IndexedLocalCoordinate(triangle, coordinates));
				penetrationPoints.first.rigidLocalPosition.setValue(meshPose.inverse() * closestPoint);
				penetrationPoints.second.rigidLocalPosition.setValue(-(spherePose.linear().inverse() * normal) * radius);
				contacts.push_back(std::make_shared<Contact>(
									   COLLISION_DETECTION_TYPE_DISCRETE, depth, 1.0,
									   Vector3d::Zero(), -normal, penetrationPoints));
			}
		}
	}
	return contacts;
}

};
};

