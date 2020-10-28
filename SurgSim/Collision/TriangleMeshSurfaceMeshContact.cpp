// This file is a part of the OpenSurgSim project.
// Copyright 2013-2016, SimQuest Solutions Inc.
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

#include "SurgSim/Collision/TriangleMeshSurfaceMeshContact.h"

#include "SurgSim/Collision/CollisionPair.h"
#include "SurgSim/Collision/Representation.h"
#include "SurgSim/DataStructures/AabbTree.h"
#include "SurgSim/DataStructures/AabbTreeNode.h"
#include "SurgSim/DataStructures/IndexedLocalCoordinate.h"
#include "SurgSim/DataStructures/TriangleMesh.h"
#include "SurgSim/Math/Geometry.h"
#include "SurgSim/Math/MeshShape.h"
#include "SurgSim/Math/RigidTransform.h"

using SurgSim::DataStructures::Location;
using SurgSim::DataStructures::TriangleMesh;
using SurgSim::Math::MeshShape;
using SurgSim::Math::RigidTransform3d;
using SurgSim::Math::Vector3d;

namespace SurgSim
{
namespace Collision
{

std::pair<int, int> TriangleMeshSurfaceMeshContact::getShapeTypes()
{
	return std::pair<int, int>(SurgSim::Math::SHAPE_TYPE_MESH, SurgSim::Math::SHAPE_TYPE_SURFACEMESH);
}

std::list<std::shared_ptr<Contact>> TriangleMeshSurfaceMeshContact::calculateDcdContact(
	const Math::MeshShape& meshA,
	const Math::RigidTransform3d& meshAPose,
	const Math::SurfaceMeshShape& meshB,
	const Math::RigidTransform3d& meshBPose) const
{
	std::list<std::shared_ptr<Contact>> contacts;

	auto intersectionList = meshA.getAabbTree()->spatialJoin(*(meshB.getAabbTree()));

	double depth = 0.0;
	Vector3d normal;
	Vector3d penetrationPointA, penetrationPointB;

	std::vector<size_t> triangleListA;
	std::vector<size_t> triangleListB;

	for (auto intersection = intersectionList.begin(); intersection != intersectionList.end(); ++intersection)
	{
		DataStructures::AabbTreeNode*  nodeA = intersection->first;
		DataStructures::AabbTreeNode*  nodeB = intersection->second;

		triangleListA.clear();
		triangleListB.clear();

		nodeA->getIntersections(nodeB->getAabb(), &triangleListA);
		nodeB->getIntersections(nodeA->getAabb(), &triangleListB);

		for (auto i = triangleListA.begin(); i != triangleListA.end(); ++i)
		{
			const Vector3d& normalA = meshA.getNormal(*i);
			if (normalA.isZero())
			{
				continue;
			}

			auto verticesA = meshA.getTrianglePositions(*i);

			for (auto j = triangleListB.begin(); j != triangleListB.end(); ++j)
			{
				const Vector3d& normalB = meshB.getNormal(*j);
				if (normalB.isZero())
				{
					continue;
				}

				auto verticesB = meshB.getTrianglePositions(*j);

				bool trianglesInContact = false;
				// Check if the triangle A is on the bottom side of triangle B
				if ((((verticesA[0] + verticesA[1] + verticesA[2]) / 3.0) - verticesB[0]).dot(normalB) < 0.0)
				{
					// The centroid of triangle A is below the plane of triangle B.
					// Reverse the normal of the triangle B
					trianglesInContact =
						Math::calculateContactTriangleTriangle(verticesA[0], verticesA[1], verticesA[2],
						verticesB[0], verticesB[2], verticesB[1],
						normalA, (-normalB).eval(), &depth,
						&penetrationPointA, &penetrationPointB,
						&normal);
				}
				else
				{
					trianglesInContact =
						Math::calculateContactTriangleTriangle(verticesA[0], verticesA[1], verticesA[2],
						verticesB[0], verticesB[1], verticesB[2],
						normalA, normalB, &depth,
						&penetrationPointA, &penetrationPointB,
						&normal);
				}

				// Check if the triangles intersect.
				if (trianglesInContact)
				{
					// Create the contact.
					std::pair<Location, Location> penetrationPoints;
					Vector3d barycentricCoordinate;
					Math::barycentricCoordinates(penetrationPointA, verticesA[0], verticesA[1], verticesA[2],
						normalA, &barycentricCoordinate);
					penetrationPoints.first.triangleMeshLocalCoordinate.setValue(
						DataStructures::IndexedLocalCoordinate(*i, barycentricCoordinate));
					Math::barycentricCoordinates(penetrationPointB, verticesB[0], verticesB[1], verticesB[2],
						normalB, &barycentricCoordinate);
					penetrationPoints.second.triangleMeshLocalCoordinate.setValue(
						DataStructures::IndexedLocalCoordinate(*j, barycentricCoordinate));

					penetrationPoints.first.rigidLocalPosition.setValue(meshAPose.inverse() * penetrationPointA);
					penetrationPoints.second.rigidLocalPosition.setValue(meshBPose.inverse() * penetrationPointB);

					contacts.emplace_back(std::make_shared<Contact>(
						COLLISION_DETECTION_TYPE_DISCRETE, std::abs(depth), 1.0,
						Vector3d::Zero(), normal, penetrationPoints));
				}
			}
		}
	}
	return contacts;
}

}; // namespace Collision
}; // namespace SurgSim
