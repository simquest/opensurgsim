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

#include "SurgSim/Collision/TriangleMeshTriangleMeshDcdContact.h"

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

TriangleMeshTriangleMeshDcdContact::TriangleMeshTriangleMeshDcdContact()
{
}

std::pair<int, int> TriangleMeshTriangleMeshDcdContact::getShapeTypes()
{
	return std::pair<int, int>(SurgSim::Math::SHAPE_TYPE_MESH, SurgSim::Math::SHAPE_TYPE_MESH);
}

#ifdef SURGSIM_DEBUG_TRIANGLETRIANGLECONTACT
namespace
{

/// Asserts the points are coplanar, and prints debug output on the failing condition.
/// \param triangle0, triangle1, triangle2 the vertices of the triangle
/// \param point the point to compare against
/// \throws If the points are not coplanar
void assertIsCoplanar(const Vector3d& triangle0,
					  const Vector3d& triangle1,
					  const Vector3d& triangle2,
					  const Vector3d& point)
{
	SURGSIM_ASSERT(SurgSim::Math::isCoplanar(triangle0, triangle1, triangle2, point))
			<< "Coplanar assertion failed with: "
			"t0 [" << triangle0.transpose() << "], "
			"t1 [" << triangle1.transpose() << "], "
			"t2 [" << triangle2.transpose() << "], "
			"pt [" << point.transpose() << "]";
}

/// Asserts the point is inside the triangle, and prints debug output on the failing condition.
/// \param point the point to compare against
/// \param triangle0, triangle1, triangle2 the vertices of the triangle
/// \param normal the unit normal of the triangle
/// \throws If the point is not inside the triangle
void assertIsPointInsideTriangle(const Vector3d& point,
								 const Vector3d& triangle0,
								 const Vector3d& triangle1,
								 const Vector3d& triangle2,
								 const Vector3d& normal)
{
	SURGSIM_ASSERT(SurgSim::Math::isPointInsideTriangle(point, triangle0, triangle1, triangle2, normal))
			<< "Point inside triangle assertion failed with: "
			"t0 [" << triangle0.transpose() << "], "
			"t1 [" << triangle1.transpose() << "], "
			"t2 [" << triangle2.transpose() << "], "
			"n [" << normal.transpose() << "], "
			"pt [" << point.transpose() << "]";
}

/// Asserts the provided normal and depth minimally resolve the interpenetration of the two triangles, and prints debug
/// output on the failing condition.
/// \param normal the unit normal in the direction to resolve the penetration
/// \param penetrationDepth the depth of penetration to check
/// \param triangleA0, triangleA1, triangleA2 the vertices of the first triangle
/// \param triangleB0, triangleB1, triangleB2 the vertices of the second triangle
/// \throws If the normal and depth do not minimally resolve the interpenetration of the two triangles
void assertIsCorrectNormalAndDepth(const Vector3d& normal,
								   double penetrationDepth,
								   const Vector3d& triangleA0,
								   const Vector3d& triangleA1,
								   const Vector3d& triangleA2,
								   const Vector3d& triangleB0,
								   const Vector3d& triangleB1,
								   const Vector3d& triangleB2)
{
	Vector3d correction = normal * (penetrationDepth - SurgSim::Math::Geometry::DistanceEpsilon);

	SURGSIM_ASSERT(SurgSim::Math::doesIntersectTriangleTriangle(
					   (Vector3d)(triangleA0 + correction), (Vector3d)(triangleA1 + correction), (Vector3d)(triangleA2 + correction),
					   triangleB0, triangleB1, triangleB2))
			<< "Correct normal and depth assertion failed with: "
			"n [" << normal.transpose() << "], "
			"d [" << penetrationDepth << "], "
			"a0 [" << triangleA0.transpose() << "], "
			"a1 [" << triangleA1.transpose() << "], "
			"a2 [" << triangleA2.transpose() << "], "
			"b0 [" << triangleB0.transpose() << "], "
			"b1 [" << triangleB1.transpose() << "], "
			"b2 [" << triangleB2.transpose() << "]";

	correction = normal * (penetrationDepth + 2.0 * SurgSim::Math::Geometry::DistanceEpsilon);

	SURGSIM_ASSERT(!SurgSim::Math::doesIntersectTriangleTriangle(
					   (Vector3d)(triangleA0 + correction), (Vector3d)(triangleA1 + correction), (Vector3d)(triangleA2 + correction),
					   triangleB0, triangleB1, triangleB2))
			<< "Correct normal and depth assertion failed with: "
			"n [" << normal.transpose() << "], "
			"d [" << penetrationDepth << "], "
			"a0 [" << triangleA0.transpose() << "], "
			"a1 [" << triangleA1.transpose() << "], "
			"a2 [" << triangleA2.transpose() << "], "
			"b0 [" << triangleB0.transpose() << "], "
			"b1 [" << triangleB1.transpose() << "], "
			"b2 [" << triangleB2.transpose() << "]";
}

} // namespace
#endif //SURGSIM_DEBUG_TRIANGLETRIANGLECONTACT

void TriangleMeshTriangleMeshDcdContact::doCalculateContact(std::shared_ptr<CollisionPair> pair)
{
	auto meshA = std::static_pointer_cast<MeshShape>(pair->getFirst()->getPosedShape());
	auto meshB = std::static_pointer_cast<MeshShape>(pair->getSecond()->getPosedShape());

	std::list<SurgSim::DataStructures::AabbTree::TreeNodePairType> intersectionList
		= meshA->getAabbTree()->spatialJoin(*meshB->getAabbTree());

	double depth = 0.0;
	Vector3d normal;
	Vector3d penetrationPointA, penetrationPointB;

	for (auto intersection = intersectionList.begin(); intersection != intersectionList.end(); ++intersection)
	{
		std::shared_ptr<SurgSim::DataStructures::AabbTreeNode> nodeA = intersection->first;
		std::shared_ptr<SurgSim::DataStructures::AabbTreeNode> nodeB = intersection->second;

		std::list<size_t> triangleListA;
		std::list<size_t> triangleListB;

		nodeA->getIntersections(nodeB->getAabb(), &triangleListA);
		nodeB->getIntersections(nodeA->getAabb(), &triangleListB);

		for (auto i = triangleListA.begin(); i != triangleListA.end(); ++i)
		{
			const Vector3d& normalA = meshA->getNormal(*i);
			if (normalA.isZero())
			{
				continue;
			}

			auto verticesA = meshA->getTrianglePositions(*i);

			for (auto j = triangleListB.begin(); j != triangleListB.end(); ++j)
			{
				const Vector3d& normalB = meshB->getNormal(*j);
				if (normalB.isZero())
				{
					continue;
				}

				auto verticesB = meshB->getTrianglePositions(*j);

				// Check if the triangles intersect.
				if (SurgSim::Math::calculateContactTriangleTriangle(verticesA[0], verticesA[1], verticesA[2],
						verticesB[0], verticesB[1], verticesB[2],
						normalA, normalB, &depth,
						&penetrationPointA, &penetrationPointB,
						&normal))
				{
#ifdef SURGSIM_DEBUG_TRIANGLETRIANGLECONTACT
					assertIsCoplanar(verticesA[0], verticesA[1], verticesA[2], penetrationPointA);
					assertIsCoplanar(verticesB[0], verticesB[1], verticesB[2], penetrationPointB);

					assertIsPointInsideTriangle(
						penetrationPointA, verticesA[0], verticesA[1], verticesA[2], normalA);
					assertIsPointInsideTriangle(penetrationPointB, verticesB[0], verticesB[1], verticesB[2], normalB);

					assertIsCorrectNormalAndDepth(normal, depth, verticesA[0], verticesA[1], verticesA[2],
												  verticesB[0], verticesB[1], verticesB[2]);
#endif

					// Create the contact.
					std::pair<Location, Location> penetrationPoints;
					Vector3d barycentricCoordinate;
					SurgSim::Math::barycentricCoordinates(penetrationPointA, verticesA[0], verticesA[1], verticesA[2],
														  normalA, &barycentricCoordinate);
					penetrationPoints.first.triangleMeshLocalCoordinate.setValue(
						SurgSim::DataStructures::IndexedLocalCoordinate(*i, barycentricCoordinate));
					SurgSim::Math::barycentricCoordinates(penetrationPointB, verticesB[0], verticesB[1], verticesB[2],
														  normalB, &barycentricCoordinate);
					penetrationPoints.second.triangleMeshLocalCoordinate.setValue(
						SurgSim::DataStructures::IndexedLocalCoordinate(*j, barycentricCoordinate));

					penetrationPoints.first.rigidLocalPosition.setValue(
						pair->getFirst()->getPose().inverse() * penetrationPointA);
					penetrationPoints.second.rigidLocalPosition.setValue(
						pair->getSecond()->getPose().inverse() * penetrationPointB);

					pair->addDcdContact(std::abs(depth), normal, penetrationPoints);
				}
			}
		}
	}
}

}; // namespace Collision
}; // namespace SurgSim
