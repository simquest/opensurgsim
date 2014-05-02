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
#include "SurgSim/DataStructures/TriangleMesh.h"
#include "SurgSim/DataStructures/TriangleMeshBase.h"
#include "SurgSim/Math/Geometry.h"
#include "SurgSim/Math/MeshShape.h"
#include "SurgSim/Math/RigidTransform.h"

using SurgSim::DataStructures::TriangleMesh;
using SurgSim::DataStructures::TriangleMeshBase;
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

std::pair<int,int> TriangleMeshTriangleMeshDcdContact::getShapeTypes()
{
	return std::pair<int,int>(SurgSim::Math::SHAPE_TYPE_MESH, SurgSim::Math::SHAPE_TYPE_MESH);
}

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
	SURGSIM_ASSERT(abs((triangle2 - triangle0).dot((triangle1 - triangle0).cross(point - triangle2)))
				   < SurgSim::Math::Geometry::ScalarEpsilon)
		<< "Coplanar assertion failed with: "
		<< "t0 " << triangle0.transpose() << ", t1 " << triangle1.transpose() << ", t2 " << triangle2.transpose()
		<< ", pt " << point.transpose();
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
	Vector3d barycentricCoordinate;

	SurgSim::Math::barycentricCoordinates(point, triangle0, triangle1, triangle2, normal, &barycentricCoordinate);

	SURGSIM_ASSERT(barycentricCoordinate.x() >= -SurgSim::Math::Geometry::ScalarEpsilon
				   && barycentricCoordinate.x() <= 1.0 + SurgSim::Math::Geometry::ScalarEpsilon)
		<< "Point inside triangle assertion failed with: "
		<< "t0 " << triangle0.transpose() << ", t1 " << triangle1.transpose() << ", t2 " << triangle2.transpose()
		<< ", n " << normal.transpose() << ", pt " << point.transpose();
	SURGSIM_ASSERT(barycentricCoordinate.y() >= -SurgSim::Math::Geometry::ScalarEpsilon
				   && barycentricCoordinate.y() <= 1.0 + SurgSim::Math::Geometry::ScalarEpsilon)
		<< "Point inside triangle assertion failed with: "
		<< "t0 " << triangle0.transpose() << ", t1 " << triangle1.transpose() << ", t2 " << triangle2.transpose()
		<< ", n " << normal.transpose() << ", pt " << point.transpose();
	SURGSIM_ASSERT(barycentricCoordinate.z() >= -SurgSim::Math::Geometry::ScalarEpsilon
				   && barycentricCoordinate.z() <= 1.0 + SurgSim::Math::Geometry::ScalarEpsilon)
		<< "Point inside triangle assertion failed with: "
		<< "t0 " << triangle0.transpose() << ", t1 " << triangle1.transpose() << ", t2 " << triangle2.transpose()
		<< ", n " << normal.transpose() << ", pt " << point.transpose();
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
	Vector3d correction = normal * (penetrationDepth + 2 * SurgSim::Math::Geometry::DistanceEpsilon);

	Vector3d temp1, temp2;
	double expectedDistance = SurgSim::Math::distanceTriangleTriangle(
		(Vector3d)(triangleA0 + correction), (Vector3d)(triangleA1 + correction), (Vector3d)(triangleA2 + correction),
		triangleB0, triangleB1, triangleB2,
		&temp1, &temp2);

	SURGSIM_ASSERT(expectedDistance > 0.0)
		<< "Correct normal and depth assertion failed with: "
		<< "calcD " << expectedDistance << ", n " << normal.transpose() << ", d " << penetrationDepth
		<< ", a0 " << triangleA0.transpose() << ", a1 " << triangleA1.transpose() << ", a2 " << triangleA2.transpose()
		<< ", b0 " << triangleB0.transpose() << ", b1 " << triangleB1.transpose() << ", b2 " << triangleB2.transpose();
	SURGSIM_ASSERT(expectedDistance <= 2.1 * SurgSim::Math::Geometry::DistanceEpsilon)
		<< "Correct normal and depth assertion failed with: "
		<< "calcD " << expectedDistance << ", n " << normal.transpose() << ", d " << penetrationDepth
		<< ", a0 " << triangleA0.transpose() << ", a1 " << triangleA1.transpose() << ", a2 " << triangleA2.transpose()
		<< ", b0 " << triangleB0.transpose() << ", b1 " << triangleB1.transpose() << ", b2 " << triangleB2.transpose();
}

}

void TriangleMeshTriangleMeshDcdContact::doCalculateContact(std::shared_ptr<CollisionPair> pair)
{
	std::shared_ptr<Representation> representationMeshA = pair->getFirst();
	std::shared_ptr<Representation> representationMeshB = pair->getSecond();

	std::shared_ptr<TriangleMesh> collisionMeshA =
		std::static_pointer_cast<MeshShape>(representationMeshA->getShape())->getMesh();
	std::shared_ptr<TriangleMesh> collisionMeshB =
		std::static_pointer_cast<MeshShape>(representationMeshB->getShape())->getMesh();

	RigidTransform3d globalCoordinatesFromMeshACoordinates = representationMeshA->getPose();
	RigidTransform3d globalCoordinatesFromMeshBCoordinates = representationMeshB->getPose();

	RigidTransform3d meshBCoordinatesFromGlobalCoordinates = globalCoordinatesFromMeshBCoordinates.inverse();
	RigidTransform3d meshBCoordinatesFromMeshACoordinates = meshBCoordinatesFromGlobalCoordinates
															* globalCoordinatesFromMeshACoordinates;

	double depth = 0.0;
	Vector3d normal;
	Vector3d penetrationPointA, penetrationPointB;

	for (size_t i = 0; i < collisionMeshA->getNumTriangles(); ++i)
	{
		// The triangleA vertices.
		const Vector3d& triangleA0InLocalB = meshBCoordinatesFromMeshACoordinates *
			collisionMeshA->getVertexPosition(collisionMeshA->getTriangle(i).verticesId[0]);
		const Vector3d& triangleA1InLocalB = meshBCoordinatesFromMeshACoordinates *
			collisionMeshA->getVertexPosition(collisionMeshA->getTriangle(i).verticesId[1]);
		const Vector3d& triangleA2InLocalB = meshBCoordinatesFromMeshACoordinates *
			collisionMeshA->getVertexPosition(collisionMeshA->getTriangle(i).verticesId[2]);

		const Vector3d& normalAInLocalB = meshBCoordinatesFromMeshACoordinates.linear() * collisionMeshA->getNormal(i);
		if (normalAInLocalB.isZero())
		{
			continue;
		}

		for (size_t j = 0; j < collisionMeshB->getNumTriangles(); ++j)
		{
			const Vector3d& normalB = collisionMeshB->getNormal(j);
			if (normalB.isZero())
			{
				continue;
			}

			// The triangleB vertices.
			const Vector3d& triangleB0 =
				collisionMeshB->getVertexPosition(collisionMeshB->getTriangle(j).verticesId[0]);
			const Vector3d& triangleB1 =
				collisionMeshB->getVertexPosition(collisionMeshB->getTriangle(j).verticesId[1]);
			const Vector3d& triangleB2 =
				collisionMeshB->getVertexPosition(collisionMeshB->getTriangle(j).verticesId[2]);

			// Check if the triangles intersect.
			if (SurgSim::Math::calculateContactTriangleTriangle(triangleA0InLocalB, triangleA1InLocalB,
																triangleA2InLocalB,
																triangleB0, triangleB1, triangleB2,
																normalAInLocalB, normalB, &depth,
																&penetrationPointA, &penetrationPointB,
																&normal))
			{
#ifdef SURGSIM_DEBUG_TRIANGLETRIANGLECONTACT
				assertIsCoplanar(triangleA0InLocalB, triangleA1InLocalB, triangleA2InLocalB, penetrationPointA);
				assertIsCoplanar(triangleB0, triangleB1, triangleB2, penetrationPointB);

				assertIsPointInsideTriangle(
					penetrationPointA, triangleA0InLocalB, triangleA1InLocalB, triangleA2InLocalB, normalAInLocalB);
				assertIsPointInsideTriangle(penetrationPointB, triangleB0, triangleB1, triangleB2, normalB);

				assertIsCorrectNormalAndDepth(normal, depth, triangleA0InLocalB, triangleA1InLocalB, triangleA2InLocalB,
					triangleB0, triangleB1, triangleB2);
#endif

				// Create the contact.
				std::pair<Location, Location> penetrationPoints;
				penetrationPoints.first.globalPosition.setValue(globalCoordinatesFromMeshBCoordinates
																* penetrationPointA);
				penetrationPoints.second.globalPosition.setValue(globalCoordinatesFromMeshBCoordinates
																 * penetrationPointB);

				pair->addContact(std::abs(depth), globalCoordinatesFromMeshBCoordinates.linear() * normal,
								 penetrationPoints);
			}
		}
	}
}

}; // namespace Collision
}; // namespace SurgSim
