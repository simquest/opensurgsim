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
#include "SurgSim/Math/Geometry.h"
#include "SurgSim/Math/RigidTransform.h"
#include "SurgSim/Math/MeshShape.h"
#include "SurgSim/DataStructures/TriangleMesh.h"

using SurgSim::Math::MeshShape;
using SurgSim::DataStructures::TriangleMesh;
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

void TriangleMeshTriangleMeshDcdContact::doCalculateContact(std::shared_ptr<CollisionPair> pair)
{
	using SurgSim::Math::Geometry::DistanceEpsilon;
	using SurgSim::Math::Geometry::SquaredDistanceEpsilon;
	
	std::shared_ptr<Representation> representationMeshA;
	std::shared_ptr<Representation> representationMeshB;

	representationMeshA = pair->getFirst();
	representationMeshB = pair->getSecond();

	std::shared_ptr<MeshShape> meshShapeA =
		std::static_pointer_cast<MeshShape>(representationMeshA->getShape());
	std::shared_ptr<MeshShape> meshShapeB =
		std::static_pointer_cast<MeshShape>(representationMeshB->getShape());

	std::shared_ptr<MeshShape::TriMesh> meshA =
		std::dynamic_pointer_cast<MeshShape::TriMesh>(meshShapeA->getMesh());
	std::shared_ptr<MeshShape::TriMesh> meshB =
		std::dynamic_pointer_cast<MeshShape::TriMesh>(meshShapeB->getMesh());

	RigidTransform3d globalCoordinatesFromMeshACoordinates = representationMeshA->getPose();
	RigidTransform3d globalCoordinatesFromMeshBCoordinates = representationMeshB->getPose();

	RigidTransform3d meshBCoordinatesFromGlobalCoordinates = globalCoordinatesFromMeshBCoordinates.inverse();

	RigidTransform3d meshBCoordinatesFromMeshACoordinates = meshBCoordinatesFromGlobalCoordinates
															* globalCoordinatesFromMeshACoordinates;

	// Precalculate mesh B normals
	std::vector<Vector3d> collectionNormalB;
	collectionNormalB.reserve(meshB->getNumTriangles());
	for (unsigned int i = 0; i < meshB->getNumTriangles(); ++i)
	{
		const Vector3d &triangleB0 = meshA->getVertexPosition(meshB->getTriangle(i).verticesId[0]);
		const Vector3d &triangleB1 = meshA->getVertexPosition(meshB->getTriangle(i).verticesId[1]);
		const Vector3d &triangleB2 = meshA->getVertexPosition(meshB->getTriangle(i).verticesId[2]);

		collectionNormalB.emplace_back((triangleB1 - triangleB0).cross(triangleB2 - triangleB0));
	}

	double depth = 0.0;
	Vector3d normal;
	Vector3d penetrationPointA, penetrationPointB;
	
	for (unsigned int i = 0; i < meshA->getNumTriangles(); ++i)
	{
		// The triangleA vertices.
		const Vector3d &triangleA0 = meshBCoordinatesFromMeshACoordinates
									 * meshA->getVertexPosition(meshA->getTriangle(i).verticesId[0]);
		const Vector3d &triangleA1 = meshBCoordinatesFromMeshACoordinates
									 * meshA->getVertexPosition(meshA->getTriangle(i).verticesId[1]);
		const Vector3d &triangleA2 = meshBCoordinatesFromMeshACoordinates
									 * meshA->getVertexPosition(meshA->getTriangle(i).verticesId[2]);

		const Vector3d normalA = (triangleA1 - triangleA0).cross(triangleA2 - triangleA0);
		if (normalA.isZero())
		{
			continue;
		}

		for (unsigned int j = 0; j < meshB->getNumTriangles(); ++j)
		{
			const Vector3d &normalB = collectionNormalB[i];
			if (normalB.isZero())
			{
				continue;
			}

			// The triangleB vertices.
			const Vector3d &triangleB0 = meshA->getVertexPosition(meshB->getTriangle(i).verticesId[0]);
			const Vector3d &triangleB1 = meshA->getVertexPosition(meshB->getTriangle(i).verticesId[1]);
			const Vector3d &triangleB2 = meshA->getVertexPosition(meshB->getTriangle(i).verticesId[2]);

			// Check if the triangles intersect.
			if (SurgSim::Math::calculateContactTriangleTriangle(triangleA0, triangleA1, triangleA2,
																triangleB0, triangleB1, triangleB2,
																normalA, normalB, &depth,
																&penetrationPointA, &penetrationPointB,
																&normal))
			{
				// Create the contact.
				std::pair<Location,Location> penetrationPoints;
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
