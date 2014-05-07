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
				// Create the contact.
				std::pair<Location, Location> penetrationPoints;
				penetrationPoints.first.triangleId.setValue(i);
				penetrationPoints.second.triangleId.setValue(j);
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
