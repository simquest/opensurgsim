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
	auto meshShapeA = std::static_pointer_cast<MeshShape>(pair->getFirst()->getShape());
	auto meshShapeB = std::static_pointer_cast<MeshShape>(pair->getSecond()->getShape());

	std::shared_ptr<TriangleMesh> collisionMeshA = meshShapeA->getMesh();
	std::shared_ptr<TriangleMesh> collisionMeshB = meshShapeB->getMesh();

	std::list<SurgSim::DataStructures::AabbTree::TreeNodePairType> intersectionList
		= meshShapeA->getAabbTree()->spatialJoin(*meshShapeB->getAabbTree());

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
			const Vector3d& normalA = collisionMeshA->getNormal(*i);
			if (normalA.isZero())
			{
				continue;
			}

			auto verticesA = collisionMeshA->getTrianglePositions(*i);

			for (auto j = triangleListB.begin(); j != triangleListB.end(); ++j)
			{
				const Vector3d& normalB = collisionMeshB->getNormal(*j);
				if (normalB.isZero())
				{
					continue;
				}

				auto verticesB = collisionMeshB->getTrianglePositions(*j);

				// Check if the triangles intersect.
				if (SurgSim::Math::calculateContactTriangleTriangle(verticesA[0], verticesA[1], verticesA[2],
																	verticesB[0], verticesB[1], verticesB[2],
																	normalA, normalB, &depth,
																	&penetrationPointA, &penetrationPointB,
																	&normal))
				{
					// Create the contact.
					std::pair<Location, Location> penetrationPoints;
					penetrationPoints.first.globalPosition.setValue(penetrationPointA);
					penetrationPoints.second.globalPosition.setValue(penetrationPointB);

					pair->addContact(std::abs(depth), normal, penetrationPoints);
				}
			}
		}
	}
}

}; // namespace Collision
}; // namespace SurgSim
