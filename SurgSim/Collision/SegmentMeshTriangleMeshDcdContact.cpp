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

#include "SurgSim/Collision/SegmentMeshTriangleMeshDcdContact.h"

#include "SurgSim/Collision/CollisionPair.h"
#include "SurgSim/Collision/Representation.h"
#include "SurgSim/DataStructures/AabbTree.h"
#include "SurgSim/DataStructures/AabbTreeNode.h"
#include "SurgSim/DataStructures/IndexedLocalCoordinate.h"
#include "SurgSim/DataStructures/TriangleMesh.h"
#include "SurgSim/Math/Geometry.h"
#include "SurgSim/Math/MeshShape.h"
#include "SurgSim/Math/RigidTransform.h"
#include "SurgSim/Math/SegmentMeshShape.h"

using SurgSim::DataStructures::Location;
using SurgSim::DataStructures::TriangleMesh;
using SurgSim::Math::MeshShape;
using SurgSim::Math::RigidTransform3d;
using SurgSim::Math::SegmentMeshShape;
using SurgSim::Math::Vector3d;

namespace SurgSim
{
namespace Collision
{

SegmentMeshTriangleMeshDcdContact::SegmentMeshTriangleMeshDcdContact()
{
}

std::pair<int,int> SegmentMeshTriangleMeshDcdContact::getShapeTypes()
{
	return std::pair<int,int>(SurgSim::Math::SHAPE_TYPE_SEGMENTMESH, SurgSim::Math::SHAPE_TYPE_MESH);
}

void SegmentMeshTriangleMeshDcdContact::doCalculateContact(std::shared_ptr<CollisionPair> pair)
{
	auto segmentMesh = std::static_pointer_cast<SegmentMeshShape>(pair->getFirst()->getPosedShape());
	auto triangleMesh = std::static_pointer_cast<MeshShape>(pair->getSecond()->getPosedShape());

	std::list<SurgSim::DataStructures::AabbTree::TreeNodePairType> intersectionList
		= segmentMesh->getAabbTree()->spatialJoin(*triangleMesh->getAabbTree());

	double radius = segmentMesh->getRadius();
	double depth = 0.0;
	Vector3d normal;
	Vector3d penetrationPointCapsule, penetrationPointTriangle;

	for (auto& intersection : intersectionList)
	{
		std::shared_ptr<SurgSim::DataStructures::AabbTreeNode> nodeSegment = intersection.first;
		std::shared_ptr<SurgSim::DataStructures::AabbTreeNode> nodeTriangle = intersection.second;

		std::list<size_t> edgeList;
		std::list<size_t> triangleList;

		nodeSegment->getIntersections(nodeTriangle->getAabb(), &edgeList);
		nodeTriangle->getIntersections(nodeSegment->getAabb(), &triangleList);

		for (auto i = triangleList.begin(); i != triangleList.end(); ++i)
		{
			const Vector3d& normalTriangle = triangleMesh->getNormal(*i);
			if (normalTriangle.isZero())
			{
				continue;
			}

			const auto& verticesTriangle = triangleMesh->getTrianglePositions(*i);

			for (auto j = edgeList.begin(); j != edgeList.end(); ++j)
			{
				const auto& verticesSegment = segmentMesh->getEdgePositions(*j);

				// Check if the triangle and capsule intersect.
				if (SurgSim::Math::calculateContactTriangleCapsule(
					verticesTriangle[0], verticesTriangle[1], verticesTriangle[2], normalTriangle,
					verticesSegment[0], verticesSegment[1], radius,
					&depth, &penetrationPointTriangle, &penetrationPointCapsule, &normal))
				{
					// Create the contact.
					std::pair<Location, Location> penetrationPoints;
					
					SurgSim::Math::Vector2d barycentricCoordinate2;
					SurgSim::Math::barycentricCoordinates(penetrationPointCapsule,
						verticesSegment[0], verticesSegment[1], &barycentricCoordinate2);
					penetrationPoints.first.elementMeshLocalCoordinate.setValue(
						SurgSim::DataStructures::IndexedLocalCoordinate(*i, barycentricCoordinate2));
					penetrationPoints.first.rigidLocalPosition.setValue(
						pair->getFirst()->getPose().inverse() * penetrationPointTriangle);
					
					Vector3d barycentricCoordinate;
					SurgSim::Math::barycentricCoordinates(penetrationPointTriangle,
						verticesTriangle[0], verticesTriangle[1], verticesTriangle[2], normalTriangle,
						&barycentricCoordinate);
					penetrationPoints.second.triangleMeshLocalCoordinate.setValue(
						SurgSim::DataStructures::IndexedLocalCoordinate(*i, barycentricCoordinate));
					penetrationPoints.second.rigidLocalPosition.setValue(
						pair->getSecond()->getPose().inverse() * penetrationPointCapsule);

					pair->addDcdContact(std::abs(depth), -normal, penetrationPoints);
				}
			}
		}
	}
}

}; // namespace Collision
}; // namespace SurgSim
