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

#include "SurgSim/Collision/TriangleMeshPlaneDcdContact.h"
#include "SurgSim/Collision/Representation.h"
#include "SurgSim/Math/Geometry.h"
#include "SurgSim/Math/RigidTransform.h"
#include "SurgSim/Math/PlaneShape.h"
#include "SurgSim/Math/MeshShape.h"

using SurgSim::DataStructures::Location;
using SurgSim::Math::MeshShape;
using SurgSim::Math::PlaneShape;
using SurgSim::Math::RigidTransform3d;
using SurgSim::Math::Vector3d;

namespace SurgSim
{
namespace Collision
{

TriangleMeshPlaneDcdContact::TriangleMeshPlaneDcdContact()
{
}

std::pair<int, int> TriangleMeshPlaneDcdContact::getShapeTypes()
{
	return std::pair<int, int> (SurgSim::Math::SHAPE_TYPE_MESH, SurgSim::Math::SHAPE_TYPE_PLANE);
}

void TriangleMeshPlaneDcdContact::doCalculateContact
	(std::shared_ptr<CollisionPair> pair)
{
	std::shared_ptr<Representation> representationTriangleMesh;
	std::shared_ptr<Representation> representationPlane;

	representationTriangleMesh = pair->getFirst();
	representationPlane = pair->getSecond();

	auto mesh = std::static_pointer_cast<MeshShape>(representationTriangleMesh->getPosedShape());

	std::shared_ptr<PlaneShape> plane(std::static_pointer_cast<PlaneShape>(representationPlane->getShape()));

	// Transform the plane normal to Mesh co-ordinate system.
	RigidTransform3d planeLocalToMeshLocal = representationPlane->getPose();
	Vector3d planeNormal = planeLocalToMeshLocal.linear() * plane->getNormal();
	Vector3d planeNormalScaled = plane->getNormal() * -plane->getD();
	Vector3d planePoint = planeLocalToMeshLocal * planeNormalScaled;
	double planeD = -planeNormal.dot(planePoint);

	double d;
	Vector3d normal;
	size_t vertexId = 0;
	for (auto& vertex : mesh->getVertices())
	{
		d = planeNormal.dot(vertex.position) + planeD;
		if (d < SurgSim::Math::Geometry::DistanceEpsilon)
		{
			// Create the contact
			normal = representationPlane->getPose().linear() * plane->getNormal();
			std::pair<Location, Location> penetrationPoints;
			penetrationPoints.first.nodeMeshLocalCoordinate.setValue(
					DataStructures::IndexedLocalCoordinate(vertexId, Math::Vector()));
			penetrationPoints.first.rigidLocalPosition.setValue(
				representationTriangleMesh->getPose().inverse() * vertex.position);

			penetrationPoints.second.rigidLocalPosition.setValue(
				representationPlane->getPose().inverse() * (vertex.position - normal * d));

			pair->addContact(-d, normal, penetrationPoints);
		}
		vertexId++;
	}
}

}; // Physics
}; // SurgSim
