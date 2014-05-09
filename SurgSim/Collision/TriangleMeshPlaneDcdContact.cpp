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
#include "SurgSim/DataStructures/TriangleMeshBase.h"

using SurgSim::Math::MeshShape;
using SurgSim::Math::PlaneShape;
using SurgSim::DataStructures::TriangleMeshBase;
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

	std::shared_ptr<MeshShape> mesh =
		std::static_pointer_cast<MeshShape>(representationTriangleMesh->getShape());

	std::shared_ptr<PlaneShape> plane(std::static_pointer_cast<PlaneShape>(representationPlane->getShape()));

	// Transform the plane normal to Mesh co-ordinate system.
	SurgSim::Math::RigidTransform3d planeLocalToMeshLocal = representationPlane->getPose();
	SurgSim::Math::Vector3d planeNormal = planeLocalToMeshLocal.linear() * plane->getNormal();
	SurgSim::Math::Vector3d planeNormalScaled = plane->getNormal() * -plane->getD();
	SurgSim::Math::Vector3d planePoint = planeLocalToMeshLocal * planeNormalScaled;
	double planeD = -planeNormal.dot(planePoint);

	// Now loop through all the vertices on the Mesh and check if it below the plane
	size_t totalMeshVertices = mesh->getMesh()->getNumVertices();

	double d;
	SurgSim::Math::Vector3d normal;
	SurgSim::Math::Vector3d meshVertex;

	for (size_t i = 0; i < totalMeshVertices; ++i)
	{
		meshVertex = mesh->getMesh()->getVertex(i).position;
		d = planeNormal.dot(meshVertex) + planeD;
		if (d < SurgSim::Math::Geometry::DistanceEpsilon)
		{
			// Create the contact
			normal = representationPlane->getPose().linear() * plane->getNormal();
			std::pair<Location,Location> penetrationPoints;
			penetrationPoints.first.globalPosition.setValue(meshVertex);
			penetrationPoints.second.globalPosition.setValue(meshVertex - normal * d);

			pair->addContact(d, normal, penetrationPoints);
		}
	}
}

}; // Physics
}; // SurgSim
