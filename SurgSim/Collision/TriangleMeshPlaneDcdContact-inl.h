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

#ifndef SURGSIM_COLLISION_TRIANGLEMESHDCDCONTACT_INL_H
#define SURGSIM_COLLISION_TRIANGLEMESHDCDCONTACT_INL_H

namespace SurgSim
{
namespace Collision
{

template <class VertexType, class EdgeType, class TriangleType>
void TriangleMeshPlaneDcdContact<VertexType, EdgeType, TriangleType>::doCalculateContact
	(std::shared_ptr<CollisionPair> pair)
{
	std::shared_ptr<CollisionRepresentation> representationTriangleMesh(pair->getFirst());
	std::shared_ptr<CollisionRepresentation> representationPlane(pair->getSecond());

	std::shared_ptr<SurgSim::Physics::MeshShape<VertexType, EdgeType, TriangleType>> mesh
		(std::static_pointer_cast<SurgSim::Physics::MeshShape<VertexType, EdgeType, TriangleType>>(representationTriangleMesh->getShape()));

	std::shared_ptr<PlaneShape> plane(std::static_pointer_cast<PlaneShape>(representationPlane->getShape()));

	// Transform the plane normal to Mesh co-ordinate system.
	SurgSim::Math::RigidTransform3d planeLocalToMeshLocal = representationTriangleMesh->getPose().inverse() *
		representationPlane->getPose();
	SurgSim::Math::Vector3d planeNormal = planeLocalToMeshLocal.linear() * plane->getNormal();
	SurgSim::Math::Vector3d planeNormalScaled = plane->getNormal() * -plane->getD();
	SurgSim::Math::Vector3d planePoint = planeLocalToMeshLocal * planeNormalScaled;
	double planeD = -planeNormal.dot(planePoint);

	// Now loop through all the vertices on the Mesh and check if it below the plane
	unsigned int totalMeshVertices = mesh->getMesh()->getNumVertices();
	const std::vector<SurgSim::DataStructures::Vertex<VertexType>> Vertices = mesh->getMesh()->getVertices();

	double d;
	SurgSim::Math::Vector3d normal;
	SurgSim::Math::Vector3d meshVertex;
	SurgSim::Math::Vector3d meshVertexGlobal;

	for (unsigned int i=0; i < totalMeshVertices; ++i)
	{
		meshVertex = mesh->getMesh()->getVertex(i).position;
		d = planeNormal.dot(meshVertex) + planeD;
		if (d < SurgSim::Math::Geometry::DistanceEpsilon)
		{
			// Create the contact
			normal = representationPlane->getPose().linear() * plane->getNormal();
			std::pair<Location,Location> penetrationPoints;
			meshVertexGlobal = representationTriangleMesh->getPose() * meshVertex;
			penetrationPoints.first.globalPosition.setValue(meshVertexGlobal);
			penetrationPoints.second.globalPosition.setValue(meshVertexGlobal - normal * d);

			pair->addContact(d, normal, penetrationPoints);

		}
	}
}

}; // Physics
}; // SurgSim

#endif
