// This file is a part of the OpenSurgSim project.
// Copyright 2013-2015, SimQuest Solutions Inc.
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

#include "SurgSim/Collision/TriangleMeshPlaneContact.h"
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
std::pair<int, int> TriangleMeshPlaneContact::getShapeTypes()
{
	return std::pair<int, int> (SurgSim::Math::SHAPE_TYPE_MESH, SurgSim::Math::SHAPE_TYPE_PLANE);
}

std::list<std::shared_ptr<Contact>> TriangleMeshPlaneContact::calculateDcdContact(
									 const Math::MeshShape& mesh,
									 const Math::RigidTransform3d& meshPose,
									 const Math::PlaneShape& plane, const Math::RigidTransform3d& planePose) const
{
	std::list<std::shared_ptr<Contact>> contacts;

	// Transform the plane normal to Mesh co-ordinate system.
	Vector3d planeNormal = planePose.linear() * plane.getNormal();
	Vector3d planeNormalScaled = plane.getNormal() * -plane.getD();
	Vector3d planePoint = planePose * planeNormalScaled;
	double planeD = -planeNormal.dot(planePoint);

	double d;
	Vector3d normal;
	for (auto& vertex : mesh.getVertices())
	{
		d = planeNormal.dot(vertex.position) + planeD;
		if (d < SurgSim::Math::Geometry::DistanceEpsilon)
		{
			// Create the contact
			normal = planePose.linear() * plane.getNormal();
			std::pair<Location, Location> penetrationPoints;
			penetrationPoints.first.rigidLocalPosition.setValue(
				meshPose.inverse() * vertex.position);
			penetrationPoints.second.rigidLocalPosition.setValue(
				planePose.inverse() * (vertex.position - normal * d));

			contacts.emplace_back(std::make_shared<Contact>(
									  COLLISION_DETECTION_TYPE_DISCRETE, -d, 1.0,
									  Vector3d::Zero(), normal, penetrationPoints));
		}
	}
	return contacts;
}

}; // Physics
}; // SurgSim
