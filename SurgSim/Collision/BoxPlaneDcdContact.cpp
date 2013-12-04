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

#include <SurgSim/Collision/BoxPlaneDcdContact.h>
#include <SurgSim/Collision/CollisionPair.h>

#include <SurgSim/Math/Geometry.h>
#include <SurgSim/Math/BoxShape.h>
#include <SurgSim/Math/PlaneShape.h>
#include <SurgSim/Math/RigidTransform.h>

using SurgSim::Math::BoxShape;
using SurgSim::Math::PlaneShape;
using SurgSim::Math::Vector3d;

namespace SurgSim
{
namespace Collision
{

BoxPlaneDcdContact::BoxPlaneDcdContact()
{
}

std::pair<int,int> BoxPlaneDcdContact::getShapeTypes()
{
	return std::pair<int,int>(SurgSim::Math::SHAPE_TYPE_BOX, SurgSim::Math::SHAPE_TYPE_PLANE);
}

void BoxPlaneDcdContact::doCalculateContact(std::shared_ptr<CollisionPair> pair)
{
	using SurgSim::Math::Geometry::DistanceEpsilon;

	std::shared_ptr<Representation> representationBox;
	std::shared_ptr<Representation> representationPlane;

	representationBox = pair->getFirst();
	representationPlane = pair->getSecond();

	std::shared_ptr<BoxShape> box = std::static_pointer_cast<BoxShape>(representationBox->getShape());
	std::shared_ptr<PlaneShape> plane = std::static_pointer_cast<PlaneShape>(representationPlane->getShape());

	// Transform the plane normal to box co-ordinate system.
	SurgSim::Math::RigidTransform3d planeLocalToBoxLocal = representationBox->getPose().inverse() *
														   representationPlane->getPose();
	Vector3d planeNormal = planeLocalToBoxLocal.linear() * plane->getNormal();
	Vector3d planeNormalScaled = plane->getNormal() * -plane->getD();
	Vector3d planePoint = planeLocalToBoxLocal * planeNormalScaled;
	double planeD = -planeNormal.dot(planePoint);

	// Loop through the box vertices (boxVertex) and check it it is below plane.
	double d = 0.0;
	Vector3d boxVertex;
	Vector3d normal;
	Vector3d boxVertexGlobal;
	for (int i = 0; i < 8; ++i)
	{
		boxVertex = box->getVertex(i);
		d = planeNormal.dot(boxVertex) + planeD;
		if (d < DistanceEpsilon)
		{
			// Add a contact.
			normal = representationPlane->getPose().linear() * plane->getNormal();
			std::pair<Location,Location> penetrationPoints;
			boxVertexGlobal = representationBox->getPose() * boxVertex;
			penetrationPoints.first.globalPosition.setValue(boxVertexGlobal);
			penetrationPoints.second.globalPosition.setValue(boxVertexGlobal - normal * d);

			pair->addContact(d, normal, penetrationPoints);
		}
	}
}

}; // namespace Collision
}; // namespace SurgSim
