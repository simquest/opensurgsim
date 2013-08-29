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

#include <SurgSim/Math/RigidTransform.h>
#include <SurgSim/Collision/CollisionPair.h>
#include <SurgSim/Math/Geometry.h>
#include <SurgSim/Physics/BoxShape.h>
#include <SurgSim/Physics/PlaneShape.h>

using SurgSim::Physics::BoxShape;
using SurgSim::Physics::PlaneShape;

namespace SurgSim
{
namespace Collision
{

void BoxPlaneDcdContact::doCalculateContact(std::shared_ptr<CollisionPair> pair)
{
    using SurgSim::Math::Geometry::DistanceEpsilon;

    std::shared_ptr<CollisionRepresentation> representationBox;
	std::shared_ptr<CollisionRepresentation> representationPlane;
    
    representationBox = pair->getFirst();
    representationPlane = pair->getSecond();

    std::shared_ptr<BoxShape> box = std::static_pointer_cast<BoxShape>(representationBox->getShape());
    std::shared_ptr<PlaneShape> plane  = std::static_pointer_cast<PlaneShape>(representationPlane->getShape());

    // Transform the plane normal to box co-ordinate system.
    SurgSim::Math::RigidTransform3d planeLocalToBoxLocal = representationBox->getPose().inverse() *
                                                           representationPlane->getPose();
    SurgSim::Math::Vector3d planeNormal = planeLocalToBoxLocal.linear() * plane->getNormal();
    SurgSim::Math::Vector3d planeNormalScaled = plane->getNormal() * -plane->getD();
    SurgSim::Math::Vector3d planePoint = planeLocalToBoxLocal * planeNormalScaled;
    double planeD = -planeNormal.dot(planePoint);

    // Loop through the box vertices (boxVertex) and check it it is below plane.
    double d = 0.0;
    SurgSim::Math::Vector3d boxVertex;
	SurgSim::Math::Vector3d normal;
	SurgSim::Math::Vector3d boxVertexGlobal;
    for (int i = -1; i <= 1; i += 2)
    {
        for (int j = -1; j <= 1; j += 2)
        {
            for (int k = -1; k <= 1; k += 2)
            {
                boxVertex.x() = box->getSizeX() * double(i) * 0.5;
                boxVertex.y() = box->getSizeY() * double(j) * 0.5;
                boxVertex.z() = box->getSizeZ() * double(k) * 0.5;
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
    }
}

}; // namespace Collision
}; // namespace SurgSim
