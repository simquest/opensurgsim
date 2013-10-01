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

#include <SurgSim/Collision/BoxDoubleSidedPlaneDcdContact.h>

#include <SurgSim/Math/RigidTransform.h>
#include <SurgSim/Collision/CollisionPair.h>
#include <SurgSim/Math/Geometry.h>
#include <SurgSim/Physics/BoxShape.h>
#include <SurgSim/Physics/DoubleSidedPlaneShape.h>

using SurgSim::Physics::BoxShape;
using SurgSim::Physics::DoubleSidedPlaneShape;

namespace SurgSim
{
namespace Collision
{

void BoxDoubleSidedPlaneDcdContact::doCalculateContact(std::shared_ptr<CollisionPair> pair)
{
    using SurgSim::Math::Geometry::DistanceEpsilon;

	std::shared_ptr<CollisionRepresentation> representationBox;
	std::shared_ptr<CollisionRepresentation> representationPlane;

    representationBox = pair->getFirst();
    representationPlane = pair->getSecond();

	std::shared_ptr<BoxShape> box = std::static_pointer_cast<BoxShape>(representationBox->getShape());
    std::shared_ptr<DoubleSidedPlaneShape> plane =
		std::static_pointer_cast<DoubleSidedPlaneShape>(representationPlane->getShape());

    // Transform the plane normal to box co-ordinate system.
    SurgSim::Math::RigidTransform3d planeLocalToBoxLocal = representationBox->getPose().inverse() *
                                                           representationPlane->getPose();
    SurgSim::Math::Vector3d planeNormal = planeLocalToBoxLocal.linear() * plane->getNormal();
    SurgSim::Math::Vector3d planeNormalScaled = plane->getNormal() * -plane->getD();
    SurgSim::Math::Vector3d planePoint = planeLocalToBoxLocal * planeNormalScaled;
    double planeD = -planeNormal.dot(planePoint);

    // Loop through the box vertices (boxVertex) and calculate "d = (planeNormal.dot(boxVertex) + planeD)".
    // Keep track of max and min of 'd'.
    // Collision check overview:
    // - If 'd' values contain both positive and negative values, there is an intersection.
    // ---- Lower of the abs(maxD) and abs(minD) is the deepest penetration point.
    // ---- collisionNormal is sign(d) * planeNormal.
    // - If not, at least one of the 'd' values is zero.
    // ---- collisionNormal is sign(max(abs(maxD), abs(minD))) * planeNormal.
    double d[8] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    double maxD = -std::numeric_limits<double>::max();
	double minD = std::numeric_limits<double>::max();
    SurgSim::Math::Vector3d boxVertices[8];
    for (int i = 0; i < 8; ++i)
    {
        boxVertices[i] = box->getLocalVertex(i);
        d[i] = planeNormal.dot(boxVertices[i]) + planeD;
        maxD = std::max(d[i], maxD);
        minD = std::min(d[i], minD);
    }

    if (!(maxD > DistanceEpsilon && minD > DistanceEpsilon) && !(maxD < -DistanceEpsilon && minD < -DistanceEpsilon))
    {
        // There is an intersection.
        // Two cases:
        // - Vertex touching plane.
        // - Vertex through plane.

        SurgSim::Math::Vector3d normal;
        SurgSim::Math::Vector3d boxVertexGlobal;

        enum BoxPlaneIntersectionType
        {
            BoxPlaneIntersectionTypeEqualsZero,
            BoxPlaneIntersectionTypeLessThanZero,
            BoxPlaneIntersectionTypeGreaterThanZero
        } boxPlaneIntersectionType;

        if (std::abs(maxD) < DistanceEpsilon)
        {
            // Box is touching the "back side" of plane.
            normal = -(representationPlane->getPose().linear() * plane->getNormal());
            boxPlaneIntersectionType = BoxPlaneIntersectionTypeEqualsZero;
        }
        else if (std::abs(minD) < DistanceEpsilon)
        {
            // Box is touching the "front side" of plane.
            normal = representationPlane->getPose().linear() * plane->getNormal();
            boxPlaneIntersectionType = BoxPlaneIntersectionTypeEqualsZero;
        }
        else
        {
            if (std::abs(maxD) >= std::abs(minD))
            {
                // Box is penetrating through the "front side" of plane.
                normal = representationPlane->getPose().linear() * plane->getNormal();
                boxPlaneIntersectionType = BoxPlaneIntersectionTypeLessThanZero;
            }
            else
            {
                // Box is penetrating through the "back side" of plane.
                normal = -(representationPlane->getPose().linear() * plane->getNormal());
                boxPlaneIntersectionType = BoxPlaneIntersectionTypeGreaterThanZero;
            }
        }

        // Loop through vertices and check if a contact point needs to be generated.
        bool generateContact = false;
        for (int i = 0; i < 8; ++i)
        {
            switch (boxPlaneIntersectionType)
            {
            case BoxPlaneIntersectionTypeEqualsZero:
                generateContact = std::abs(d[i]) < DistanceEpsilon;
                break;
            case BoxPlaneIntersectionTypeLessThanZero:
                generateContact = d[i] < -DistanceEpsilon;
                break;
            case BoxPlaneIntersectionTypeGreaterThanZero:
                generateContact = d[i] > DistanceEpsilon;
                break;
            }

            if (generateContact)
            {
                std::pair<Location,Location> penetrationPoints;
                boxVertexGlobal = representationBox->getPose() * boxVertices[i];
                penetrationPoints.first.globalPosition.setValue(boxVertexGlobal);
                penetrationPoints.second.globalPosition.setValue(boxVertexGlobal + normal * std::abs(d[i]));

                pair->addContact(std::abs(d[i]), normal, penetrationPoints);

                generateContact = false;
            }
        }
    }
}

}; // namespace Collision
}; // namespace SurgSim
