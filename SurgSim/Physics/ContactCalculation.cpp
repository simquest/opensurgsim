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


#include <SurgSim/Physics/ContactCalculation.h>

#include <SurgSim/Framework/Log.h>
#include <SurgSim/Physics/CollisionPair.h>
#include <SurgSim/Physics/PhysicsManagerState.h>
#include <SurgSim/Math/Geometry.h>
#include <SurgSim/Math/RigidTransform.h>


namespace SurgSim
{
namespace Physics
{

void DefaultContactCalculation::doCalculateContact(std::shared_ptr<CollisionPair> pair)
{


	SURGSIM_ASSERT(!m_doAssert) << "Contact calculation not implemented for pairs with types ("<<
								pair->getFirst()->getShapeType() << ", " << pair->getSecond()->getShapeType() << ").";
	SURGSIM_LOG_INFO(SurgSim::Framework::Logger::getDefaultLogger()) <<
			"Contact calculation not implemented for pairs with types (" <<
			pair->getFirst()->getShapeType() << ", " << pair->getSecond()->getShapeType() << ").";
}

// Specific calculation for pairs of Spheres

void SphereSphereDcdContact::doCalculateContact(std::shared_ptr<CollisionPair> pair)
{
	SURGSIM_ASSERT(pair->getFirst()->getShapeType() == RIGID_SHAPE_TYPE_SPHERE) <<
			"First Object, wrong type of object" << pair->getFirst()->getShapeType();
	SURGSIM_ASSERT(pair->getSecond()->getShapeType() == RIGID_SHAPE_TYPE_SPHERE) <<
			"Second Object, wrong type of object" << pair->getSecond()->getShapeType();

	std::shared_ptr<SphereShape> firstSphere = std::static_pointer_cast<SphereShape>(pair->getFirst()->getShape());
	std::shared_ptr<SphereShape> secondSphere = std::static_pointer_cast<SphereShape>(pair->getSecond()->getShape());


	Vector3d firstCenter = pair->getFirst()->getPose().translation();
	Vector3d secondCenter = pair->getSecond()->getPose().translation();


	Vector3d normal = secondCenter - firstCenter;
	double dist = normal.norm();
	double maxDist = firstSphere->getRadius() + secondSphere->getRadius();
	if (dist < maxDist)
	{
		std::pair<Location,Location> penetrationPoints;
		normal.normalize();
		penetrationPoints.first.globalPosition.setValue(firstCenter - normal * firstSphere->getRadius());
		penetrationPoints.second.globalPosition.setValue(secondCenter + normal * secondSphere->getRadius());

		pair->addContact(maxDist - dist, normal, penetrationPoints);
	}
}

void SphereDoubleSidedPlaneDcdContact::doCalculateContact(std::shared_ptr<CollisionPair> pair)
{
	std::shared_ptr<CollisionRepresentation> representationPlane;
	std::shared_ptr<CollisionRepresentation> representationSphere;

	representationSphere = pair->getFirst();
	representationPlane = pair->getSecond();

	SURGSIM_ASSERT(representationSphere->getShapeType() == RIGID_SHAPE_TYPE_SPHERE) <<
			"First Object, wrong type of object" << pair->getFirst()->getShapeType();
	SURGSIM_ASSERT(representationPlane->getShapeType() == RIGID_SHAPE_TYPE_DOUBLESIDEDPLANE) <<
			"Second Object, wrong type of object" << pair->getSecond()->getShapeType();

	std::shared_ptr<SphereShape> sphere = std::static_pointer_cast<SphereShape>(representationSphere->getShape());
	std::shared_ptr<DoubleSidedPlaneShape> plane =
		std::static_pointer_cast<DoubleSidedPlaneShape>(representationPlane->getShape());

	Vector3d sphereCenter = representationSphere->getPose().translation();

	// Move into Plane coordinate system
	Vector3d planeLocalSphereCenter =  representationPlane->getPose().inverse() * sphereCenter;

	Vector3d result;
	double dist = SurgSim::Math::distancePointPlane(planeLocalSphereCenter, plane->getNormal(), plane->getD(), &result);
	double distAbsolute = std::abs(dist);
	if (distAbsolute < sphere->getRadius())
	{
		double depth = sphere->getRadius() - distAbsolute;

		// Calculate the normal going from the plane to the sphere, it is the plane normal transformed by the
		// plane pose, flipped if the sphere is behind the plane and normalize it
		Vector3d normal =
			((representationPlane->getPose() * plane->getNormal()) * ((dist < 0) ? -1.0 : 1.0)).normalized();

		std::pair<Location,Location> penetrationPoints;
		penetrationPoints.first.globalPosition.setValue(sphereCenter - normal * sphere->getRadius());
		penetrationPoints.second.globalPosition.setValue(sphereCenter - normal * distAbsolute);

		pair->addContact(depth, normal, penetrationPoints);
	}
}


void BoxDoubleSidedPlaneDcdContact::doCalculateContact(std::shared_ptr<CollisionPair> pair)
{
    using SurgSim::Math::Geometry::DistanceEpsilon;

    std::shared_ptr<CollisionRepresentation> representationPlane;
    std::shared_ptr<CollisionRepresentation> representationBox;

    representationBox = pair->getFirst();
    representationPlane = pair->getSecond();

    SURGSIM_ASSERT(representationBox->getShapeType() == RIGID_SHAPE_TYPE_BOX) <<
            "First Object, wrong type of object" << pair->getFirst()->getShapeType();
    SURGSIM_ASSERT(representationPlane->getShapeType() == RIGID_SHAPE_TYPE_DOUBLESIDEDPLANE) <<
            "Second Object, wrong type of object" << pair->getSecond()->getShapeType();

    std::shared_ptr<BoxShape> box = std::static_pointer_cast<BoxShape>(representationBox->getShape());
    std::shared_ptr<DoubleSidedPlaneShape> plane =
		std::static_pointer_cast<DoubleSidedPlaneShape>(representationPlane->getShape());

    // Transform the plane normal to box co-ordinate system.
    SurgSim::Math::RigidTransform3d planeLocalToBoxLocal = representationBox->getPose().inverse() *
                                                           representationPlane->getPose();
    SurgSim::Math::Vector3d planeNormal = planeLocalToBoxLocal.linear() * plane->getNormal();
    SurgSim::Math::Vector3d planeNormalScaled = plane->getNormal() * -plane->getD();
    SurgSim::Math::Vector4d planePoint = planeLocalToBoxLocal * SurgSim::Math::Vector4d(planeNormalScaled.x(),
                                         planeNormalScaled.y(), planeNormalScaled.z(), 1.0);
    double planeD = -planeNormal.dot(SurgSim::Math::Vector3d(planePoint.x(), planePoint.y(), planePoint.z()));

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
    int iVertex = 0;
    for (int i = -1; i <= 1; i += 2)
    {
        for (int j = -1; j <= 1; j += 2)
        {
            for (int k = -1; k <= 1; k += 2)
            {
                boxVertices[iVertex].x() = box->getSizeX() * double(i) * 0.5;
                boxVertices[iVertex].y() = box->getSizeY() * double(j) * 0.5;
                boxVertices[iVertex].z() = box->getSizeZ() * double(k) * 0.5;
                d[iVertex] = planeNormal.dot(boxVertices[iVertex]) + planeD;
                maxD = std::max(d[iVertex], maxD);
                minD = std::min(d[iVertex], minD);
                ++iVertex;
            }
        }
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

}; // Physics
}; // SurgSim
