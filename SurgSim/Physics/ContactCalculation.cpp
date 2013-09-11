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

	Vector3d normal = firstCenter - secondCenter;
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
	double dist = SurgSim::Math::distancePointPlane(planeLocalSphereCenter, plane->getNormal(), plane->getD(),
													&result);
	double distAbsolute = std::abs(dist);
	if (distAbsolute < sphere->getRadius())
	{
		double depth = sphere->getRadius() - distAbsolute;

		// Calculate the normal going from the plane to the sphere, it is the plane normal transformed by the
		// plane pose, flipped if the sphere is behind the plane and normalize it
		Vector3d normal =
			(representationPlane->getPose().linear() * plane->getNormal()) * ((dist < 0.0) ? -1.0 : 1.0);

		std::pair<Location,Location> penetrationPoints;
		penetrationPoints.first.globalPosition.setValue(sphereCenter - normal * sphere->getRadius());
		penetrationPoints.second.globalPosition.setValue(sphereCenter - normal * distAbsolute);

		pair->addContact(depth, normal, penetrationPoints);
	}
}

void SpherePlaneDcdContact::doCalculateContact(std::shared_ptr<CollisionPair> pair)
{
	std::shared_ptr<CollisionRepresentation> representationSphere;
	std::shared_ptr<CollisionRepresentation> representationPlane;

	representationSphere = pair->getFirst();
	representationPlane = pair->getSecond();

	SURGSIM_ASSERT(representationSphere->getShapeType() == RIGID_SHAPE_TYPE_SPHERE) <<
			"First Object, wrong type of object" << pair->getFirst()->getShapeType();
	SURGSIM_ASSERT(representationPlane->getShapeType() == RIGID_SHAPE_TYPE_PLANE) <<
			"Second Object, wrong type of object" << pair->getSecond()->getShapeType();

	std::shared_ptr<SphereShape> sphere = std::static_pointer_cast<SphereShape>(representationSphere->getShape());
	std::shared_ptr<PlaneShape> plane = std::static_pointer_cast<PlaneShape>(representationPlane->getShape());

	Vector3d sphereCenter = representationSphere->getPose().translation();

	// Move into Plane coordinate system
	Vector3d planeLocalSphereCenter =  representationPlane->getPose().inverse() * sphereCenter;

	Vector3d result;
	double dist = SurgSim::Math::distancePointPlane(planeLocalSphereCenter, plane->getNormal(), plane->getD(),
													&result);
	if (dist < sphere->getRadius())
	{
		double depth = sphere->getRadius() - dist;

		// Calculate the normal going from the plane to the sphere, it is the plane normal transformed by the
		// plane pose, flipped if the sphere is behind the plane and normalize it
		Vector3d normal = representationPlane->getPose().linear() * plane->getNormal();

		std::pair<Location,Location> penetrationPoints;
		penetrationPoints.first.globalPosition.setValue(sphereCenter - normal * sphere->getRadius());
		penetrationPoints.second.globalPosition.setValue(sphereCenter - normal * dist);

		pair->addContact(depth, normal, penetrationPoints);
	}
}

void BoxDoubleSidedPlaneDcdContact::doCalculateContact(std::shared_ptr<CollisionPair> pair)
{
    using SurgSim::Math::Geometry::DistanceEpsilon;

	std::shared_ptr<CollisionRepresentation> representationBox;
	std::shared_ptr<CollisionRepresentation> representationPlane;

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
    int iVertex = 0;
    for (int i = -1; i <= 1; i += 2)
    {
        for (int j = -1; j <= 1; j += 2)
        {
            for (int k = -1; k <= 1; k += 2)
            {
                boxVertices[iVertex].x() = box->getSizeX() * static_cast<double>(i) * 0.5;
                boxVertices[iVertex].y() = box->getSizeY() * static_cast<double>(j) * 0.5;
                boxVertices[iVertex].z() = box->getSizeZ() * static_cast<double>(k) * 0.5;
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

void BoxPlaneDcdContact::doCalculateContact(std::shared_ptr<CollisionPair> pair)
{
    using SurgSim::Math::Geometry::DistanceEpsilon;

    std::shared_ptr<CollisionRepresentation> representationBox;
	std::shared_ptr<CollisionRepresentation> representationPlane;

	representationBox = pair->getFirst();
    representationPlane = pair->getSecond();

    SURGSIM_ASSERT(representationBox->getShapeType() == RIGID_SHAPE_TYPE_BOX) <<
            "First Object, wrong type of object" << pair->getFirst()->getShapeType();
    SURGSIM_ASSERT(representationPlane->getShapeType() == RIGID_SHAPE_TYPE_PLANE) <<
            "Second Object, wrong type of object" << pair->getSecond()->getShapeType();

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
                boxVertex.x() = box->getSizeX() * static_cast<double>(i) * 0.5;
                boxVertex.y() = box->getSizeY() * static_cast<double>(j) * 0.5;
                boxVertex.z() = box->getSizeZ() * static_cast<double>(k) * 0.5;
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

void BoxSphereDcdContact::doCalculateContact(std::shared_ptr<CollisionPair> pair)
{
	using SurgSim::Math::Geometry::DistanceEpsilon;
	using SurgSim::Math::Geometry::SquaredDistanceEpsilon;

    std::shared_ptr<CollisionRepresentation> representationBox;
    std::shared_ptr<CollisionRepresentation> representationSphere;

    representationBox = pair->getFirst();
    representationSphere = pair->getSecond();

    SURGSIM_ASSERT(representationBox->getShapeType() == RIGID_SHAPE_TYPE_BOX) <<
            "First Object, wrong type of object" << pair->getFirst()->getShapeType();
    SURGSIM_ASSERT(representationSphere->getShapeType() == RIGID_SHAPE_TYPE_SPHERE) <<
            "Second Object, wrong type of object" << pair->getSecond()->getShapeType();

    std::shared_ptr<BoxShape> box = std::static_pointer_cast<BoxShape>(representationBox->getShape());
    std::shared_ptr<SphereShape> sphere = std::static_pointer_cast<SphereShape>(representationSphere->getShape());

    // Sphere center...
    Vector3d sphereCenter = representationSphere->getPose().translation();
    // ... in Box coordinate system.
    Vector3d boxLocalSphereCenter =  representationBox->getPose().inverse() * sphereCenter;

    // Box half size.
    Vector3d boxSize(box->getSizeX() * 0.5, box->getSizeY() * 0.5, box->getSizeZ() * 0.5);

    // Determine the closest point to the sphere center in the box
    Vector3d closestPoint = boxLocalSphereCenter;
    closestPoint.x() = std::min(boxSize.x(), closestPoint.x());
    closestPoint.x() = std::max(-boxSize.x(), closestPoint.x());
    closestPoint.y() = std::min(boxSize.y(), closestPoint.y());
    closestPoint.y() = std::max(-boxSize.y(), closestPoint.y());
    closestPoint.z() = std::min(boxSize.z(), closestPoint.z());
    closestPoint.z() = std::max(-boxSize.z(), closestPoint.z());

    // Distance between the closestPoint and boxLocalSphereCenter.
    Vector3d normal = boxLocalSphereCenter - closestPoint;
    double distanceSquared = normal.squaredNorm();
    if (distanceSquared - (sphere->getRadius() * sphere->getRadius()) > SquaredDistanceEpsilon)
    {
        // There is no collision.
        return;
    }

    double distance = 0.0;

    // If sphere center is inside box, it is handled differently.
    if (distanceSquared <= SquaredDistanceEpsilon)
    {
        // Sphere center is inside the box.
        // In this case closestPoint is equal to boxLocalSphereCenter.
        // Find which face of the box which is closest to the closestPoint.
        // abs(boxSize.x - closestPoint.x) and abs(-boxSize.x - closestPoint.x) are the distances between the
        // closestPoint and the two faces (along x-axis) of the box.
        // But since the abs(closestPoint.x) will always <= boxSize.x (because the point is inside box),
        // (boxSize.x() - abs(closestPoint.x())) gives the distance of the face from the closestPoint in x-axis.
        // This value is calculated for all the axes. The axis with the minimum value is assumed to contain the
        // colliding face.
        double distancesFromFaces[3] = {boxSize.x() - std::abs(closestPoint.x()),
                                        boxSize.y() - std::abs(closestPoint.y()),
                                        boxSize.z() - std::abs(closestPoint.z())
                                       };
        int minimumDistanceId = SurgSim::Math::indexOfMinimum(distancesFromFaces[0], distancesFromFaces[1],
                                                              distancesFromFaces[2]);
        // The mininumDistanceId gives the normal of the closet face.
        double direction = closestPoint[minimumDistanceId] > -DistanceEpsilon ? 1.0 : -1.0;
        normal.setZero();
        normal[minimumDistanceId] = direction;
        closestPoint[minimumDistanceId] = boxSize[minimumDistanceId] * direction;
        distance = -std::abs(distancesFromFaces[minimumDistanceId]);
    }
    else
    {
        // Sphere center is outside box.
        distance = normal.norm();
        normal /= distance;
    }

    // Transform normal into global pose.
    normal = representationBox->getPose().linear() * normal;

    // Create the contact.
    std::pair<Location,Location> penetrationPoints;
    penetrationPoints.first.globalPosition.setValue(representationBox->getPose() * closestPoint);
    penetrationPoints.second.globalPosition.setValue(sphereCenter - (normal * sphere->getRadius()));

    pair->addContact(std::abs(distance - sphere->getRadius()), normal, penetrationPoints);
}

void CapsuleSphereDcdContact::doCalculateContact(std::shared_ptr<CollisionPair> pair)
{
	std::shared_ptr<CollisionRepresentation> representationCapsule(pair->getFirst());
	std::shared_ptr<CollisionRepresentation> representationSphere(pair->getSecond());

	SURGSIM_ASSERT(representationCapsule->getShapeType() == RIGID_SHAPE_TYPE_CAPSULE) <<
			"First Object, wrong type of object" << pair->getFirst()->getShapeType();
	SURGSIM_ASSERT(representationSphere->getShapeType() == RIGID_SHAPE_TYPE_SPHERE) <<
			"Second Object, wrong type of object" << pair->getSecond()->getShapeType();

	std::shared_ptr<CapsuleShape> capsule(std::static_pointer_cast<CapsuleShape>(representationCapsule->getShape()));
	std::shared_ptr<SphereShape> sphere(std::static_pointer_cast<SphereShape>(representationSphere->getShape()));

	Vector3d sphereCenter(representationSphere->getPose().translation());
	Vector3d globalTop(representationCapsule->getPose() * capsule->topCentre());
	Vector3d globalBottom(representationCapsule->getPose() * capsule->bottomCentre());
	Vector3d result;

	double dist =
		SurgSim::Math::distancePointSegment(sphereCenter, globalTop, globalBottom, &result);
	double distThreshold = capsule->getRadius() + sphere->getRadius();

	if (dist < distThreshold)
	{
		double depth = distThreshold - dist;

		// Calculate the normal going from the sphere to the capsule
		Vector3d normal = (result - sphereCenter).normalized();

		std::pair<Location,Location> penetrationPoints;
		penetrationPoints.first.globalPosition.setValue(result - normal * capsule->getRadius());
		penetrationPoints.second.globalPosition.setValue(sphereCenter + normal * sphere->getRadius());

		pair->addContact(depth, normal, penetrationPoints);
	}
}


}; // Physics
}; // SurgSim
