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

#include "SurgSim/Collision/TriangleMeshTriangleMeshContact.h"

#include "SurgSim/Collision/CollisionPair.h"
#include "SurgSim/Collision/Representation.h"
#include "SurgSim/DataStructures/AabbTree.h"
#include "SurgSim/DataStructures/AabbTreeNode.h"
#include "SurgSim/DataStructures/IndexedLocalCoordinate.h"
#include "SurgSim/DataStructures/TriangleMesh.h"
#include "SurgSim/Math/Geometry.h"
#include "SurgSim/Math/MeshShape.h"
#include "SurgSim/Math/RigidTransform.h"

using SurgSim::DataStructures::Location;
using SurgSim::DataStructures::TriangleMesh;
using SurgSim::Math::MeshShape;
using SurgSim::Math::RigidTransform3d;
using SurgSim::Math::Vector3d;

namespace SurgSim
{
namespace Collision
{

std::pair<int, int> TriangleMeshTriangleMeshContact::getShapeTypes()
{
	return std::pair<int, int>(SurgSim::Math::SHAPE_TYPE_MESH, SurgSim::Math::SHAPE_TYPE_MESH);
}

#ifdef SURGSIM_DEBUG_TRIANGLETRIANGLECONTACT
namespace
{

/// Asserts the points are coplanar, and prints debug output on the failing condition.
/// \param triangle0, triangle1, triangle2 the vertices of the triangle
/// \param point the point to compare against
/// \throws If the points are not coplanar
void assertIsCoplanar(const Vector3d& triangle0,
	const Vector3d& triangle1,
	const Vector3d& triangle2,
	const Vector3d& point)
{
	SURGSIM_ASSERT(SurgSim::Math::isCoplanar(triangle0, triangle1, triangle2, point))
		<< "Coplanar assertion failed with: "
		"t0 [" << triangle0.transpose() << "], "
		"t1 [" << triangle1.transpose() << "], "
		"t2 [" << triangle2.transpose() << "], "
		"pt [" << point.transpose() << "]";
}

/// Asserts the point is inside the triangle, and prints debug output on the failing condition.
/// \param point the point to compare against
/// \param triangle0, triangle1, triangle2 the vertices of the triangle
/// \param normal the unit normal of the triangle
/// \throws If the point is not inside the triangle
void assertIsPointInsideTriangle(const Vector3d& point,
	const Vector3d& triangle0,
	const Vector3d& triangle1,
	const Vector3d& triangle2,
	const Vector3d& normal)
{
	SURGSIM_ASSERT(SurgSim::Math::isPointInsideTriangle(point, triangle0, triangle1, triangle2, normal))
		<< "Point inside triangle assertion failed with: "
		"t0 [" << triangle0.transpose() << "], "
		"t1 [" << triangle1.transpose() << "], "
		"t2 [" << triangle2.transpose() << "], "
		"n [" << normal.transpose() << "], "
		"pt [" << point.transpose() << "]";
}

/// Asserts the provided normal and depth minimally resolve the interpenetration of the two triangles, and prints debug
/// output on the failing condition.
/// \param normal the unit normal in the direction to resolve the penetration
/// \param penetrationDepth the depth of penetration to check
/// \param triangleA0, triangleA1, triangleA2 the vertices of the first triangle
/// \param triangleB0, triangleB1, triangleB2 the vertices of the second triangle
/// \throws If the normal and depth do not minimally resolve the interpenetration of the two triangles
void assertIsCorrectNormalAndDepth(const Vector3d& normal,
	double penetrationDepth,
	const Vector3d& triangleA0,
	const Vector3d& triangleA1,
	const Vector3d& triangleA2,
	const Vector3d& triangleB0,
	const Vector3d& triangleB1,
	const Vector3d& triangleB2)
{
	Vector3d correction = normal * (penetrationDepth - SurgSim::Math::Geometry::DistanceEpsilon);

	SURGSIM_ASSERT(SurgSim::Math::doesIntersectTriangleTriangle(
		(Vector3d)(triangleA0 + correction),
		(Vector3d)(triangleA1 + correction),
		(Vector3d)(triangleA2 + correction),
		triangleB0, triangleB1, triangleB2))
		<< "Correct normal and depth assertion failed with: "
		"n [" << normal.transpose() << "], "
		"d [" << penetrationDepth << "], "
		"a0 [" << triangleA0.transpose() << "], "
		"a1 [" << triangleA1.transpose() << "], "
		"a2 [" << triangleA2.transpose() << "], "
		"b0 [" << triangleB0.transpose() << "], "
		"b1 [" << triangleB1.transpose() << "], "
		"b2 [" << triangleB2.transpose() << "]";

	correction = normal * (penetrationDepth + 2.0 * SurgSim::Math::Geometry::DistanceEpsilon);

	SURGSIM_ASSERT(!SurgSim::Math::doesIntersectTriangleTriangle(
		(Vector3d)(triangleA0 + correction),
		(Vector3d)(triangleA1 + correction),
		(Vector3d)(triangleA2 + correction),
		triangleB0, triangleB1, triangleB2))
		<< "Correct normal and depth assertion failed with: "
		"n [" << normal.transpose() << "], "
		"d [" << penetrationDepth << "], "
		"a0 [" << triangleA0.transpose() << "], "
		"a1 [" << triangleA1.transpose() << "], "
		"a2 [" << triangleA2.transpose() << "], "
		"b0 [" << triangleB0.transpose() << "], "
		"b1 [" << triangleB1.transpose() << "], "
		"b2 [" << triangleB2.transpose() << "]";
}

} // namespace
#endif //SURGSIM_DEBUG_TRIANGLETRIANGLECONTACT

namespace {
struct IntersectionData {
public:
	IntersectionData() :
		timeOfImpact(std::numeric_limits<double>::max()),
		triangle1Alpha(0.0),
		triangle2Alpha(0.0),
		triangle2Beta(0.0),
		triangle1Beta(0.0),
		didOccur(false),
		isSegment(false),
		t1VertexThroughT2(false)
	{}

	double timeOfImpact;
	double triangle1Alpha;
	double triangle2Alpha;
	double triangle1Beta;
	double triangle2Beta;
	bool didOccur;
	bool isSegment;
	bool t1VertexThroughT2;
};
}

std::list<std::shared_ptr<Contact>> TriangleMeshTriangleMeshContact::calculateDcdContact(
	const Math::MeshShape& meshA,
	const Math::RigidTransform3d& meshAPose,
	const Math::MeshShape& meshB,
	const Math::RigidTransform3d& meshBPose) const
{

	std::list<std::shared_ptr<Contact>> contacts;

	std::list<SurgSim::DataStructures::AabbTree::TreeNodePairType> intersectionList
		= meshA.getAabbTree()->spatialJoin(*(meshB.getAabbTree()));

	double depth = 0.0;
	Vector3d normal;
	Vector3d penetrationPointA, penetrationPointB;

	for (auto intersection = intersectionList.begin(); intersection != intersectionList.end(); ++intersection)
	{
		std::shared_ptr<SurgSim::DataStructures::AabbTreeNode> nodeA = intersection->first;
		std::shared_ptr<SurgSim::DataStructures::AabbTreeNode> nodeB = intersection->second;

		std::list<size_t> triangleListA;
		std::list<size_t> triangleListB;

		nodeA->getIntersections(nodeB->getAabb(), &triangleListA);
		nodeB->getIntersections(nodeA->getAabb(), &triangleListB);

		for (auto i = triangleListA.begin(); i != triangleListA.end(); ++i)
		{
			const Vector3d& normalA = meshA.getNormal(*i);
			if (normalA.isZero())
			{
				continue;
			}

			auto verticesA = meshA.getTrianglePositions(*i);

			for (auto j = triangleListB.begin(); j != triangleListB.end(); ++j)
			{
				const Vector3d& normalB = meshB.getNormal(*j);
				if (normalB.isZero())
				{
					continue;
				}

				auto verticesB = meshB.getTrianglePositions(*j);

				// Check if the triangles intersect.
				if (Math::calculateContactTriangleTriangle(verticesA[0], verticesA[1], verticesA[2],
					verticesB[0], verticesB[1], verticesB[2],
					normalA, normalB, &depth,
					&penetrationPointA, &penetrationPointB,
					&normal))
				{
#ifdef SURGSIM_DEBUG_TRIANGLETRIANGLECONTACT
					assertIsCoplanar(verticesA[0], verticesA[1], verticesA[2], penetrationPointA);
					assertIsCoplanar(verticesB[0], verticesB[1], verticesB[2], penetrationPointB);

					assertIsPointInsideTriangle(
						penetrationPointA, verticesA[0], verticesA[1], verticesA[2], normalA);
					assertIsPointInsideTriangle(penetrationPointB, verticesB[0], verticesB[1], verticesB[2], normalB);

					assertIsCorrectNormalAndDepth(normal, depth, verticesA[0], verticesA[1], verticesA[2],
						verticesB[0], verticesB[1], verticesB[2]);
#endif

					// Create the contact.
					std::pair<Location, Location> penetrationPoints;
					Vector3d barycentricCoordinate;
					Math::barycentricCoordinates(penetrationPointA, verticesA[0], verticesA[1], verticesA[2],
						normalA, &barycentricCoordinate);
					penetrationPoints.first.triangleMeshLocalCoordinate.setValue(
						DataStructures::IndexedLocalCoordinate(*i, barycentricCoordinate));
					Math::barycentricCoordinates(penetrationPointB, verticesB[0], verticesB[1], verticesB[2],
						normalB, &barycentricCoordinate);
					penetrationPoints.second.triangleMeshLocalCoordinate.setValue(
						DataStructures::IndexedLocalCoordinate(*j, barycentricCoordinate));

					penetrationPoints.first.rigidLocalPosition.setValue(meshAPose.inverse() * penetrationPointA);
					penetrationPoints.second.rigidLocalPosition.setValue(meshBPose.inverse() * penetrationPointB);

					contacts.emplace_back(std::make_shared<Contact>(
						COLLISION_DETECTION_TYPE_DISCRETE, std::abs(depth), 1.0,
						Vector3d::Zero(), normal, penetrationPoints));
				}
			}
		}
	}
	return contacts;
}

std::list<std::shared_ptr<Contact>> TriangleMeshTriangleMeshContact::calculateCcdContact(
	const Math::MeshShape& shape1AtTime0, const Math::RigidTransform3d& pose1AtTime0,
	const Math::MeshShape& shape1AtTime1, const Math::RigidTransform3d& pose1AtTime1,
	const Math::MeshShape& shape2AtTime0, const Math::RigidTransform3d& pose2AtTime0,
	const Math::MeshShape& shape2AtTime1, const Math::RigidTransform3d& pose2AtTime1) const
{
	using Math::Geometry::ScalarEpsilon;
	using Math::Geometry::DistanceEpsilon;

	std::list<std::shared_ptr<Contact>> contacts;

	if (shape1AtTime0.getNumTriangles() == 0 || shape2AtTime0.getNumTriangles() == 0)
	{
		return contacts;
	}

	SURGSIM_ASSERT(shape1AtTime0.getNumTriangles() == shape1AtTime1.getNumTriangles());
	SURGSIM_ASSERT(shape2AtTime0.getNumTriangles() == shape2AtTime1.getNumTriangles());

	for (size_t triangle1Id = 0; triangle1Id < shape1AtTime0.getNumTriangles(); triangle1Id++)
	{
		auto triangle1T0 = shape1AtTime0.getTriangle(triangle1Id);
		auto triangle1T1 = shape1AtTime1.getTriangle(triangle1Id);

		std::pair<Math::Vector3d, Math::Vector3d> t1v0 = std::make_pair(
			shape1AtTime0.getVertexPosition(triangle1T0.verticesId[0]),
			shape1AtTime1.getVertexPosition(triangle1T1.verticesId[0]));
		std::pair<Math::Vector3d, Math::Vector3d> t1v1 = std::make_pair(
			shape1AtTime0.getVertexPosition(triangle1T0.verticesId[1]),
			shape1AtTime1.getVertexPosition(triangle1T1.verticesId[1]));
		std::pair<Math::Vector3d, Math::Vector3d> t1v2 = std::make_pair(
			shape1AtTime0.getVertexPosition(triangle1T0.verticesId[2]),
			shape1AtTime1.getVertexPosition(triangle1T1.verticesId[2]));

		Math::Aabbd triangle1Aabb;
		triangle1Aabb.extend(t1v0.first);
		triangle1Aabb.extend(t1v0.second);
		triangle1Aabb.extend(t1v1.first);
		triangle1Aabb.extend(t1v1.second);
		triangle1Aabb.extend(t1v2.first);
		triangle1Aabb.extend(t1v2.second);

		for (size_t triangle2Id = 0; triangle2Id < shape2AtTime0.getNumTriangles(); triangle2Id++)
		{
			auto triangle2T0 = shape2AtTime0.getTriangle(triangle2Id);
			auto triangle2T1 = shape2AtTime1.getTriangle(triangle2Id);

			std::pair<Math::Vector3d, Math::Vector3d> t2v0 = std::make_pair(
				shape2AtTime0.getVertexPosition(triangle2T0.verticesId[0]),
				shape2AtTime1.getVertexPosition(triangle2T1.verticesId[0]));
			std::pair<Math::Vector3d, Math::Vector3d> t2v1 = std::make_pair(
				shape2AtTime0.getVertexPosition(triangle2T0.verticesId[1]),
				shape2AtTime1.getVertexPosition(triangle2T1.verticesId[1]));
			std::pair<Math::Vector3d, Math::Vector3d> t2v2 = std::make_pair(
				shape2AtTime0.getVertexPosition(triangle2T0.verticesId[2]),
				shape2AtTime1.getVertexPosition(triangle2T1.verticesId[2]));

			Math::Aabbd triangle2Aabb;
			triangle2Aabb.extend(t2v0.first);
			triangle2Aabb.extend(t2v0.second);
			triangle2Aabb.extend(t2v1.first);
			triangle2Aabb.extend(t2v1.second);
			triangle2Aabb.extend(t2v2.first);
			triangle2Aabb.extend(t2v2.second);

			if (!SurgSim::Math::doAabbIntersect(triangle1Aabb, triangle2Aabb))
			{
				continue;
			}

			Math::Vector3d t1n = ((t1v1.first - t1v0.first).cross(t1v2.first - t1v0.first));
			if (t1n.norm() < Math::Geometry::DistanceEpsilon)
			{
				SURGSIM_LOG_WARNING(Framework::Logger::getLogger("TriangleMeshTriangleMeshContact")) <<
					"The triangle mesh contains a degenerate triangle (null normal)";
			}

			Math::Vector3d t2n = ((t2v1.first - t2v0.first).cross(t2v2.first - t2v0.first));
			if (t2n.norm() < Math::Geometry::DistanceEpsilon)
			{
				SURGSIM_LOG_WARNING(Framework::Logger::getLogger("TriangleMeshTriangleMeshContact")) <<
					"The triangle mesh contains a degenerate triangle (null normal)";
			}

			t1n.normalize();
			t2n.normalize();

			// Check collision at time t = 0
			Math::Vector3d pt1;
			Math::Vector3d pt2;

			ccdContactDcdCase(t1v0, t1v1, t1v2, t2v0, t2v1, t2v2, t1n, t2n,
				triangle1Id, triangle2Id, pose1AtTime1, pose2AtTime1, &contacts);

			if (!contacts.empty())
			{
				ccdContactCcdCase(t1v0, t1v1, t1v2, t2v0, t2v1, t2v2, t1n, t2n,
					triangle1Id, triangle2Id, pose1AtTime1, pose2AtTime1, &contacts);
			}
		}
	}

	return contacts;
}

void TriangleMeshTriangleMeshContact::ccdContactDcdCase(
	const std::pair<Math::Vector3d, Math::Vector3d>& t1v0, const std::pair<Math::Vector3d, Math::Vector3d>& t1v1,
	const std::pair<Math::Vector3d, Math::Vector3d>& t1v2, const std::pair<Math::Vector3d, Math::Vector3d>& t2v0,
	const std::pair<Math::Vector3d, Math::Vector3d>& t2v1, const std::pair<Math::Vector3d, Math::Vector3d>& t2v2,
	const Math::Vector3d& t1n, const Math::Vector3d& t2n, size_t triangle1Id, size_t triangle2Id,
	const Math::RigidTransform3d& pose1AtTime1, const Math::RigidTransform3d& pose2AtTime1,
	std::list<std::shared_ptr<Contact>>* contacts) const
{
	using Math::Geometry::ScalarEpsilon;

	double depth;
	Math::Vector3d pt1;
	Math::Vector3d pt2;
	Math::Vector3d normal;
	if (!Math::calculateContactTriangleTriangle(t1v0.first, t1v1.first, t1v2.first,
		t2v0.first, t2v1.first, t2v2.first, t1n, t2n, &depth, &pt1, &pt2, &normal))
	{
		return;
	}

	Math::Vector3d baryCoordTriangle1;
	Math::Vector3d baryCoordTriangle2;
	bool invalidCoordinates = false;
	if (!Math::barycentricCoordinates(pt1, t1v0.first, t1v1.first, t1v2.first, t1n, &baryCoordTriangle1))
	{
		SURGSIM_LOG_WARNING(Framework::Logger::getLogger("TriangleMeshTriangleMeshContact")) <<
			"[t=0] Could not deduce the barycentric coordinate of (" << pt1.transpose() <<
			") in the triangle (" << t1v0.first.transpose() << ")  (" << t1v1.first.transpose() <<
			") (" << t1v2.first.transpose() << ")";
		invalidCoordinates = true;
	}
	if (!Math::barycentricCoordinates(pt2, t2v0.first, t2v1.first, t2v2.first, t2n, &baryCoordTriangle2))
	{
		SURGSIM_LOG_WARNING(Framework::Logger::getLogger("TriangleMeshTriangleMeshContact")) <<
			"[t=0] Could not deduce the barycentric coordinate of (" << pt2.transpose() <<
			") in the triangle (" << t2v0.first.transpose() << ")  (" << t2v1.first.transpose() <<
			") (" << t2v2.first.transpose() << ")";
		invalidCoordinates = true;
	}

	if (invalidCoordinates)
	{
		return;
	}

	Math::Vector3d t1contact = t1v0.second * baryCoordTriangle1[0] +
		t1v1.second * baryCoordTriangle1[1] + t1v2.second * baryCoordTriangle1[2];

	Math::Vector3d t2contact = t2v0.second * baryCoordTriangle2[0] +
		t2v1.second * baryCoordTriangle2[1] + t2v2.second * baryCoordTriangle2[2];

	DataStructures::IndexedLocalCoordinate localCoordinateTriangle1(triangle1Id, baryCoordTriangle1);
	// The location related to the TriangleMesh can carry a TRIANGLE information
	DataStructures::Location locationTriangle1(localCoordinateTriangle1, Location::TRIANGLE);
	// The location related to the TriangleMesh can carry an ELEMENT information
	locationTriangle1.elementMeshLocalCoordinate = locationTriangle1.triangleMeshLocalCoordinate;
	// The location related to the TriangleMesh can carry a RIGID LOCAL POSITION information
	locationTriangle1.rigidLocalPosition = pose1AtTime1.inverse() * t1contact;

	DataStructures::IndexedLocalCoordinate localCoordinateTriangle2(triangle2Id, baryCoordTriangle2);
	// The location related to the TriangleMesh can carry a TRIANGLE information
	DataStructures::Location locationTriangle2(localCoordinateTriangle2, Location::TRIANGLE);
	// The location related to the TriangleMesh can carry an ELEMENT information
	locationTriangle2.elementMeshLocalCoordinate = locationTriangle2.triangleMeshLocalCoordinate;
	// The location related to the TriangleMesh can carry a RIGID LOCAL POSITION information
	locationTriangle2.rigidLocalPosition = pose2AtTime1.inverse() * t2contact;

	if (std::abs(std::abs(normal.dot(t1n)) - 1.0) < ScalarEpsilon)
	{
		normal = -((t1v1.second - t1v0.second).cross(t1v2.second - t1v0.second)).normalized();
	}
	else
	{
		normal = ((t2v1.second - t2v0.second).cross(t2v2.second - t2v0.second)).normalized();
	}

	depth = (t2contact - t1contact).dot(normal);
	contacts->emplace_back(std::make_shared<Contact>(
		COLLISION_DETECTION_TYPE_CONTINUOUS,
		depth,
		0.0,
		(t1contact + t2contact) * 0.5,
		normal,
		std::make_pair(locationTriangle1, locationTriangle2)));
}

void TriangleMeshTriangleMeshContact::ccdContactCcdCase(
	const std::pair<Math::Vector3d, Math::Vector3d>& t1v0, const std::pair<Math::Vector3d, Math::Vector3d>& t1v1,
	const std::pair<Math::Vector3d, Math::Vector3d>& t1v2, const std::pair<Math::Vector3d, Math::Vector3d>& t2v0,
	const std::pair<Math::Vector3d, Math::Vector3d>& t2v1, const std::pair<Math::Vector3d, Math::Vector3d>& t2v2,
	const Math::Vector3d& t1n, const Math::Vector3d& t2n, size_t triangle1Id, size_t triangle2Id,
	const Math::RigidTransform3d& pose1AtTime1, const Math::RigidTransform3d& pose2AtTime1,
	std::list<std::shared_ptr<Contact>>* contacts) const
{
	using Math::Geometry::ScalarEpsilon;

	// No collision at time t = 0, let's look for collision in the interval ]0..1]
	double earliestTimeOfImpact = std::numeric_limits<double>::max();
	double triangle1Alpha = -1.0;  //!< Barycentric coordinates of P1 in triangle t1v0t1v1t1v2
	double triangle1Beta = -1.0;  //!< P1 = t1v0 + triangle1Alpha.t1v0t1v1 + triangle1Beta.t1v0t1v2
	double triangle2Alpha = -1.0;  //!< Barycentric coordinates of P2 in triangle t2v0t2v1t2v2
	double triangle2Beta = -1.0;   //!< P2 = t2v0 + triangleAlpha.t2v0t2v1 + triangleBeta.t2v0t2v2

	bool segmentSegmentCcdFound = false;
	bool t1VertexThroughT2 = false;

	bool tritriCcdFound = false;

	// Calculate Segment/Segment ccd
	segmentSegmentCcdFound = ccdContactCcdCaseSegmentSegment(t1v0, t1v1, t1v2, t2v0, t2v1, t2v2,
		&earliestTimeOfImpact, &triangle1Alpha, &triangle1Beta, &triangle2Alpha, &triangle2Beta);

	// Calculate Point/Triangle ccd
	tritriCcdFound = ccdContactCcdCasePointTriangle(t1v0, t1v1, t1v2, t2v0, t2v1, t2v2, &earliestTimeOfImpact,
		&triangle1Alpha, &triangle1Beta, &triangle2Alpha, &triangle2Beta, &t1VertexThroughT2);

	segmentSegmentCcdFound = segmentSegmentCcdFound && !tritriCcdFound;

	// False positive from the AABB, no collision found
	if (!segmentSegmentCcdFound && !tritriCcdFound)
	{
		return;
	}

	Math::Vector3d t1contact, t2contact, normal;
	double penetrationDepthAtT1;

	Math::Vector3d t1v0v1 = t1v1.second - t1v0.second;
	Math::Vector3d t1v0v2 = t1v2.second - t1v0.second;
	t1contact = t1v0.second + triangle1Alpha * t1v0v1 + triangle1Beta * t1v0v2;

	Math::Vector3d t2v0v1 = t2v1.second - t2v0.second;
	Math::Vector3d t2v0v2 = t2v2.second - t2v0.second;
	t2contact = t2v0.second + triangle2Alpha * t2v0v1 + triangle2Beta * t2v0v2;

	if (segmentSegmentCcdFound)
	{
		Math::Vector3d t1v0v1T0 = t1v1.first - t1v0.first;
		Math::Vector3d t1v0v2T0 = t1v2.first - t1v0.first;
		Math::Vector3d t1contactT0 = t1v0.first + triangle1Alpha * t1v0v1T0 + triangle1Beta * t1v0v2T0;

		Math::Vector3d t2v0v1T0 = t2v1.first - t2v0.first;
		Math::Vector3d t2v0v2T0 = t2v2.first - t2v0.first;
		Math::Vector3d t2contactT0 = t2v0.first + triangle2Alpha * t2v0v1T0 + triangle2Beta * t2v0v2T0;

		auto dir1 = (t1contact - t1contactT0).normalized();
		auto dir2 = (t2contact - t2contactT0).normalized();

		if (std::abs(std::abs(dir1.dot(dir2)) - 1.0) < ScalarEpsilon)
		{
			normal = -dir1;
		}
		else
		{
			auto axis1 = (dir1 + dir2).normalized();
			auto axis2 = axis1.cross(dir1);
			normal = axis2.cross(axis1).normalized();
		}

		if (!Math::isValid(normal))
		{
			normal = (t1contact - t2contact).normalized();
		}
	}
	else
	{
		if (t1VertexThroughT2)
		{
			normal = t2v0v1.cross(t2v0v2).normalized();
		}
		else
		{
			normal = -t1v0v1.cross(t1v0v2).normalized();
		}
	}
	penetrationDepthAtT1 = (t2contact - t1contact).dot(normal);

	Math::Vector triangle1BaryCoord(3);
	triangle1BaryCoord << 1.0 - triangle1Alpha - triangle1Beta, triangle1Alpha, triangle1Beta;
	DataStructures::IndexedLocalCoordinate localCoordinateTriangle1(triangle1Id, triangle1BaryCoord);
	// The location related to the TriangleMesh can carry a TRIANGLE information
	DataStructures::Location locationTriangle1(localCoordinateTriangle1, Location::TRIANGLE);
	// The location related to the TriangleMesh can carry an ELEMENT information
	locationTriangle1.elementMeshLocalCoordinate = locationTriangle1.triangleMeshLocalCoordinate;
	// The location related to the TriangleMesh can carry a RIGID LOCAL POSITION information
	locationTriangle1.rigidLocalPosition = pose1AtTime1.inverse() * t1contact;

	Math::Vector triangle2BaryCoord(3);
	triangle2BaryCoord << 1.0 - triangle2Alpha - triangle2Beta, triangle2Alpha, triangle2Beta;
	DataStructures::IndexedLocalCoordinate localCoordinateTriangle2(triangle2Id, triangle2BaryCoord);
	// The location related to the TriangleMesh can carry a TRIANGLE information
	DataStructures::Location locationTriangle2(localCoordinateTriangle2, Location::TRIANGLE);
	// The location related to the TriangleMesh can carry an ELEMENT information
	locationTriangle2.elementMeshLocalCoordinate = locationTriangle2.triangleMeshLocalCoordinate;
	// The location related to the TriangleMesh can carry a RIGID LOCAL POSITION information
	locationTriangle2.rigidLocalPosition = pose2AtTime1.inverse() * t2contact;

	contacts->emplace_back(std::make_shared<Contact>(
		COLLISION_DETECTION_TYPE_CONTINUOUS,
		penetrationDepthAtT1,
		earliestTimeOfImpact,
		(t1contact + t2contact) * 0.5,
		normal,
		std::make_pair(locationTriangle1, locationTriangle2)));
}

bool TriangleMeshTriangleMeshContact::ccdContactCcdCaseSegmentSegment(
	const std::pair<Math::Vector3d, Math::Vector3d>& t1v0, const std::pair<Math::Vector3d, Math::Vector3d>& t1v1,
	const std::pair<Math::Vector3d, Math::Vector3d>& t1v2, const std::pair<Math::Vector3d, Math::Vector3d>& t2v0,
	const std::pair<Math::Vector3d, Math::Vector3d>& t2v1, const std::pair<Math::Vector3d, Math::Vector3d>& t2v2,
	double* earliestTimeOfImpact, double* triangle1Alpha, double* triangle1Beta, double* triangle2Alpha,
	double* triangle2Beta) const
{
	using Math::calculateCcdContactSegmentSegment;

	bool segmentSegmentCcdFound = false;
	double timeOfImpact;
	double t1Factor;
	double t2Factor;

	std::array<IntersectionData, 9> d;

	d[0].didOccur = calculateCcdContactSegmentSegment(t1v0, t1v1, t2v0, t2v1, &d[0].timeOfImpact, &t1Factor, &t2Factor);
	d[0].isSegment = true;
	d[0].triangle1Alpha = t1Factor;
	d[0].triangle1Beta = 0.0;
	d[0].triangle2Alpha = t2Factor;
	d[0].triangle2Beta = 0.0;

	d[1].didOccur = calculateCcdContactSegmentSegment(t1v0, t1v1, t2v1, t2v2, &d[1].timeOfImpact, &t1Factor, &t2Factor);
	d[1].isSegment = true;
	d[1].triangle1Alpha = t1Factor;
	d[1].triangle1Beta = 0.0;
	d[1].triangle2Alpha = 1.0 - t2Factor; // P = P0 + P0P1.(1 - tFactor) + P0P2.tFactor
	d[1].triangle2Beta = t2Factor;

	d[2].didOccur = calculateCcdContactSegmentSegment(t1v0, t1v1, t2v0, t2v2, &d[2].timeOfImpact, &t1Factor, &t2Factor);
	d[2].isSegment = true;
	d[2].triangle1Alpha = t1Factor;
	d[2].triangle1Beta = 0.0;
	d[2].triangle2Alpha = 0.0; // P = P0 + P0P2.(tFactor)
	d[2].triangle2Beta = t2Factor;

	d[3].didOccur = calculateCcdContactSegmentSegment(t1v1, t1v2, t2v0, t2v1, &d[3].timeOfImpact, &t1Factor, &t2Factor);
	d[3].isSegment = true;
	d[3].triangle1Alpha = 1.0 - t1Factor; // P = P0 + P0P1.(1 - tFactor) + P0P2.tFactor
	d[3].triangle1Beta = t1Factor;
	d[3].triangle2Alpha = t2Factor;
	d[3].triangle2Beta = 0.0;

	d[4].didOccur = calculateCcdContactSegmentSegment(t1v1, t1v2, t2v1, t2v2, &d[4].timeOfImpact, &t1Factor, &t2Factor);
	d[4].isSegment = true;
	d[4].triangle1Alpha = 1.0 - t1Factor; // P = P0 + P0P1.(1 - tFactor) + P0P2.tFactor
	d[4].triangle1Beta = t1Factor;
	d[4].triangle2Alpha = 1.0 - t2Factor; // P = P0 + P0P1.(1 - tFactor) + P0P2.tFactor
	d[4].triangle2Beta = t2Factor;

	d[5].didOccur = calculateCcdContactSegmentSegment(t1v1, t1v2, t2v0, t2v2, &d[5].timeOfImpact, &t1Factor, &t2Factor);
	d[5].isSegment = true;
	d[5].triangle1Alpha = 1.0 - t1Factor; // P = P0 + P0P1.(1 - tFactor) + P0P2.tFactor
	d[5].triangle1Beta = t1Factor;
	d[5].triangle2Alpha = 0.0; // P = P0 + P0P2.(tFactor)
	d[5].triangle2Beta = t2Factor;

	d[6].didOccur = calculateCcdContactSegmentSegment(t1v0, t1v2, t2v0, t2v1, &d[6].timeOfImpact, &t1Factor, &t2Factor);
	d[6].isSegment = true;
	d[6].triangle1Alpha = 0.0; // P = P0 + P0P2.(tFactor)
	d[6].triangle1Beta = t1Factor;
	d[6].triangle2Alpha = t2Factor;
	d[6].triangle2Beta = 0.0;

	d[7].didOccur = calculateCcdContactSegmentSegment(t1v0, t1v2, t2v1, t2v2, &d[7].timeOfImpact, &t1Factor, &t2Factor);
	d[7].isSegment = true;
	d[7].triangle1Alpha = 0.0; // P = P0 + P0P2.(tFactor)
	d[7].triangle1Beta = t1Factor;
	d[7].triangle2Alpha = 1.0 - t2Factor; // P = P0 + P0P1.(1 - tFactor) + P0P2.tFactor
	d[7].triangle2Beta = t2Factor;

	d[8].didOccur = calculateCcdContactSegmentSegment(t1v0, t1v2, t2v0, t2v2, &d[8].timeOfImpact, &t1Factor, &t2Factor);
	d[8].isSegment = true;
	d[8].triangle1Alpha = 0.0; // P = P0 + P0P2.(tFactor)
	d[8].triangle1Beta = t1Factor;
	d[8].triangle2Alpha = 0.0; // P = P0 + P0P2.(tFactor)
	d[8].triangle2Beta = t2Factor;

	std::sort(d.begin(), d.end(), [](const IntersectionData& a, const IntersectionData& b) {
		return a.timeOfImpact < b.timeOfImpact; });

	*earliestTimeOfImpact = d[0].timeOfImpact;
	*triangle1Alpha = d[0].triangle1Alpha;
	*triangle2Alpha = d[0].triangle2Alpha;
	*triangle1Beta = d[0].triangle1Beta;
	*triangle2Beta = d[0].triangle2Beta;

	return d[0].didOccur;
}


bool TriangleMeshTriangleMeshContact::ccdContactCcdCasePointTriangle(
	const std::pair<Math::Vector3d, Math::Vector3d>& t1v0, const std::pair<Math::Vector3d, Math::Vector3d>& t1v1,
	const std::pair<Math::Vector3d, Math::Vector3d>& t1v2, const std::pair<Math::Vector3d, Math::Vector3d>& t2v0,
	const std::pair<Math::Vector3d, Math::Vector3d>& t2v1, const std::pair<Math::Vector3d, Math::Vector3d>& t2v2,
	double* earliestTimeOfImpact, double* triangle1Alpha, double* triangle1Beta, double* triangle2Alpha,
	double* triangle2Beta, bool* t1VertexThroughT2) const
{
	using Math::calculateCcdContactPointTriangle;

	bool pointTriangleCcdFound = false;
	double timeOfImpact;
	double u;
	double v;
	std::array<IntersectionData, 6> d;

	d[0].didOccur = calculateCcdContactPointTriangle(t1v0, t2v0, t2v1, t2v2, &d[0].timeOfImpact, &u, &v);
	d[0].t1VertexThroughT2 = true;
	d[0].triangle1Alpha = 0.0;
	d[0].triangle1Beta = 0.0;
	d[0].triangle2Alpha = u;
	d[0].triangle2Beta = v;



	d[1].didOccur = calculateCcdContactPointTriangle(t1v1, t2v0, t2v1, t2v2, &d[1].timeOfImpact, &u, &v);
	d[1].t1VertexThroughT2 = true;
	d[1].triangle1Alpha = 1.0;
	d[1].triangle1Beta = 0.0;
	d[1].triangle2Alpha = u;
	d[1].triangle2Beta = v;


	d[2].didOccur = calculateCcdContactPointTriangle(t1v2, t2v0, t2v1, t2v2, &d[2].timeOfImpact, &u, &v);
	d[2].t1VertexThroughT2 = true;
	d[2].triangle1Alpha = 0.0;
	d[2].triangle1Beta = 1.0;
	d[2].triangle2Alpha = u;
	d[2].triangle2Beta = v;

	d[3].didOccur = calculateCcdContactPointTriangle(t2v0, t1v0, t1v1, t1v2, &d[3].timeOfImpact, &u, &v);
	d[3].t1VertexThroughT2 = false;
	d[3].triangle1Alpha = u;
	d[3].triangle1Beta = v;
	d[3].triangle2Alpha = 0.0;
	d[3].triangle2Beta = 0.0;

	d[4].didOccur = calculateCcdContactPointTriangle(t2v1, t1v0, t1v1, t1v2, &d[4].timeOfImpact, &u, &v);
	d[4].t1VertexThroughT2 = false;
	d[4].triangle1Alpha = u;
	d[4].triangle1Beta = v;
	d[4].triangle2Alpha = 1.0;
	d[4].triangle2Beta = 0.0;

	d[5].didOccur = calculateCcdContactPointTriangle(t2v2, t1v0, t1v1, t1v2, &d[5].timeOfImpact, &u, &v);
	d[5].t1VertexThroughT2 = false;
	d[5].triangle1Alpha = u;
	d[5].triangle1Beta = v;
	d[5].triangle2Alpha = 0.0;
	d[5].triangle2Beta = 1.0;

	std::sort(d.begin(), d.end(), [](const IntersectionData& a, const IntersectionData& b) {
		return a.timeOfImpact < b.timeOfImpact; });

	if (d[0].didOccur)
	{
		*earliestTimeOfImpact = d[0].timeOfImpact;
		*triangle1Alpha = d[0].triangle1Alpha;
		*triangle2Alpha = d[0].triangle2Alpha;
		*triangle1Beta = d[0].triangle1Beta;
		*triangle2Beta = d[0].triangle2Beta;
		*t1VertexThroughT2 = d[0].t1VertexThroughT2;
	}

	return d[0].didOccur;
}

}; // namespace Collision
}; // namespace SurgSim
