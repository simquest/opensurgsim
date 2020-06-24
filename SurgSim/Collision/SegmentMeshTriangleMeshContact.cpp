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

#include "SurgSim/Collision/SegmentMeshTriangleMeshContact.h"

#include "SurgSim/Collision/CollisionPair.h"
#include "SurgSim/Collision/Representation.h"
#include "SurgSim/DataStructures/AabbTree.h"
#include "SurgSim/DataStructures/AabbTreeNode.h"
#include "SurgSim/DataStructures/IndexedLocalCoordinate.h"
#include "SurgSim/DataStructures/TriangleMesh.h"
#include "SurgSim/Math/Geometry.h"
#include "SurgSim/Math/MeshShape.h"
#include "SurgSim/Math/RigidTransform.h"
#include "SurgSim/Math/SegmentMeshShape.h"

using SurgSim::DataStructures::Location;
using SurgSim::DataStructures::TriangleMesh;
using SurgSim::Math::MeshShape;
using SurgSim::Math::RigidTransform3d;
using SurgSim::Math::SegmentMeshShape;
using SurgSim::Math::Vector3d;


namespace SegmentMeshTriangleMesh
{

bool isThisContactADuplicate(
	const std::shared_ptr<SurgSim::Collision::Contact>& newContact,
	const std::list<std::shared_ptr<SurgSim::Collision::Contact>>& contacts)
{
	for (const auto& contact : contacts)
	{
		if (*newContact == *contact)
		{
			return true;
		}
	}

	return false;
}

}
namespace SurgSim
{
namespace Collision
{

std::pair<int, int> SegmentMeshTriangleMeshContact::getShapeTypes()
{
	return std::pair<int, int>(SurgSim::Math::SHAPE_TYPE_SEGMENTMESH, SurgSim::Math::SHAPE_TYPE_MESH);
}

std::list<std::shared_ptr<Contact>> SegmentMeshTriangleMeshContact::calculateDcdContact(
									 const Math::SegmentMeshShape& segmentMeshShape,
									 const Math::RigidTransform3d& segmentMeshPose,
									 const Math::MeshShape& triangleMeshShape,
									 const Math::RigidTransform3d& triangleMeshPose) const
{
	std::list<std::shared_ptr<Contact>> contacts;

	auto intersectionList = segmentMeshShape.getAabbTree()->spatialJoin(*triangleMeshShape.getAabbTree());

	double radius = segmentMeshShape.getRadius();
	double depth = 0.0;
	Vector3d normal;
	Vector3d penetrationPointCapsule, penetrationPointTriangle, penetrationPointCapsuleAxis;

	std::vector<size_t> edgeList;
	std::vector<size_t> triangleList;

	for (const auto& intersection : intersectionList)
	{
		DataStructures::AabbTreeNode* nodeSegment = intersection.first;
		DataStructures::AabbTreeNode* nodeTriangle = intersection.second;

		edgeList.clear();
		triangleList.clear();

		nodeSegment->getIntersections(nodeTriangle->getAabb(), &edgeList);
		nodeTriangle->getIntersections(nodeSegment->getAabb(), &triangleList);

		for (auto i = triangleList.begin(); i != triangleList.end(); ++i)
		{
			const Vector3d& normalTriangle = triangleMeshShape.getNormal(*i);
			if (normalTriangle.isZero())
			{
				continue;
			}

			const auto& verticesTriangle = triangleMeshShape.getTrianglePositions(*i);

			for (auto j = edgeList.begin(); j != edgeList.end(); ++j)
			{
				const auto& verticesSegment = segmentMeshShape.getEdgePositions(*j);

				try
				{
					// Check if the triangle and capsule intersect.
					if (SurgSim::Math::calculateContactTriangleCapsule(
						verticesTriangle[0], verticesTriangle[1], verticesTriangle[2], normalTriangle,
						verticesSegment[0], verticesSegment[1], radius,
						&depth, &penetrationPointTriangle, &penetrationPointCapsule, &normal,
						&penetrationPointCapsuleAxis))
					{
						// Create the contact.
						std::pair<Location, Location> penetrationPoints;
						SurgSim::Math::Vector2d barycentricCoordinate2;
						SurgSim::Math::barycentricCoordinates(penetrationPointCapsuleAxis,
							verticesSegment[0],
							verticesSegment[1],
							&barycentricCoordinate2);
						penetrationPoints.first.elementMeshLocalCoordinate.setValue(
							SurgSim::DataStructures::IndexedLocalCoordinate(*j, barycentricCoordinate2));
						penetrationPoints.first.rigidLocalPosition.setValue(
							segmentMeshPose.inverse() * penetrationPointCapsuleAxis);

						Vector3d barycentricCoordinate;
						SurgSim::Math::barycentricCoordinates(penetrationPointTriangle,
							verticesTriangle[0],
							verticesTriangle[1],
							verticesTriangle[2],
							normalTriangle,
							&barycentricCoordinate);

						penetrationPoints.second.triangleMeshLocalCoordinate.setValue(
							SurgSim::DataStructures::IndexedLocalCoordinate(*i, barycentricCoordinate));
						penetrationPoints.second.rigidLocalPosition.setValue(
							triangleMeshPose.inverse() * penetrationPointTriangle);

						// Create the contact.
						contacts.push_back(std::make_shared<Contact>(COLLISION_DETECTION_TYPE_DISCRETE,
							std::abs(depth) +
							(penetrationPointCapsule - penetrationPointCapsuleAxis).dot(normal),
							1.0, Vector3d::Zero(), -normal, penetrationPoints));
					}
				}
				catch (std::exception e)
				{
					SURGSIM_LOG_CRITICAL(SurgSim::Framework::Logger::getLogger("Collision")) <<
						__func__ << " " << __LINE__ << ": Failed calculateContactTriangleCapsule for triangle ID: " <<
						*i << " and segment ID: " << *j;
					throw e;
				}
			}
		}
	}

	return contacts;
}

std::list<std::shared_ptr<Contact>>SegmentMeshTriangleMeshContact::calculateCcdContact(
									 const Math::SegmentMeshShape& shape1AtTime0,
									 const Math::RigidTransform3d& pose1AtTime0,
									 const Math::SegmentMeshShape& shape1AtTime1,
									 const Math::RigidTransform3d& pose1AtTime1,
									 const Math::MeshShape& shape2AtTime0,
									 const Math::RigidTransform3d& pose2AtTime0,
									 const Math::MeshShape& shape2AtTime1,
									 const Math::RigidTransform3d& pose2AtTime1) const
{
	using SegmentMeshTriangleMesh::isThisContactADuplicate;
	using Math::calculateCcdContactSegmentSegment;
	using Math::calculateCcdContactPointTriangle;
	double epsilon = Math::Geometry::DistanceEpsilon;

	std::list<std::shared_ptr<Contact>> contacts;

	// This code is not tested for SegmentMeshes on Rigids, warn the user !
	SURGSIM_LOG_ONCE_IF(! pose1AtTime0.isApprox(Math::RigidTransform3d::Identity()) &&
						! pose1AtTime1.isApprox(Math::RigidTransform3d::Identity()),
						SurgSim::Framework::Logger::getLogger("Collision"), SEVERE)
			<< "It looks like you're using the SegmentMesh with a rigid object under CCD, the "
			<< "SegmentMeshTriangleMeshContact is not tested for this case.";


	SURGSIM_ASSERT(shape1AtTime0.getNumEdges() > 0);
	SURGSIM_ASSERT(shape1AtTime0.getNumEdges() == shape1AtTime1.getNumEdges());
	SURGSIM_ASSERT(shape2AtTime0.getNumTriangles() > 0);
	SURGSIM_ASSERT(shape2AtTime0.getNumTriangles() == shape2AtTime1.getNumTriangles());

	for (size_t edgeId = 0; edgeId < shape1AtTime0.getNumEdges(); edgeId++)
	{
		auto edgeT0 = shape1AtTime0.getEdge(edgeId);
		auto edgeT1 = shape1AtTime1.getEdge(edgeId);

		SURGSIM_ASSERT(edgeT0.verticesId == edgeT1.verticesId) << "Edges are different:\n" <<
				"(" << edgeT0.verticesId[0] << "," << edgeT0.verticesId[1] << ")\n" <<
				"(" << edgeT1.verticesId[0] << "," << edgeT1.verticesId[1] << ")\n" <<
				"edgeT0.valid = " << edgeT0.isValid << "\nedgeT1.valid = " << edgeT1.isValid;

		std::pair<Math::Vector3d, Math::Vector3d> sv0 = std::make_pair(
					shape1AtTime0.getVertexPosition(edgeT0.verticesId[0]),
					shape1AtTime1.getVertexPosition(edgeT1.verticesId[0]));
		std::pair<Math::Vector3d, Math::Vector3d> sv1 = std::make_pair(
					shape1AtTime0.getVertexPosition(edgeT0.verticesId[1]),
					shape1AtTime1.getVertexPosition(edgeT1.verticesId[1]));
		Math::Aabbd segmentAabb;
		segmentAabb.extend(sv0.first);
		segmentAabb.extend(sv0.second);
		segmentAabb.extend(sv1.first);
		segmentAabb.extend(sv1.second);

		for (size_t triangleId = 0; triangleId < shape2AtTime0.getNumTriangles(); triangleId++)
		{
			auto triangleT0 = shape2AtTime0.getTriangle(triangleId);
			auto triangleT1 = shape2AtTime1.getTriangle(triangleId);

			SURGSIM_ASSERT(triangleT0.verticesId == triangleT1.verticesId) << "Triangles are different:\n" <<
					"(" << triangleT0.verticesId[0] << "," << triangleT0.verticesId[1] << "," <<
					triangleT0.verticesId[2] << ")\n" <<
					"(" << triangleT1.verticesId[0] << "," << triangleT1.verticesId[1] << "," <<
					triangleT1.verticesId[2] << ")\n" <<
					"triangleT0.valid = " << triangleT0.isValid << "\ntriangleT1.valid = " << triangleT1.isValid;

			std::pair<Math::Vector3d, Math::Vector3d> tv0 = std::make_pair(
						shape2AtTime0.getVertexPosition(triangleT0.verticesId[0]),
						shape2AtTime1.getVertexPosition(triangleT1.verticesId[0]));
			std::pair<Math::Vector3d, Math::Vector3d> tv1 = std::make_pair(
						shape2AtTime0.getVertexPosition(triangleT0.verticesId[1]),
						shape2AtTime1.getVertexPosition(triangleT1.verticesId[1]));
			std::pair<Math::Vector3d, Math::Vector3d> tv2 = std::make_pair(
						shape2AtTime0.getVertexPosition(triangleT0.verticesId[2]),
						shape2AtTime1.getVertexPosition(triangleT1.verticesId[2]));

			Math::Aabbd triangleAabb;
			triangleAabb.extend(tv0.first);
			triangleAabb.extend(tv0.second);
			triangleAabb.extend(tv1.first);
			triangleAabb.extend(tv1.second);
			triangleAabb.extend(tv2.first);
			triangleAabb.extend(tv2.second);

			if (!SurgSim::Math::doAabbIntersect(segmentAabb, triangleAabb))
			{
				continue;
			}

			double earliestTimeOfImpact = std::numeric_limits<double>::max();
			double segmentAlpha = -1.0;  //!< Barycentric coordinates of P in the segment sv0sv1
			//!< P = sv0 + segmentAlpha.sv0sv1
			double triangleAlpha = -1.0;  //!< Barycentric coordinates of P in triangle tv0tv1tv2
			double triangleBeta = -1.0;   //!< P = tv0 + triangleAlpha.tv0tv1 + triangleBeta.tv0tv2

			// Check collision at time t = 0
			Math::Vector3d pt;
			Math::Vector3d tn = ((tv1.first - tv0.first).cross(tv2.first - tv0.first));
			if (tn.norm() < Math::Geometry::DistanceEpsilon)
			{
				SURGSIM_LOG_WARNING(Framework::Logger::getLogger("SegmentMeshTriangleMeshContact")) <<
						"The triangle mesh contains a degenerate triangle (null normal)";
			}
			tn.normalize();
			bool segmentSegmentCcdFound = false;

			if (Math::doesCollideSegmentTriangle<Math::Vector3d::Scalar, Math::Vector3d::Options>(
					sv0.first, sv1.first,
					tv0.first, tv1.first, tv2.first,
					tn,
					&pt))
			{
				Math::Vector2d baryCoordSegment;
				Math::Vector3d baryCoordTriangle;
				if (!Math::barycentricCoordinates(pt, sv0.first, sv1.first, &baryCoordSegment))
				{
					SURGSIM_LOG_WARNING(Framework::Logger::getLogger("SegmentMeshTriangleMeshContact")) <<
							"[t=0] Could not deduce the barycentric coordinate of (" << pt.transpose() <<
							") in the segment (" << sv0.first.transpose() << ")  (" << sv1.first.transpose() << ")";
				}
				if (!Math::barycentricCoordinates(pt, tv0.first, tv1.first, tv2.first, &baryCoordTriangle))
				{
					SURGSIM_LOG_WARNING(Framework::Logger::getLogger("SegmentMeshTriangleMeshContact")) <<
							"[t=0] Could not deduce the barycentric coordinate of (" << pt.transpose() <<
							") in the triangle (" << tv0.first.transpose() << ")  (" << tv1.first.transpose() <<
							") (" << tv2.first.transpose() << ")";
				}
				segmentSegmentCcdFound = false;
				earliestTimeOfImpact = 0.0;
				segmentAlpha = baryCoordSegment[1];
				triangleAlpha = baryCoordTriangle[1];
				triangleBeta = baryCoordTriangle[2];
			}
			else
			{
				// No collision at time t = 0, let's look for collision in the interval ]0..1]

				// Calculate Segment/Segment ccd
				double timeOfImpact;
				double sFactor, tFactor;
				if (calculateCcdContactSegmentSegment(sv0, sv1, tv0, tv1, &timeOfImpact, &sFactor, &tFactor))
				{
					if (timeOfImpact < earliestTimeOfImpact)
					{
						segmentSegmentCcdFound = true;
						earliestTimeOfImpact = timeOfImpact;
						segmentAlpha = sFactor;
						triangleAlpha = tFactor;
						triangleBeta = 0.0;
					}
				}

				if (calculateCcdContactSegmentSegment(sv0, sv1, tv1, tv2, &timeOfImpact, &sFactor, &tFactor))
				{
					if (timeOfImpact < earliestTimeOfImpact)
					{
						segmentSegmentCcdFound = true;
						earliestTimeOfImpact = timeOfImpact;
						segmentAlpha = sFactor;
						triangleAlpha = 1.0 - tFactor; // P = P0 + P0P1.(1 - tFactor) + P0P2.tFactor
						triangleBeta = tFactor;
					}
				}

				if (calculateCcdContactSegmentSegment(sv0, sv1, tv2, tv0, &timeOfImpact, &sFactor, &tFactor))
				{
					if (timeOfImpact < earliestTimeOfImpact)
					{
						segmentSegmentCcdFound = true;
						earliestTimeOfImpact = timeOfImpact;
						segmentAlpha = sFactor;
						triangleAlpha = 0.0; // P = P0 + P0P2.(1 - tFactor)
						triangleBeta = 1.0 - tFactor;
					}
				}

				// Calculate Point/Triangle ccd
				double u, v;
				if (calculateCcdContactPointTriangle(sv0, tv0, tv1, tv2, &timeOfImpact, &u, &v))
				{
					if (timeOfImpact < earliestTimeOfImpact)
					{
						segmentSegmentCcdFound = false;
						earliestTimeOfImpact = timeOfImpact;
						segmentAlpha = 0.0;
						triangleAlpha = u;
						triangleBeta = v;
					}
				}

				if (calculateCcdContactPointTriangle(sv1, tv0, tv1, tv2, &timeOfImpact, &u, &v))
				{
					if (timeOfImpact < earliestTimeOfImpact)
					{
						segmentSegmentCcdFound = false;
						earliestTimeOfImpact = timeOfImpact;
						segmentAlpha = 1.0;
						triangleAlpha = u;
						triangleBeta = v;
					}
				}
			}

			// False positive from the AABB, no collision found
			if (earliestTimeOfImpact == std::numeric_limits<double>::max())
			{
				continue;
			}

			SURGSIM_ASSERT(segmentAlpha >= -epsilon && segmentAlpha <= (1.0 + epsilon)) <<
					"earliestTimeOfImpact = " << earliestTimeOfImpact <<
					"; segmentAlpha = " << segmentAlpha;
			SURGSIM_ASSERT(triangleAlpha >= -epsilon && triangleBeta >= -epsilon &&
						   triangleAlpha + triangleBeta <= (1.0 + epsilon + epsilon)) <<
								   "earliestTimeOfImpact = " << earliestTimeOfImpact <<
								   "; triangleAlpha = " << triangleAlpha <<
								   "; triangleBeta = " << triangleBeta <<
								   "; triangleAlpha + triangleBeta = " << triangleAlpha + triangleBeta;


			Math::Vector3d T, Tn;
			double penentrationDepthAtT1;
			Math::Vector3d S = Math::interpolate(sv0.second, sv1.second, segmentAlpha);
			{
				Math::Vector3d T0T1 = tv1.second - tv0.second;
				Math::Vector3d T0T2 = tv2.second - tv0.second;
				T = tv0.second + triangleAlpha * T0T1 + triangleBeta * T0T2;
				Tn = (segmentSegmentCcdFound) ? (T - S).normalized() : T0T1.cross(T0T2).normalized();
				penentrationDepthAtT1 = (S - T).dot(Tn);
			}

			Math::Vector segmentBaryCoord(2);
			segmentBaryCoord << 1.0 - segmentAlpha, segmentAlpha;
			DataStructures::IndexedLocalCoordinate localCoordinateSegment(edgeId, segmentBaryCoord);
			DataStructures::Location locationSegment(localCoordinateSegment, Location::ELEMENT);
			locationSegment.rigidLocalPosition = S;

			Math::Vector triangleBaryCoord(3);
			triangleBaryCoord << 1.0 - triangleAlpha - triangleBeta, triangleAlpha, triangleBeta;
			DataStructures::IndexedLocalCoordinate localCoordinateTriangle(triangleId, triangleBaryCoord);
			// The location related to the TriangleMesh can carry a TRIANGLE information
			// e.g. part of a Mass-Spring with deformable triangulation for collision
			DataStructures::Location locationTriangle(localCoordinateTriangle, Location::TRIANGLE);
			// The location related to the TriangleMesh can carry an ELEMENT information
			// e.g. part of an Fem2D for example
			locationTriangle.elementMeshLocalCoordinate = locationTriangle.triangleMeshLocalCoordinate;
			// The location related to the TriangleMesh can carry a RIGID LOCAL POSITION information
			// e.g. part of a rigid body
			locationTriangle.rigidLocalPosition = pose2AtTime1.inverse() * T;

			auto contact = std::make_shared<Contact>(
							   COLLISION_DETECTION_TYPE_CONTINUOUS,
							   penentrationDepthAtT1,
							   earliestTimeOfImpact,
							   T,
							   Tn,
							   std::make_pair(locationSegment, locationTriangle));

			if (!isThisContactADuplicate(contact, contacts))
			{
				contacts.push_back(std::move(contact));
			}
		}
	}

	return contacts;
}

}; // namespace Collision
}; // namespace SurgSim
