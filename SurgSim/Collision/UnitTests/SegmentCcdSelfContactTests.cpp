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

/// \file
/// Tests for the SegmentSegmentCcdCheck functions.

#include <array>

#include <gtest/gtest.h>

#include "SurgSim/Collision/CollisionPair.h"
#include "SurgSim/Collision/SegmentCcdSelfContact.h"
#include "SurgSim/DataStructures/AabbTree.h"
#include "SurgSim/DataStructures/Location.h"
#include "SurgSim/Math/SegmentMeshShape.h"

using SurgSim::DataStructures::Location;
using SurgSim::DataStructures::SegmentMeshPlain;
using SurgSim::Math::SegmentMeshShape;
using SurgSim::Math::Vector3d;

namespace SurgSim
{

namespace Collision
{

class MockSegmentCcdSelfContact : public SurgSim::Collision::SegmentCcdSelfContact
{
public:
	double maxTimePrecision(
		const std::array<SurgSim::Math::Vector3d, 2>& pt0Positions,
		const std::array<SurgSim::Math::Vector3d, 2>& pt1Positions,
		const std::array<SurgSim::Math::Vector3d, 2>& qt0Positions,
		const std::array<SurgSim::Math::Vector3d, 2>& qt1Positions,
		double effectiveThickness) const
	{
		return SegmentCcdSelfContact::maxTimePrecision(pt0Positions, pt1Positions,
				qt0Positions, qt1Positions,
				effectiveThickness);
	}

	bool isSameSegContactPoint(const Math::SegmentMeshShape& segmentShape,
							   size_t segId1, double s1, size_t segId2, double s2) const
	{
		return SegmentCcdSelfContact::isSameSegContactPoint(segmentShape, segId1, s1, segId2, s2);
	}

	bool findSegSegContact(const Math::SegmentMeshShape& segmentShape,
						   const std::list<std::shared_ptr<Contact>>& contacts,
						   double toi, Collision::CollisionDetectionType collisionType,
						   size_t segId1, double s1, size_t segId2, double s2, double timeEpsilon) const
	{
		return SegmentCcdSelfContact::findSegSegContact(
				   segmentShape, contacts, toi, collisionType, segId1, s1, segId2, s2, timeEpsilon);
	}

	bool detectExcessMovement(const SurgSim::Math::Vector3d& pt0,
							  const SurgSim::Math::Vector3d& pt1,
							  double threshold) const
	{
		return SegmentCcdSelfContact::detectExcessMovement(pt0, pt1, threshold);
	}

	bool removeInvalidCollisions(
		const Math::SegmentMeshShape& segmentA,
		const Math::SegmentMeshShape& segmentB,
		size_t id1, size_t id2) const
	{
		return SegmentCcdSelfContact::removeInvalidCollisions(segmentA, segmentB, id1, id2);
	}

	void getUniqueCandidates(
		const std::list<SurgSim::DataStructures::AabbTree::TreeNodePairType>& intersectionList,
		std::set<std::pair<size_t, size_t>>* segmentIdList) const
	{
		SegmentCcdSelfContact::getUniqueCandidates(intersectionList, segmentIdList);
	}

	bool detectCollision(
		const std::array<SurgSim::Math::Vector3d, 2>& pt0Positions,
		const std::array<SurgSim::Math::Vector3d, 2>& pt1Positions,
		const std::array<SurgSim::Math::Vector3d, 2>& qt0Positions,
		const std::array<SurgSim::Math::Vector3d, 2>& qt1Positions,
		double segmentRadius1, double segmentRadius2, double timePrecision,
		double* r, double* s, double* t,
		SurgSim::Math::Vector3d* pToQDir,
		SurgSim::Math::Vector3d* contactPtP,
		SurgSim::Math::Vector3d* contactPtQ) const
	{
		return SegmentCcdSelfContact::detectCollision(pt0Positions, pt1Positions, qt0Positions, qt1Positions,
				segmentRadius1, segmentRadius2, timePrecision,
				r, s, t, pToQDir, contactPtP, contactPtQ);
	}
};

class SegmentCcdSelfContactTests : public ::testing::Test
{
public:
	void SetUp()
	{
		MockSegmentCcdSelfContact m_selfContact;
	}

	std::shared_ptr<Contact> createContact(CollisionDetectionType type, double r, double s, size_t seg1, size_t seg2,
										   double toi)
	{
		std::pair<Location, Location> penetrationPoints;
		Math::Vector2d parametricCoordinateP(r, 1.0 - r);
		penetrationPoints.first.elementMeshLocalCoordinate.setValue(
			SurgSim::DataStructures::IndexedLocalCoordinate(seg1, parametricCoordinateP));
		Math::Vector2d parametricCoordinateQ(s, 1.0 - s);
		penetrationPoints.second.elementMeshLocalCoordinate.setValue(
			SurgSim::DataStructures::IndexedLocalCoordinate(seg2, parametricCoordinateQ));

		// Dummy up some values unless we need them
		Vector3d normal(1.0, 0.0, 0.0);
		Vector3d contactPoint(0.0, 0.0, 0.0);
		return std::make_shared<Contact>(type, 0.0, toi, contactPoint, normal, penetrationPoints);
	}

	std::shared_ptr<SegmentMeshShape> build(const Vector3d& start, const Vector3d& direction, double radius,
											size_t numVertices = 10) const
	{
		auto mesh = std::make_shared<SegmentMeshPlain>();

		// Add the vertices
		for (size_t i = 0; i < numVertices; ++i)
		{
			mesh->addVertex(SegmentMeshPlain::VertexType(start + direction * static_cast<double>(i)));
		}

		// Define the edges
		for (size_t i = 0; i < numVertices - 1; ++i)
		{
			std::array<size_t, 2> indices = {{i, i + 1}};
			mesh->addEdge(SegmentMeshPlain::EdgeType(indices));
		}

		return std::make_shared<SegmentMeshShape>(*mesh, radius);
	}

	std::shared_ptr<SegmentMeshShape> buildLoop(double zInit, double radius) const
	{
		auto mesh = std::make_shared<SegmentMeshPlain>();
		double zDelta = 2 * zInit / 10;
		double z = zInit;

		mesh->addVertex(SegmentMeshPlain::VertexType(Vector3d(1.0, 1.0, z)));
		z -= zDelta;
		mesh->addVertex(SegmentMeshPlain::VertexType(Vector3d(0.0, 0.0, z)));
		z -= zDelta;
		mesh->addVertex(SegmentMeshPlain::VertexType(Vector3d(-0.25, -0.25, z)));
		z -= zDelta;
		mesh->addVertex(SegmentMeshPlain::VertexType(Vector3d(-0.5, -0.5, z)));
		z -= zDelta;
		mesh->addVertex(SegmentMeshPlain::VertexType(Vector3d(-0.25, -0.75, z)));
		z -= zDelta;
		mesh->addVertex(SegmentMeshPlain::VertexType(Vector3d(0.0, -1.0, z)));
		z -= zDelta;
		mesh->addVertex(SegmentMeshPlain::VertexType(Vector3d(0.25, -0.75, z)));
		z -= zDelta;
		mesh->addVertex(SegmentMeshPlain::VertexType(Vector3d(0.5, -0.5, z)));
		z -= zDelta;
		mesh->addVertex(SegmentMeshPlain::VertexType(Vector3d(0.25, -0.25, z)));
		z -= zDelta;
		mesh->addVertex(SegmentMeshPlain::VertexType(Vector3d(0.0, 0.0, z)));
		z -= zDelta;
		mesh->addVertex(SegmentMeshPlain::VertexType(Vector3d(-1.0, 1.0, z)));

		// Define the edges
		for (size_t i = 0; i < 10; ++i)
		{
			std::array<size_t, 2> indices = {{i, i + 1}};
			mesh->addEdge(SegmentMeshPlain::EdgeType(indices));
		}

		return std::make_shared<SegmentMeshShape>(*mesh, radius);
	}

	MockSegmentCcdSelfContact m_selfContact;
};

TEST_F(SegmentCcdSelfContactTests, Initialization)
{
	ASSERT_NO_THROW(SegmentCcdSelfContact selfContact);
	SegmentCcdSelfContact selfContact;

	auto shapeTypes = selfContact.getShapeTypes();
	EXPECT_EQ(SurgSim::Math::SHAPE_TYPE_SEGMENTMESH, shapeTypes.first);
	EXPECT_EQ(SurgSim::Math::SHAPE_TYPE_SEGMENTMESH, shapeTypes.second);

	EXPECT_DOUBLE_EQ(1e-06, selfContact.getTimeMinPrecisionEpsilon());
	EXPECT_DOUBLE_EQ(1e-06, selfContact.getTimeMaxPrecisionEpsilon());
	EXPECT_DOUBLE_EQ(1e-09, selfContact.distanceEpsilon());

	EXPECT_ANY_THROW(selfContact.setTimeMinPrecisionEpsilon(0.0));
	EXPECT_ANY_THROW(selfContact.setTimeMinPrecisionEpsilon(-2.0e-06));
	EXPECT_ANY_THROW(selfContact.setTimeMaxPrecisionEpsilon(0.0));
	EXPECT_ANY_THROW(selfContact.setTimeMaxPrecisionEpsilon(-1.0e-05));

	selfContact.setTimeMinPrecisionEpsilon(2.0e-06);
	selfContact.setTimeMaxPrecisionEpsilon(1.0e-05);
	selfContact.setDistanceEpsilon(1.0e-06);

	EXPECT_DOUBLE_EQ(2e-06, selfContact.getTimeMinPrecisionEpsilon());
	EXPECT_DOUBLE_EQ(1.0e-05, selfContact.getTimeMaxPrecisionEpsilon());
	EXPECT_DOUBLE_EQ(1.0e-06, selfContact.distanceEpsilon());
};

TEST_F(SegmentCcdSelfContactTests, MaxTimePrecision)
{
	Math::Vector3d zeroPosition(0.0, 0.0, 0.0);
	Math::Vector3d changePos(1.0, 2.0, 3.0);
	Math::Vector3d changeNeg(-1.0, -2.0, -3.0);

	std::array<Math::Vector3d, 2> noChange = {zeroPosition, zeroPosition};
	std::array<Math::Vector3d, 2> change = {changePos, changeNeg};
	m_selfContact.setTimeMaxPrecisionEpsilon(1.0e-05);

	ASSERT_ANY_THROW(m_selfContact.maxTimePrecision(change, noChange, noChange, noChange, -0.5));

	EXPECT_DOUBLE_EQ(1.0e-05,
					 m_selfContact.maxTimePrecision(change, noChange, noChange, noChange, 0.5));
	EXPECT_DOUBLE_EQ(1.0e-05,
					 m_selfContact.maxTimePrecision(noChange, change, noChange, noChange, 0.5));
	EXPECT_DOUBLE_EQ(1.0e-05,
					 m_selfContact.maxTimePrecision(noChange, noChange, change, noChange, 0.5));
	EXPECT_DOUBLE_EQ(1.0e-05,
					 m_selfContact.maxTimePrecision(noChange, noChange, noChange, change, 0.5));

	EXPECT_DOUBLE_EQ(3.0e-05 / std::sqrt(14),
					 m_selfContact.maxTimePrecision(change, noChange, noChange, noChange, 3.0e-05));
	EXPECT_DOUBLE_EQ(3.0e-05 / std::sqrt(14),
					 m_selfContact.maxTimePrecision(noChange, change, noChange, noChange, 3.0e-05));
	EXPECT_DOUBLE_EQ(3.0e-05 / std::sqrt(14),
					 m_selfContact.maxTimePrecision(noChange, noChange, change, noChange, 3.0e-05));
	EXPECT_DOUBLE_EQ(3.0e-05 / std::sqrt(14),
					 m_selfContact.maxTimePrecision(noChange, noChange, noChange, change, 3.0e-05));

	EXPECT_DOUBLE_EQ(1.0e-06,
					 m_selfContact.maxTimePrecision(change, noChange, noChange, noChange, 3.0e-07));
	EXPECT_DOUBLE_EQ(1.0e-06,
					 m_selfContact.maxTimePrecision(noChange, change, noChange, noChange, 3.0e-07));
	EXPECT_DOUBLE_EQ(1.0e-06,
					 m_selfContact.maxTimePrecision(noChange, noChange, change, noChange, 3.0e-07));
	EXPECT_DOUBLE_EQ(1.0e-06,
					 m_selfContact.maxTimePrecision(noChange, noChange, noChange, change, 3.0e-07));
};

TEST_F(SegmentCcdSelfContactTests, IsSameSegContactPoint)
{
	std::shared_ptr<SegmentMeshShape> shape =
		build(Vector3d(-10.0, -10.0, -10.0), Vector3d(10.0, 10.0, 10.0), 0.5, 10);

	// Different segments, no shared endpoints
	EXPECT_FALSE(m_selfContact.isSameSegContactPoint(*shape, 1, 0.9, 7, 0.8));

	// Same segment, discrete contact points
	EXPECT_FALSE(m_selfContact.isSameSegContactPoint(*shape, 1, 0.9 - 1.0e-09, 1, 0.9 + 1.0e-09));

	// Same segment, same contact point
	EXPECT_TRUE(m_selfContact.isSameSegContactPoint(*shape, 1, 0.9 - 5.0e-10, 1, 0.9 + 4.9e-10));

	// Different segment, failure on distance from endpoint
	EXPECT_FALSE(m_selfContact.isSameSegContactPoint(*shape, 1, 2.0e-09, 0, 1.0));
	EXPECT_FALSE(m_selfContact.isSameSegContactPoint(*shape, 1, 1.0 - 2.0e-09, 2, 0.0));
	EXPECT_FALSE(m_selfContact.isSameSegContactPoint(*shape, 1, 0.0, 0, 1.0 - 2.0e-09));
	EXPECT_FALSE(m_selfContact.isSameSegContactPoint(*shape, 1, 1.0, 2, 2.0e-09));

	// Different segment, failure on adjacency
	EXPECT_FALSE(m_selfContact.isSameSegContactPoint(*shape, 1, 4.9e-10, 2, 1.0 - 5.0e-10));
	EXPECT_FALSE(m_selfContact.isSameSegContactPoint(*shape, 1, 1.0 - 5.0e-10, 0, 4.9e-10));
	EXPECT_FALSE(m_selfContact.isSameSegContactPoint(*shape, 1, 4.9e-10, 2, 1.0 - 5.0e-10));
	EXPECT_FALSE(m_selfContact.isSameSegContactPoint(*shape, 1, 1.0 - 5.0e-10, 0, 4.9e-10));

	// Different segment, same contact point
	EXPECT_TRUE(m_selfContact.isSameSegContactPoint(*shape, 1, 4.9e-10, 0, 1.0 - 5.0e-10));
	EXPECT_TRUE(m_selfContact.isSameSegContactPoint(*shape, 1, 1.0 - 5.0e-10, 2, 4.9e-10));
	EXPECT_TRUE(m_selfContact.isSameSegContactPoint(*shape, 1, 4.9e-10, 0, 1.0 - 5.0e-10));
	EXPECT_TRUE(m_selfContact.isSameSegContactPoint(*shape, 1, 1.0 - 5.0e-10, 2, 4.9e-10));
};

TEST_F(SegmentCcdSelfContactTests, FindSegSegContact)
{
	std::shared_ptr<SegmentMeshShape> shape =
		build(Vector3d(-10.0, -10.0, -10.0), Vector3d(10.0, 10.0, 10.0), 0.5, 10);

	{
		SCOPED_TRACE("Wrong collision type");
		std::list<std::shared_ptr<Contact>> contacts;
		contacts.emplace_back(createContact(
								  CollisionDetectionType::COLLISION_DETECTION_TYPE_DISCRETE, 0.8, 0.4, 1, 3, 0.25));

		EXPECT_FALSE(m_selfContact.findSegSegContact(*shape, contacts, 0.25,
					 CollisionDetectionType::COLLISION_DETECTION_TYPE_CONTINUOUS, 1, 0.8, 3, 0.4, 1.0e-06));
	}
	{
		SCOPED_TRACE("Wrong time");
		std::list<std::shared_ptr<Contact>> contacts;
		contacts.emplace_back(createContact(
								  CollisionDetectionType::COLLISION_DETECTION_TYPE_CONTINUOUS, 0.8, 0.4, 1, 3, 0.25));

		EXPECT_FALSE(m_selfContact.findSegSegContact(*shape, contacts, 0.25 - 2.0e-06,
					 CollisionDetectionType::COLLISION_DETECTION_TYPE_CONTINUOUS, 1, 0.8, 3, 0.4, 1.0e-06));
		EXPECT_FALSE(m_selfContact.findSegSegContact(*shape, contacts, 0.25 + 2.0e-06,
					 CollisionDetectionType::COLLISION_DETECTION_TYPE_CONTINUOUS, 1, 0.8, 3, 0.4, 1.0e-06));
	}
	{
		SCOPED_TRACE("Wrong segment ID");
		std::list<std::shared_ptr<Contact>> contacts;
		contacts.emplace_back(createContact(
								  CollisionDetectionType::COLLISION_DETECTION_TYPE_CONTINUOUS, 0.8, 0.4, 1, 3, 0.25));

		EXPECT_FALSE(m_selfContact.findSegSegContact(*shape, contacts, 0.25,
					 CollisionDetectionType::COLLISION_DETECTION_TYPE_CONTINUOUS, 2, 0.8, 3, 0.4, 1.0e-06));
		EXPECT_FALSE(m_selfContact.findSegSegContact(*shape, contacts, 0.25,
					 CollisionDetectionType::COLLISION_DETECTION_TYPE_CONTINUOUS, 1, 0.4, 4, 0.4, 1.0e-06));
	}
	{
		SCOPED_TRACE("Wrong parametric value");
		std::list<std::shared_ptr<Contact>> contacts;
		contacts.emplace_back(createContact(
								  CollisionDetectionType::COLLISION_DETECTION_TYPE_CONTINUOUS, 0.8, 0.4, 1, 3, 0.25));

		EXPECT_FALSE(m_selfContact.findSegSegContact(*shape, contacts, 0.25,
					 CollisionDetectionType::COLLISION_DETECTION_TYPE_CONTINUOUS, 1, 0.8 - 1.0e-03, 3, 0.4, 1.0e-06));
	}
	{
		SCOPED_TRACE("Same point, middle of segment");
		std::list<std::shared_ptr<Contact>> contacts;
		contacts.emplace_back(createContact(
								  CollisionDetectionType::COLLISION_DETECTION_TYPE_CONTINUOUS, 0.8, 0.4, 1, 3, 0.25));

		EXPECT_TRUE(m_selfContact.findSegSegContact(*shape, contacts, 0.25,
					CollisionDetectionType::COLLISION_DETECTION_TYPE_CONTINUOUS, 1, 0.8, 3, 0.4, 1.0e-06));
	}
	{
		SCOPED_TRACE("Same point, segment endpoints");
		std::list<std::shared_ptr<Contact>> contacts;
		contacts.emplace_back(createContact(
								  CollisionDetectionType::COLLISION_DETECTION_TYPE_CONTINUOUS, 1.0, 0.0, 1, 4, 0.25));

		EXPECT_TRUE(m_selfContact.findSegSegContact(*shape, contacts, 0.25,
					CollisionDetectionType::COLLISION_DETECTION_TYPE_CONTINUOUS, 2, 0.0, 3, 1.0, 1.0e-06));
	}
};

TEST_F(SegmentCcdSelfContactTests, DetectExcessMovement)
{
	Vector3d point1(1.0e-03 / 2.0, 1.0e-03 / 2.0, 1.0e-03 / 2.0);
	Vector3d point2(0.0, 0.0, 0.0);

	EXPECT_FALSE(m_selfContact.detectExcessMovement(point1, point2, 1.0e-03));
	EXPECT_FALSE(m_selfContact.detectExcessMovement(point2, point1, 1.0e-03));

	point2 = Vector3d(-1.0e-03, 0.0, 0.0);
	EXPECT_TRUE(m_selfContact.detectExcessMovement(point1, point2, 1.0e-03));
	EXPECT_TRUE(m_selfContact.detectExcessMovement(point2, point1, 1.0e-03));

	point2 = Vector3d(0.0, -1.0e-03, 0.0);
	EXPECT_TRUE(m_selfContact.detectExcessMovement(point1, point2, 1.0e-03));
	EXPECT_TRUE(m_selfContact.detectExcessMovement(point2, point1, 1.0e-03));

	point2 = Vector3d(0.0, 0.0, -1.0e-03);
	EXPECT_TRUE(m_selfContact.detectExcessMovement(point1, point2, 1.0e-03));
	EXPECT_TRUE(m_selfContact.detectExcessMovement(point2, point1, 1.0e-03));
};

TEST_F(SegmentCcdSelfContactTests, RemoveInvalidCollisions)
{
	std::shared_ptr<SegmentMeshShape> shapeT0 =
		buildLoop(1.0e-03, 1.0e-04);
	std::shared_ptr<SegmentMeshShape> shapeT1 =
		buildLoop(-1.0e-03, 1.0e-04);

	EXPECT_FALSE(m_selfContact.removeInvalidCollisions(*shapeT0, *shapeT1, 0, 9));
	EXPECT_FALSE(m_selfContact.removeInvalidCollisions(*shapeT0, *shapeT1, 1, 9));
	EXPECT_FALSE(m_selfContact.removeInvalidCollisions(*shapeT0, *shapeT1, 0, 8));
	EXPECT_FALSE(m_selfContact.removeInvalidCollisions(*shapeT0, *shapeT1, 1, 8));

	EXPECT_TRUE(m_selfContact.removeInvalidCollisions(*shapeT0, *shapeT1, 1, 1));
	EXPECT_TRUE(m_selfContact.removeInvalidCollisions(*shapeT0, *shapeT1, 0, 1));
	EXPECT_TRUE(m_selfContact.removeInvalidCollisions(*shapeT0, *shapeT1, 1, 2));
	EXPECT_TRUE(m_selfContact.removeInvalidCollisions(*shapeT0, *shapeT1, 1, 0));

	shapeT0->getVertex(0).position += Vector3d(10, 0, 0);
	EXPECT_TRUE(m_selfContact.removeInvalidCollisions(*shapeT0, *shapeT1, 0, 9));
	shapeT0->getVertex(0).position -= Vector3d(10, 0, 0);

	shapeT0->getVertex(1).position += Vector3d(10, 0, 0);
	EXPECT_TRUE(m_selfContact.removeInvalidCollisions(*shapeT0, *shapeT1, 0, 9));
	shapeT0->getVertex(1).position -= Vector3d(10, 0, 0);

	shapeT1->getVertex(0).position += Vector3d(10, 0, 0);
	EXPECT_TRUE(m_selfContact.removeInvalidCollisions(*shapeT0, *shapeT1, 0, 9));
	shapeT1->getVertex(0).position -= Vector3d(10, 0, 0);

	shapeT1->getVertex(1).position += Vector3d(10, 0, 0);
	EXPECT_TRUE(m_selfContact.removeInvalidCollisions(*shapeT0, *shapeT1, 0, 9));
	shapeT1->getVertex(1).position -= Vector3d(10, 0, 0);

	shapeT0->getVertex(9).position += Vector3d(10, 0, 0);
	EXPECT_TRUE(m_selfContact.removeInvalidCollisions(*shapeT0, *shapeT1, 0, 9));
	shapeT0->getVertex(9).position -= Vector3d(10, 0, 0);

	shapeT0->getVertex(10).position += Vector3d(10, 0, 0);
	EXPECT_TRUE(m_selfContact.removeInvalidCollisions(*shapeT0, *shapeT1, 0, 9));
	shapeT0->getVertex(10).position -= Vector3d(10, 0, 0);

	shapeT1->getVertex(9).position += Vector3d(10, 0, 0);
	EXPECT_TRUE(m_selfContact.removeInvalidCollisions(*shapeT0, *shapeT1, 0, 9));
	shapeT1->getVertex(9).position -= Vector3d(10, 0, 0);

	shapeT1->getVertex(10).position += Vector3d(10, 0, 0);
	EXPECT_TRUE(m_selfContact.removeInvalidCollisions(*shapeT0, *shapeT1, 0, 9));
	shapeT1->getVertex(10).position -= Vector3d(10, 0, 0);
};

TEST_F(SegmentCcdSelfContactTests, GetUniqueCandidates)
{
	std::shared_ptr<SegmentMeshShape> shapeT0 =
		buildLoop(1.0e-03, 1.0e-04);
	std::shared_ptr<SegmentMeshShape> shapeT1 =
		buildLoop(-1.0e-03, 1.0e-04);

	std::set<std::pair<size_t, size_t>> segmentIdList;
	std::list<SurgSim::DataStructures::AabbTree::TreeNodePairType> intersectionList
		= shapeT0->getAabbTree()->spatialJoin(*(shapeT1->getAabbTree()));
	m_selfContact.getUniqueCandidates(intersectionList, &segmentIdList);
	EXPECT_EQ(6, segmentIdList.size());
	EXPECT_TRUE(std::find(segmentIdList.begin(), segmentIdList.end(),
						  std::pair<size_t, size_t>(0, 9)) != segmentIdList.end());
	EXPECT_TRUE(std::find(segmentIdList.begin(), segmentIdList.end(),
						  std::pair<size_t, size_t>(0, 8)) != segmentIdList.end());
	EXPECT_TRUE(std::find(segmentIdList.begin(), segmentIdList.end(),
						  std::pair<size_t, size_t>(1, 9)) != segmentIdList.end());
	EXPECT_TRUE(std::find(segmentIdList.begin(), segmentIdList.end(),
						  std::pair<size_t, size_t>(1, 8)) != segmentIdList.end());
	EXPECT_TRUE(std::find(segmentIdList.begin(), segmentIdList.end(),
						  std::pair<size_t, size_t>(4, 5)) != segmentIdList.end());
	EXPECT_TRUE(std::find(segmentIdList.begin(), segmentIdList.end(),
						  std::pair<size_t, size_t>(5, 6)) != segmentIdList.end());

	EXPECT_FALSE(std::find(segmentIdList.begin(), segmentIdList.end(),
						   std::pair<size_t, size_t>(1, 7)) != segmentIdList.end());
};

TEST_F(SegmentCcdSelfContactTests, DetectCollision)
{
	std::shared_ptr<SegmentMeshShape> shapeT0 =
		buildLoop(1.0e-03, 1.0e-04);
	std::shared_ptr<SegmentMeshShape> shapeT1 =
		buildLoop(-1.0e-03, 1.0e-04);

	double r;
	double s;
	double t;
	Vector3d pToQDir;
	Vector3d contactPtP;
	Vector3d contactPtQ;

	{
		SCOPED_TRACE("Successful detection");
		std::array<Math::Vector3d, 2> pt0Positions = shapeT0->getEdgePositions(0);
		std::array<Math::Vector3d, 2> pt1Positions = shapeT1->getEdgePositions(0);
		std::array<Math::Vector3d, 2> qt0Positions = shapeT0->getEdgePositions(9);
		std::array<Math::Vector3d, 2> qt1Positions = shapeT1->getEdgePositions(9);

		EXPECT_TRUE(m_selfContact.detectCollision(pt0Positions, pt1Positions, qt0Positions, qt1Positions,
					1.0e-04, 1.0e-04, 1.0e-09,
					&r, &s, &t, &pToQDir, &contactPtP, &contactPtQ));
		EXPECT_GT(1.0e-09, std::abs(r - 1.0));
		EXPECT_GT(1.0e-09, std::abs(s));
		EXPECT_GT(2.0e-09, std::abs(t - 0.4375));
		EXPECT_TRUE(pToQDir.normalized().isApprox(Vector3d(0.0, 0.0, -1.0), 1.0e-05));
		EXPECT_TRUE(contactPtP.isApprox(Vector3d(0.0, 0.0, 1.0e-04), 1.0e-05));
		EXPECT_TRUE(contactPtQ.isApprox(Vector3d(0.0, 0.0, -1.0e-04), 1.0e-05));

		EXPECT_TRUE(m_selfContact.detectCollision(pt0Positions, pt1Positions, qt0Positions, qt1Positions,
					0.0, 0.0, 1.0e-09,
					&r, &s, &t, &pToQDir, &contactPtP, &contactPtQ));
		EXPECT_GT(1.0e-09, std::abs(r - 1.0));
		EXPECT_GT(1.0e-09, std::abs(s));
		EXPECT_GT(2.0e-03, std::abs(t - 0.5));
		EXPECT_GT(1.0e-09, pToQDir.norm());
		EXPECT_GT(1.0e-09, (contactPtP - Vector3d(0.0, 0.0, 5.0e-10)).norm());
		EXPECT_GT(1.0e-09, (contactPtQ - Vector3d(0.0, 0.0, 5.0e-10)).norm());
	}
	{
		SCOPED_TRACE("Detection at T=0 (before movement)");
		std::array<Math::Vector3d, 2> pt0Positions = shapeT0->getEdgePositions(4);
		std::array<Math::Vector3d, 2> pt1Positions = shapeT1->getEdgePositions(4);
		std::array<Math::Vector3d, 2> qt0Positions = shapeT0->getEdgePositions(5);
		std::array<Math::Vector3d, 2> qt1Positions = shapeT1->getEdgePositions(5);

		EXPECT_TRUE(m_selfContact.detectCollision(pt0Positions, pt1Positions, qt0Positions, qt1Positions,
					1.0e-04, 1.0e-04, 1.0e-03,
					&r, &s, &t, &pToQDir, &contactPtP, &contactPtQ));
		EXPECT_GT(1.0e-09, std::abs(r - 1.0));
		EXPECT_GT(1.0e-09, std::abs(s));
		EXPECT_GT(1.0e-09, std::abs(t));
		EXPECT_GT(1.0e-09, pToQDir.norm());
		EXPECT_TRUE(contactPtP.isApprox(Vector3d(0.0, -1.0, 0.0), 1.0e-05));
		EXPECT_TRUE(contactPtQ.isApprox(Vector3d(0.0, -1.0, 0.0), 1.0e-05));

		EXPECT_TRUE(m_selfContact.detectCollision(pt0Positions, pt1Positions, qt0Positions, qt1Positions,
					0.0, 0.0, 1.0e-03,
					&r, &s, &t, &pToQDir, &contactPtP, &contactPtQ));
		EXPECT_GT(1.0e-09, std::abs(r - 1.0));
		EXPECT_GT(1.0e-09, std::abs(s));
		EXPECT_GT(1.0e-09, std::abs(t));
		EXPECT_GT(1.0e-09, pToQDir.norm());
		EXPECT_TRUE(contactPtP.isApprox(Vector3d(0.0, -1.0, 0.0), 1.0e-05));
		EXPECT_TRUE(contactPtQ.isApprox(Vector3d(0.0, -1.0, 0.0), 1.0e-05));
	}
	{
		SCOPED_TRACE("No detection");
		std::array<Math::Vector3d, 2> pt0Positions = shapeT0->getEdgePositions(1);
		std::array<Math::Vector3d, 2> pt1Positions = shapeT1->getEdgePositions(1);
		std::array<Math::Vector3d, 2> qt0Positions = shapeT0->getEdgePositions(7);
		std::array<Math::Vector3d, 2> qt1Positions = shapeT1->getEdgePositions(7);

		EXPECT_FALSE(m_selfContact.detectCollision(pt0Positions, pt1Positions, qt0Positions, qt1Positions,
					 1.0e-04, 1.0e-04, 1.0e-03,
					 &r, &s, &t, &pToQDir, &contactPtP, &contactPtQ));
		EXPECT_FALSE(m_selfContact.detectCollision(pt0Positions, pt1Positions, qt0Positions, qt1Positions,
					 0.0, 0.0, 1.0e-03,
					 &r, &s, &t, &pToQDir, &contactPtP, &contactPtQ));
	}
};
TEST_F(SegmentCcdSelfContactTests, CalculateContact)
{
	{
		SCOPED_TRACE("Successful detection");
		std::shared_ptr<SegmentMeshShape> shapeT0 =
			buildLoop(1.0e-03, 1.0e-04);
		std::shared_ptr<SegmentMeshShape> shapeT1 =
			buildLoop(-1.0e-03, 1.0e-04);

		m_selfContact.setDistanceEpsilon(1.0e-06);
		auto transformP = Math::RigidTransform3d::Identity();
		transformP.translate(Vector3d(1.0, 2.0, 3.0));

		auto transformQ = Math::RigidTransform3d::Identity();
		transformQ.translate(Vector3d(3.0, 4.0, 5.0));

		auto collisionList = m_selfContact.calculateContact(*shapeT0, transformP,
							 *shapeT1, transformQ);

		EXPECT_EQ(1, collisionList.size());
		std::shared_ptr<Collision::Contact> contacted = *(collisionList.begin());
		EXPECT_EQ(CollisionDetectionType::COLLISION_DETECTION_TYPE_CONTINUOUS, contacted->type);
		EXPECT_GT(2.0 * m_selfContact.getTimeMinPrecisionEpsilon(), std::abs(contacted->time - 0.4375));
		EXPECT_GT(1.0e-08, contacted->contact.norm());
		EXPECT_GT(1.0e-04, (contacted->normal.normalized() - Vector3d(0.0, 0.0, -1.0)).norm());
		auto contactP = contacted->penetrationPoints.first.rigidLocalPosition.getValue();
		EXPECT_TRUE((contactP.isApprox(transformP.inverse() *
									   (contacted->contact + Vector3d(0.0, 0.0, 0.0001)), 1.0e-06)));
		auto contactQ = contacted->penetrationPoints.second.rigidLocalPosition.getValue();
		EXPECT_TRUE((contactQ.isApprox(transformQ.inverse() *
									   (contacted->contact + Vector3d(0.0, 0.0, -0.0001)), 1.0e-06)));
		EXPECT_GT(1.0e-10, contacted->depth);
	}
};

}; // namespace Collision
}; // namespace SurgSim
