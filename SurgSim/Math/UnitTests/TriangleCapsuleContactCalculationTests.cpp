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

#include <gtest/gtest.h>

#include "SurgSim/Math/Geometry.h"
#include "SurgSim/Math/UnitTests/MockCapsule.h"
#include "SurgSim/Math/UnitTests/MockTriangle.h"

namespace SurgSim
{
namespace Math
{

class TriangleCapsuleContactCalculationTest : public ::testing::Test
{
protected:
	typedef std::tuple<std::string,	// String to describe the scenario.
		MockTriangle,	// The triangle.
		MockCapsule,	// The capsule.
		bool,		// Flag to indicate if the two shapes are expected to be found intersecting.
		bool,		// Flag to indicate if expected contact info is available to check against.
		Vector3d,	// Expected penetration point in the triangle.
		Vector3d>	// Expected penetration point in the capsule.
		TriangleCapsuleTestCase;

	SurgSim::Math::RigidTransform3d buildRigidTransform(double angle, double axisX, double axisY, double axisZ,
		double translationX, double translationY, double translationZ)
	{
		using SurgSim::Math::makeRigidTransform;
		using SurgSim::Math::makeRotationQuaternion;
		return makeRigidTransform(makeRotationQuaternion(angle, Vector3d(axisX, axisY, axisZ).normalized()),
									Vector3d(translationX, translationY, translationZ));
	}

	void SetUp() override
	{
		m_transforms.push_back(buildRigidTransform(0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0));
		m_transforms.push_back(buildRigidTransform(1.234, 17.04, 2.047, 3.052, 23.34, 42.45, 83.68));
		m_transforms.push_back(buildRigidTransform(-5.34, 41.03, -2.52, -3.84, -3.45, 66.47, 29.34));
		m_transforms.push_back(buildRigidTransform(0.246, -9.42, -4.86, 2.469, 37.68, -34.6, -17.1));
		m_transforms.push_back(buildRigidTransform(-0.85, 3.344, 8.329, -97.4, 9.465, 0.275, -95.9));
	}

	void checkEqual(const Vector3d& v1, const Vector3d& v2)
	{
		if (v1.isZero(Geometry::DistanceEpsilon))
		{
			EXPECT_TRUE(v2.isZero(Geometry::DistanceEpsilon));
		}
		else
		{
			EXPECT_TRUE(v1.isApprox(v2, Geometry::DistanceEpsilon));
		}
	}

	void testTriangleCapsuleContactCalculation(const TriangleCapsuleTestCase& data)
	{
		MockTriangle t = std::get<1>(data);
		MockCapsule c = std::get<2>(data);
		bool contactExpected = std::get<3>(data);
		bool checkForPenetrationPoints = std::get<4>(data);
		Vector3d expectedTPoint = std::get<5>(data);
		Vector3d expectedCPoint = std::get<6>(data);

		double expectedPenetrationDepth = (expectedCPoint - expectedTPoint).norm();
		double penetrationDepth;
		Vector3d tPoint, cPoint, normal;
		bool contactFound = false;
		std::string traceMessage[12] = {"Triangle vs Capsule",
			"Triangle (vertices shifted once) vs Capsule",
			"Triangle (vertices shifted twice) vs Capsule",
			"Triangle vs Capsule (vertices interchanged)",
			"Triangle (vertices shifted once) vs Capsule (vertices interchanged)",
			"Triangle (vertices shifted twice) vs Capsule (vertices interchanged)",
			"Reversed Triangle vs Capsule",
			"Reversed Triangle (vertices shifted once) vs Capsule",
			"Reversed Triangle (vertices shifted twice) vs Capsule",
			"Reversed Triangle vs Capsule (vertices interchanged)",
			"Reversed Triangle (vertices shifted once) vs Capsule (vertices interchanged)",
			"Reversed Triangle (vertices shifted twice) vs Capsule (vertices interchanged)"
		};
		for (int count = 0; count < 12; ++count)
		{
			SCOPED_TRACE(std::get<0>(data) + ": " + traceMessage[count]);

			switch (count)
			{
			case 0:
				EXPECT_NO_THROW(
					contactFound = calculateContactTriangleCapsule(t.v0, t.v1, t.v2, t.n, c.v0, c.v1, c.r,
					&penetrationDepth, &tPoint, &cPoint, &normal));
				break;
			case 1:
				EXPECT_NO_THROW(
					contactFound = calculateContactTriangleCapsule(t.v1, t.v2, t.v0, t.n, c.v0, c.v1, c.r,
					&penetrationDepth, &tPoint, &cPoint, &normal));
				break;
			case 2:
				EXPECT_NO_THROW(
					contactFound = calculateContactTriangleCapsule(t.v2, t.v0, t.v1, t.n, c.v0, c.v1, c.r,
					&penetrationDepth, &tPoint, &cPoint, &normal));
				break;
			case 3:
				EXPECT_NO_THROW(
					contactFound = calculateContactTriangleCapsule(t.v0, t.v1, t.v2, t.n, c.v1, c.v0, c.r,
					&penetrationDepth, &tPoint, &cPoint, &normal));
				break;
			case 4:
				EXPECT_NO_THROW(
					contactFound = calculateContactTriangleCapsule(t.v1, t.v2, t.v0, t.n, c.v1, c.v0, c.r,
					&penetrationDepth, &tPoint, &cPoint, &normal));
				break;
			case 5:
				EXPECT_NO_THROW(
					contactFound = calculateContactTriangleCapsule(t.v2, t.v0, t.v1, t.n, c.v1, c.v0, c.r,
					&penetrationDepth, &tPoint, &cPoint, &normal));
				break;
			case 6:
				EXPECT_NO_THROW(
					contactFound = calculateContactTriangleCapsule(t.v2, t.v1, t.v0, c.v0, c.v1, c.r,
					&penetrationDepth, &tPoint, &cPoint, &normal));
				break;
			case 7:
				EXPECT_NO_THROW(
					contactFound = calculateContactTriangleCapsule(t.v1, t.v0, t.v2, c.v0, c.v1, c.r,
					&penetrationDepth, &tPoint, &cPoint, &normal));
				break;
			case 8:
				EXPECT_NO_THROW(
					contactFound = calculateContactTriangleCapsule(t.v0, t.v2, t.v1, c.v0, c.v1, c.r,
					&penetrationDepth, &tPoint, &cPoint, &normal));
				break;
			case 9:
				EXPECT_NO_THROW(
					contactFound = calculateContactTriangleCapsule(t.v2, t.v1, t.v0, c.v1, c.v0, c.r,
					&penetrationDepth, &tPoint, &cPoint, &normal));
				break;
			case 10:
				EXPECT_NO_THROW(
					contactFound = calculateContactTriangleCapsule(t.v1, t.v0, t.v2, c.v1, c.v0, c.r,
					&penetrationDepth, &tPoint, &cPoint, &normal));
				break;
			case 11:
				EXPECT_NO_THROW(
					contactFound = calculateContactTriangleCapsule(t.v0, t.v2, t.v1, c.v1, c.v0, c.r,
					&penetrationDepth, &tPoint, &cPoint, &normal));
				break;
			}

			EXPECT_EQ(contactExpected, contactFound);
			if (contactFound)
			{
				if (checkForPenetrationPoints && count < 6)
				{
					EXPECT_NEAR(expectedPenetrationDepth, penetrationDepth, Geometry::DistanceEpsilon);
					checkEqual(expectedTPoint, tPoint);
					checkEqual(expectedCPoint, cPoint);
				}

				// Check that tPoint is on the plane of t.
				double tSignedDistance = std::abs(tPoint.dot(t.n) - t.v0.dot(t.n));
				EXPECT_NEAR(tSignedDistance, 0.0, Geometry::DistanceEpsilon);

				// Check that cPoint is on the surface of c.
				Vector3d result;
				double cDistance = SurgSim::Math::distancePointSegment(cPoint, c.v0, c.v1, &result) - c.r;
				EXPECT_LE(std::abs(cDistance), Geometry::DistanceEpsilon);

				// Check that tPoint is inside t.
				EXPECT_TRUE(SurgSim::Math::isPointInsideTriangle(tPoint, t.v0, t.v1, t.v2));

				// Check if the penetration depth when applied as correction, separates the shapes.
				// First move the shapes apart by just short of the penetration depth, to make sure
				// the shapes are still colliding.
				{
					Vector3d tP, cP;
					Vector3d correction = normal * (0.5 * penetrationDepth - Geometry::DistanceEpsilon);
					MockTriangle correctedT(t);
					correctedT.translate(correction);
					MockCapsule correctedC(c);
					correctedC.translate(-correction);
					auto correctedDistance = distanceSegmentTriangle(correctedC.v0, correctedC.v1, correctedT.v0,
						correctedT.v1, correctedT.v2, correctedT.n, &cP, &tP) - c.r;
					EXPECT_TRUE(correctedDistance >= -4.0 * Geometry::DistanceEpsilon) << "correctedDistance = " << correctedDistance;
					EXPECT_TRUE(correctedDistance <= Geometry::DistanceEpsilon) << "correctedDistance = " << correctedDistance;
				}
				// Now move the shapes apart by just a little farther than the penetration depth, to establish
				// that the shapes are not colliding.
				{
					Vector3d tP, cP;
					Vector3d correction = normal * (0.5 * penetrationDepth + Geometry::DistanceEpsilon);
					MockTriangle correctedT(t);
					correctedT.translate(correction);
					MockCapsule correctedC(c);
					correctedC.translate(-correction);
					auto correctedDistance = distanceSegmentTriangle(correctedC.v0, correctedC.v1, correctedT.v0,
						correctedT.v1, correctedT.v2, correctedT.n, &cP, &tP) - c.r;
					EXPECT_TRUE(correctedDistance <= 4.0 * Geometry::DistanceEpsilon) << "correctedDistance = " << correctedDistance;
					EXPECT_TRUE(correctedDistance >= -Geometry::DistanceEpsilon) << "correctedDistance = " << correctedDistance;
				}
			}
		}
	}

	void testTriangleCapsuleContactCalculation(std::string scenario, Vector3d cv0, Vector3d cv1, bool contactFound,
		bool checkPenetrationPoints = false, Vector3d pointOnTriangle = Vector3d::Zero(),
		Vector3d pointOnCapsule = Vector3d::Zero())
	{
		MockTriangle t(Vector3d(5, -5, 0), Vector3d(0, 5, 0), Vector3d(-5, -5, 0));
		MockCapsule c(MockCapsule(cv0, cv1, 0.5));

		for (const auto& transform : m_transforms)
		{
			MockTriangle transformedT(t);
			transformedT.transform(transform);
			MockCapsule transformedC(c);
			transformedC.transform(transform);
			testTriangleCapsuleContactCalculation(TriangleCapsuleTestCase(scenario, transformedT, transformedC,
				contactFound, checkPenetrationPoints, (transform * pointOnTriangle).eval(),
				(transform * pointOnCapsule).eval()));
		}
	}

private:
	// List of random transformations.
	std::vector<SurgSim::Math::RigidTransform3d> m_transforms;
};

TEST_F(TriangleCapsuleContactCalculationTest, TestCase1)
{
	testTriangleCapsuleContactCalculation("(Perpendicular) Capsule far away from triangle",
		Vector3d(0.0, 0.0, 10.0), Vector3d(0.0, 0.0, 5.0), false);
}

TEST_F(TriangleCapsuleContactCalculationTest, TestCase2)
{
	testTriangleCapsuleContactCalculation("(Perpendicular) Capsule far away from triangle",
		Vector3d(0.0, 0.0, 10.0), Vector3d(0.0, 0.0, 5.0), false);
}

TEST_F(TriangleCapsuleContactCalculationTest, TestCase3)
{
	testTriangleCapsuleContactCalculation("(Perpendicular) Capsule just away triangle",
		Vector3d(0.0, 0.0, 10.0), Vector3d(0.0, 0.0, 0.500001), false);
}

TEST_F(TriangleCapsuleContactCalculationTest, TestCase4)
{
	testTriangleCapsuleContactCalculation("(Perpendicular) Capsule just touching triangle",
		Vector3d(0.0, 0.0, 10.0), Vector3d(0.0, 0.0, 0.49999), true, true, Vector3d::Zero(), Vector3d(0, 0, -0.00001));
}

TEST_F(TriangleCapsuleContactCalculationTest, TestCase5)
{
	testTriangleCapsuleContactCalculation("(Perpendicular) Capsule axis just away from triangle",
		Vector3d(0.0, 0.0, 10.0), Vector3d(0.0, 0.0, 0.0001), true, true, Vector3d::Zero(), Vector3d(0, 0, -0.4999));
}

TEST_F(TriangleCapsuleContactCalculationTest, TestCase6)
{
	testTriangleCapsuleContactCalculation("(Perpendicular) Capsule axis just touching triangle",
		Vector3d(0.0, 0.0, 10.0), Vector3d(0.0, 0.0, -0.0001), true, true, Vector3d::Zero(), Vector3d(0, 0, -0.5001));
}

TEST_F(TriangleCapsuleContactCalculationTest, TestCase7)
{
	testTriangleCapsuleContactCalculation("(Angled) Capsule far away from triangle",
		Vector3d(1.0, 1.0, 10.0), Vector3d(0.0, 0.0, 5.0), false);
}

TEST_F(TriangleCapsuleContactCalculationTest, TestCase8)
{
	testTriangleCapsuleContactCalculation("(Angled) Capsule just away triangle",
		Vector3d(1.0, 1.0, 10.0), Vector3d(0.0, 0.0, 0.500001), false);
}

TEST_F(TriangleCapsuleContactCalculationTest, TestCase9)
{
	testTriangleCapsuleContactCalculation("(Angled) Capsule just touching triangle",
		Vector3d(1.0, 1.0, 10.0), Vector3d(0.0, 0.0, 0.49999), true, true, Vector3d::Zero(), Vector3d(0, 0, -0.00001));
}

TEST_F(TriangleCapsuleContactCalculationTest, TestCase10)
{
	testTriangleCapsuleContactCalculation("(Angled) Capsule axis just away from triangle",
		Vector3d(1.0, 1.0, 10.0), Vector3d(0.0, 0.0, 0.0001), true, true, Vector3d::Zero(), Vector3d(0, 0, -0.4999));
}

TEST_F(TriangleCapsuleContactCalculationTest, TestCase11)
{
	testTriangleCapsuleContactCalculation("(Angled) Capsule axis just touching triangle",
		Vector3d(1.0, 1.0, 10.0), Vector3d(0.0, 0.0, -0.0001), true, true, Vector3d::Zero(), Vector3d(0, 0, -0.5001));
}

TEST_F(TriangleCapsuleContactCalculationTest, TestCase12)
{
	testTriangleCapsuleContactCalculation("Capsule axis through triangle",
		Vector3d(0.0, 6.0, 1.0), Vector3d(0.0, -6.0, -1.0), true, false);
}

TEST_F(TriangleCapsuleContactCalculationTest, TestCase13)
{
	testTriangleCapsuleContactCalculation("(Angled, near edge 1) Capsule far away from triangle",
		Vector3d(4.0, 0.0, 5.0), Vector3d(2.4, 0.0, 2.0), false);
}

TEST_F(TriangleCapsuleContactCalculationTest, TestCase14)
{
	testTriangleCapsuleContactCalculation("(Angled, near edge 1) Capsule just away triangle",
		Vector3d(4.0, 0.0, 5.0), Vector3d(2.4, 0.0, 0.500001), false);
}

TEST_F(TriangleCapsuleContactCalculationTest, TestCase15)
{
	testTriangleCapsuleContactCalculation("(Angled, near edge 1) Capsule just touching triangle",
		Vector3d(4.0, 0.0, 5.0), Vector3d(2.4, 0.0, 0.49999), true,
		true, Vector3d(2.4, 0.0, 0.0), Vector3d(2.4, 0.0,-0.00001));
}

TEST_F(TriangleCapsuleContactCalculationTest, TestCase16)
{
	testTriangleCapsuleContactCalculation("(Angled, near edge 1) Capsule axis just away from triangle",
		Vector3d(4.0, 0.0, 5.0), Vector3d(2.4, 0.0, 0.0001), true,
		true, Vector3d(2.4, 0.0, 0.0), Vector3d(2.4, 0.0, -0.4999));
}

TEST_F(TriangleCapsuleContactCalculationTest, TestCase17)
{
	testTriangleCapsuleContactCalculation("(Angled, near edge 1) Capsule axis just touching triangle",
		Vector3d(4.0, 0.0, 5.0), Vector3d(2.4, 0.0, -0.0001), true,
		true, Vector3d(2.4, 0.0, 0.0), Vector3d(2.4, 0.0, -0.5001));
}

TEST_F(TriangleCapsuleContactCalculationTest, TestCase18)
{
	testTriangleCapsuleContactCalculation("(Angled, near edge 1) Capsule axis inside triangle",
		Vector3d(4.0, 0.0, 5.0), Vector3d(2.4, 0.0, -0.5), true);
}

TEST_F(TriangleCapsuleContactCalculationTest, TestCase19)
{
	testTriangleCapsuleContactCalculation("(Angled, near edge 2) Capsule far away from triangle",
		Vector3d(-4.0, 0.0, 5.0), Vector3d(-2.4, 0.0, 2.0), false);
}

TEST_F(TriangleCapsuleContactCalculationTest, TestCase20)
{
	testTriangleCapsuleContactCalculation("(Angled, near edge 2) Capsule just away triangle",
		Vector3d(-4.0, 0.0, 5.0), Vector3d(-2.4, 0.0, 0.500001), false);
}

TEST_F(TriangleCapsuleContactCalculationTest, TestCase21)
{
	testTriangleCapsuleContactCalculation("(Angled, near edge 2) Capsule just touching triangle",
		Vector3d(-4.0, 0.0, 5.0), Vector3d(-2.4, 0.0, 0.49999), true,
		true, Vector3d(-2.4, 0.0, 0.0), Vector3d(-2.4, 0.0, -0.00001));
}

TEST_F(TriangleCapsuleContactCalculationTest, TestCase22)
{
	testTriangleCapsuleContactCalculation("(Angled, near edge 2) Capsule axis just away from triangle",
		Vector3d(-4.0, 0.0, 5.0), Vector3d(-2.4, 0.0, 0.0001), true,
		true, Vector3d(-2.4, 0.0, 0.0), Vector3d(-2.4, 0.0, -0.4999));
}

TEST_F(TriangleCapsuleContactCalculationTest, TestCase23)
{
	testTriangleCapsuleContactCalculation("(Angled, near edge 2) Capsule axis just touching triangle",
		Vector3d(-4.0, 0.0, 5.0), Vector3d(-2.4, 0.0, -0.0001), true,
		true, Vector3d(-2.4, 0.0, 0.0), Vector3d(-2.4, 0.0, -0.5001));
}

TEST_F(TriangleCapsuleContactCalculationTest, TestCase24)
{
	testTriangleCapsuleContactCalculation("(Angled, near edge 2) Capsule axis inside triangle",
		Vector3d(-4.0, 0.0, 5.0), Vector3d(-2.4, 0.0, -0.5), true);
}

TEST_F(TriangleCapsuleContactCalculationTest, TestCase25)
{
	testTriangleCapsuleContactCalculation("(Angled, near edge 3) Capsule far away from triangle",
		Vector3d(0.0, -6.0, 5.0), Vector3d(0.0, -4.8, 2.0), false);
}

TEST_F(TriangleCapsuleContactCalculationTest, TestCase26)
{
	testTriangleCapsuleContactCalculation("(Angled, near edge 3) Capsule just away triangle",
		Vector3d(0.0, -6.0, 5.0), Vector3d(0.0, -4.8, 0.500001), false);
}

TEST_F(TriangleCapsuleContactCalculationTest, TestCase27)
{
	testTriangleCapsuleContactCalculation("(Angled, near edge 3) Capsule just touching triangle",
		Vector3d(0.0, -6.0, 5.0), Vector3d(0.0, -4.8, 0.49999), true,
		true, Vector3d(0,-4.8,0), Vector3d(0,-4.8,-0.00001));
}

TEST_F(TriangleCapsuleContactCalculationTest, TestCase28)
{
	testTriangleCapsuleContactCalculation("(Angled, near edge 3) Capsule axis just away from triangle",
		Vector3d(0.0, -6.0, 5.0), Vector3d(0.0, -4.8, 0.0001), true,
		true, Vector3d(0.0, -4.8, 0.0), Vector3d(0.0, -4.8, -0.4999));
}

TEST_F(TriangleCapsuleContactCalculationTest, TestCase29)
{
	testTriangleCapsuleContactCalculation("(Angled, near edge 3) Capsule axis just touching triangle",
		Vector3d(0.0, -6.0, 5.0), Vector3d(0.0, -4.8, -0.0001), true,
		true, Vector3d(0.0 ,-4.8, 0.0), Vector3d(0.0, -4.8, -0.5001));
}

TEST_F(TriangleCapsuleContactCalculationTest, TestCase30)
{
	testTriangleCapsuleContactCalculation("(Angled, near edge 3) Capsule axis inside triangle",
		Vector3d(0.0, -6.0, 5.0), Vector3d(0.0, -4.8, -0.5), true);
}

}
}