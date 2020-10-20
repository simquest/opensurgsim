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

#include "SurgSim/Framework/ApplicationData.h"
#include "SurgSim/Math/Geometry.h"
#include "SurgSim/Math/UnitTests/TriangleTriangleTestParameters.h"

namespace SurgSim
{
namespace Math
{

class TriangleTriangleContactCalculationTest : public ::testing::Test, public TriangleTriangleTestParameters
{
protected:
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

	void testTriangleTriangleContactCalculation(const TriangleTriangleTestCase& data)
	{
		SCOPED_TRACE(std::get<0>(data));

		MockTriangle t0 = std::get<1>(data);
		MockTriangle t1 = std::get<2>(data);
		bool contactExpected = std::get<3>(data);
		bool checkForPenetrationPoints = std::get<4>(data);
		Vector3d expectedT0Point = std::get<5>(data);
		Vector3d expectedT1Point = std::get<6>(data);

		double expectedPenetrationDepth = (expectedT1Point - expectedT0Point).norm();
		double penetrationDepth;
		Vector3d t0Point, t1Point, normal;
		bool contactFound = false;
		std::string traceMessage[6] = {"Normal Test",
									   "Shift t0 edges once",
									   "Shift t0 edges twice",
									   "Switched triangles: Normal Test",
									   "Switched triangles: Shift t1 edges once",
									   "Switched triangles: Shift t1 edges twise"
									  };
		for (int count = 0; count < 6; ++count)
		{
			SCOPED_TRACE(traceMessage[count]);

			switch (count)
			{
			case 0:
				EXPECT_NO_THROW(
					contactFound = calculateContactTriangleTriangle(t0.v0, t0.v1, t0.v2, t1.v0, t1.v1, t1.v2,
																	&penetrationDepth, &t0Point, &t1Point, &normal););
				break;
			case 1:
				EXPECT_NO_THROW(
					contactFound = calculateContactTriangleTriangle(t0.v1, t0.v2, t0.v0, t1.v0, t1.v1, t1.v2,
																	&penetrationDepth, &t0Point, &t1Point, &normal););
				break;
			case 2:
				EXPECT_NO_THROW(
					contactFound = calculateContactTriangleTriangle(t0.v2, t0.v0, t0.v1, t1.v0, t1.v1, t1.v2,
																	&penetrationDepth, &t0Point, &t1Point, &normal););
				break;
			case 3:
				EXPECT_NO_THROW(
					contactFound = calculateContactTriangleTriangle(t1.v0, t1.v1, t1.v2, t0.v0, t0.v1, t0.v2,
																	&penetrationDepth, &t1Point, &t0Point, &normal););
				break;
			case 4:
				EXPECT_NO_THROW(
					contactFound = calculateContactTriangleTriangle(t1.v1, t1.v2, t1.v0, t0.v0, t0.v1, t0.v2,
																	&penetrationDepth, &t1Point, &t0Point, &normal););
				break;
			case 5:
				EXPECT_NO_THROW(
					contactFound = calculateContactTriangleTriangle(t1.v2, t1.v0, t1.v1, t0.v0, t0.v1, t0.v2,
																	&penetrationDepth, &t1Point, &t0Point, &normal););
				break;
			}

			EXPECT_EQ(contactExpected, contactFound);
			if (contactFound)
			{
				if (checkForPenetrationPoints && count < 3)
				{
					EXPECT_NEAR(expectedPenetrationDepth, penetrationDepth, Geometry::DistanceEpsilon);
					checkEqual(expectedT0Point, t0Point);
					checkEqual(expectedT1Point, t1Point);
				}

				// Check that t0Point is on the plane of t0.
				double t0SignedDistance = std::abs(t0Point.dot(t0.n) - t0.v0.dot(t0.n));
				EXPECT_NEAR(t0SignedDistance, 0.0, Geometry::DistanceEpsilon);

				// Check that t1Point is on the plane of t1.
				double t1SignedDistance = std::abs(t1Point.dot(t1.n) - t1.v0.dot(t1.n));
				EXPECT_NEAR(t1SignedDistance, 0.0, Geometry::DistanceEpsilon);

				// Check that t0Point is inside t0.
				Vector3d bary0;
				barycentricCoordinates(t0Point, t0.v0, t0.v1, t0.v2, &bary0);
				bool isBary0WithinTriangle =
					bary0[0] >= -Geometry::DistanceEpsilon && bary0[0] <= (1.0 + Geometry::DistanceEpsilon) &&
					bary0[1] >= -Geometry::DistanceEpsilon && bary0[1] <= (1.0 + Geometry::DistanceEpsilon) &&
					bary0[2] >= -Geometry::DistanceEpsilon && bary0[2] <= (1.0 + Geometry::DistanceEpsilon);
				EXPECT_TRUE(isBary0WithinTriangle);

				// Check that t1Point is inside t1.
				Vector3d bary1;
				barycentricCoordinates(t1Point, t1.v0, t1.v1, t1.v2, &bary1);
				bool isBary1WithinTriangle =
					bary1[0] >= -Geometry::DistanceEpsilon && bary1[0] <= (1.0 + Geometry::DistanceEpsilon) &&
					bary1[1] >= -Geometry::DistanceEpsilon && bary1[1] <= (1.0 + Geometry::DistanceEpsilon) &&
					bary1[2] >= -Geometry::DistanceEpsilon && bary1[2] <= (1.0 + Geometry::DistanceEpsilon);
				EXPECT_TRUE(isBary1WithinTriangle);

				// Check if the penetration depth when applied as correction, separates the triangles.
				// First move the triangles apart by just short of the penetration depth, to make sure
				// the triangles are still colliding.
				{
					Vector3d correction = normal * (0.5 * penetrationDepth - Geometry::DistanceEpsilon);
					if (count > 2)
					{
						// Switched triangles.
						correction = -correction;
					}
					MockTriangle correctedT0(t0);
					correctedT0.translate(correction);
					MockTriangle correctedT1(t1);
					correctedT1.translate(-correction);
					EXPECT_TRUE(doesIntersectTriangleTriangle(correctedT0.v0, correctedT0.v1, correctedT0.v2,
															  correctedT1.v0, correctedT1.v1, correctedT1.v2));
				}
				// Now move the triangles apart by just a little farther than the penetration depth, to establish
				// that the triangles are not colliding.
				{
					Vector3d correction = normal * (0.5 * penetrationDepth + Geometry::DistanceEpsilon);
					if (count > 2)
					{
						// Switched triangles.
						correction = -correction;
					}
					MockTriangle correctedT0(t0);
					correctedT0.translate(correction);
					MockTriangle correctedT1(t1);
					correctedT1.translate(-correction);
					EXPECT_FALSE(doesIntersectTriangleTriangle(correctedT0.v0, correctedT0.v1, correctedT0.v2,
															   correctedT1.v0, correctedT1.v1, correctedT1.v2));
				}
			}
		}
	}
};

TEST_F(TriangleTriangleContactCalculationTest, TestCases)
{
	for (auto it = m_testCases.begin(); it != m_testCases.end(); ++it)
	{
		testTriangleTriangleContactCalculation(*it);
	}
}
namespace
{
void fromFile(const std::string& filename, MockTriangle* t0, MockTriangle* t1)
{
	// 8 Vector 3 doubles = 24 doubles
	size_t size = sizeof(double) * 24;

	std::ifstream in(filename, std::ios_base::binary);
	SURGSIM_ASSERT(in.is_open());

	std::vector<uint8_t> bytes(std::istreambuf_iterator<char>(in), {});

	size_t start = 0;

	start += SurgSim::Math::fromBytes(bytes, &t0->v0, start);
	start += SurgSim::Math::fromBytes(bytes, &t0->v1, start);
	start += SurgSim::Math::fromBytes(bytes, &t0->v2, start);
	start += SurgSim::Math::fromBytes(bytes, &t0->n, start);
	start += SurgSim::Math::fromBytes(bytes, &t1->v0, start);
	start += SurgSim::Math::fromBytes(bytes, &t1->v1, start);
	start += SurgSim::Math::fromBytes(bytes, &t1->v2, start);
	start += SurgSim::Math::fromBytes(bytes, &t1->n, start);

	SURGSIM_ASSERT(start == size);
}
}

TEST_F(TriangleTriangleContactCalculationTest, TestFromFile)
{
	SurgSim::Framework::ApplicationData data("config.txt");
	std::string fileName = "TriangleTriangleContactException.bin";
	std::string path = data.findFile(fileName);
	ASSERT_TRUE(!path.empty()) << "Can not locate file " << fileName;
	MockTriangle t0;
	MockTriangle t1;
	fromFile(path, &t0, &t1);

	std::string scenario = "TestFromFile";
	Vector3d t0p;
	Vector3d t1p;
	testTriangleTriangleContactCalculation(TriangleTriangleTestCase(scenario, t0, t1, false, false, t0p, t1p));
}

}
}