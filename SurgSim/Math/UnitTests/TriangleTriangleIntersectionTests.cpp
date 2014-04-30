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
#include "SurgSim/Math/UnitTests/TriangleTriangleTestParameters.h"

namespace SurgSim
{
namespace Math
{

class TriangleTriangleIntersectionTest : public ::testing::Test, public TriangleTriangleTestParameters
{
protected:
	void testTriangleTriangleIntersection(const TriangleTriangleTestCase& data)
	{
		SCOPED_TRACE(std::get<0>(data));

		MockTriangle t0 = std::get<1>(data);
		MockTriangle t1 = std::get<2>(data);
		bool intersectionExpected = std::get<3>(data);

		bool intersectionFound = false;
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
				intersectionFound = checkTriangleTriangleIntersection(t0.v0, t0.v1, t0.v2, t1.v0, t1.v1, t1.v2);
				break;
			case 1:
				intersectionFound = checkTriangleTriangleIntersection(t0.v1, t0.v2, t0.v0, t1.v0, t1.v1, t1.v2);
				break;
			case 2:
				intersectionFound = checkTriangleTriangleIntersection(t0.v2, t0.v0, t0.v1, t1.v0, t1.v1, t1.v2);
				break;
			case 3:
				intersectionFound = checkTriangleTriangleIntersection(t1.v0, t1.v1, t1.v2, t0.v0, t0.v1, t0.v2);
				break;
			case 4:
				intersectionFound = checkTriangleTriangleIntersection(t1.v1, t1.v2, t1.v0, t0.v0, t0.v1, t0.v2);
				break;
			case 5:
				intersectionFound = checkTriangleTriangleIntersection(t1.v2, t1.v0, t1.v1, t0.v0, t0.v1, t0.v2);
				break;
			}

			EXPECT_EQ(intersectionExpected, intersectionFound);
		}
	}
};

TEST_F(TriangleTriangleIntersectionTest, TestCases)
{
	for (auto it = m_testCases.begin(); it != m_testCases.end(); ++it)
	{
		testTriangleTriangleIntersection(*it);
	}
}

}
}