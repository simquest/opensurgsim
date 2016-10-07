// This file is a part of the OpenSurgSim project.
// Copyright 2013 - 2016, SimQuest Solutions Inc.
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
#include <gmock/gmock.h>

#include "SurgSim/Blocks/KnotIdentificationBehavior.h"
#include "SurgSim/Framework/BasicSceneElement.h"
#include "SurgSim/Framework/Runtime.h"
#include "SurgSim/Framework/Scene.h"

#define MAKE_VEC(vec) std::vector<int>(vec, vec + sizeof(vec) / sizeof(int))

namespace SurgSim
{
namespace Blocks
{
class MockKnotIdentificationBehavior : public KnotIdentificationBehavior
{
public:
	explicit MockKnotIdentificationBehavior(const std::string& name) : KnotIdentificationBehavior(name) {}

	bool tryReidmeisterMove1(std::vector<int>* gaussCode, std::vector<int>* erased)
	{
		return KnotIdentificationBehavior::tryReidmeisterMove1(gaussCode, erased);
	}

	bool tryReidmeisterMove2(std::vector<int>* gaussCode, std::vector<int>* erased)
	{
		return KnotIdentificationBehavior::tryReidmeisterMove2(gaussCode, erased);
	}

	bool tryReidmeisterMove3(std::vector<int>* gaussCode)
	{
		return KnotIdentificationBehavior::tryReidmeisterMove3(gaussCode);
	}

	void adjustGaussCodeForErasedCrossings(std::vector<int>* gaussCode, std::vector<int>* erased)
	{
		return KnotIdentificationBehavior::adjustGaussCodeForErasedCrossings(gaussCode, erased);
	}

	std::string identifyKnot(const std::vector<int>& gaussCode)
	{
		KnotIdentificationBehavior::identifyKnot(gaussCode);
		return KnotIdentificationBehavior::getKnotName();
	}
};

class KnotIdentificationBehaviorTest : public testing::Test
{
public:
	KnotIdentificationBehaviorTest() : testing::Test(), mockKnotIdentificationBehavior3("Test3")
	{

	}

	void SetUp()
	{

	}

	void testReidmeisterMove1(std::vector<int> gaussCodeBefore, std::vector<int> gaussCodeAfter,
							  std::vector<int> erased = std::vector<int>())
	{
		MockKnotIdentificationBehavior mockKnotIdentificationBehavior("Test ReidmeisterMove1");
		std::vector<int> erasedCalculated;
		bool expectedReturn = gaussCodeBefore != gaussCodeAfter;
		bool actualReturn = mockKnotIdentificationBehavior.tryReidmeisterMove1(&gaussCodeBefore, &erasedCalculated);
		EXPECT_EQ(expectedReturn, actualReturn);
		EXPECT_TRUE(gaussCodeBefore == gaussCodeAfter);
		std::sort(erased.begin(), erased.end(), std::greater<int>());
		std::sort(erasedCalculated.begin(), erasedCalculated.end(), std::greater<int>());
		EXPECT_TRUE(erased == erasedCalculated);
	}

	void testReidmeisterMove2(std::vector<int> gaussCodeBefore, std::vector<int> gaussCodeAfter,
							  std::vector<int> erased = std::vector<int>())
	{
		MockKnotIdentificationBehavior mockKnotIdentificationBehavior("Test ReidmeisterMove2");
		std::vector<int> erasedCalculated;
		bool expectedReturn = gaussCodeBefore != gaussCodeAfter;
		bool actualReturn = mockKnotIdentificationBehavior.tryReidmeisterMove2(&gaussCodeBefore, &erasedCalculated);
		EXPECT_EQ(expectedReturn, actualReturn);
		EXPECT_TRUE(gaussCodeBefore == gaussCodeAfter);
		std::sort(erased.begin(), erased.end(), std::greater<int>());
		std::sort(erasedCalculated.begin(), erasedCalculated.end(), std::greater<int>());
		EXPECT_TRUE(erased == erasedCalculated);
	}

	void testReidmeisterMove3(std::vector<int> gaussCodeBefore, std::vector<int> gaussCodeAfter, bool expectedReturn)
	{
		bool actualReturn = mockKnotIdentificationBehavior3.tryReidmeisterMove3(&gaussCodeBefore);
		EXPECT_EQ(expectedReturn, actualReturn);
		EXPECT_TRUE(gaussCodeBefore == gaussCodeAfter);
	}

	void testAdjustGaussCodeForErasedCrossings(std::vector<int> gaussCodeBefore, std::vector<int> gaussCodeAfter,
		std::vector<int> erased)
	{
		MockKnotIdentificationBehavior mockKnotIdentificationBehavior("Test AdjustGaussCodeForErasedCrossings");
		mockKnotIdentificationBehavior.adjustGaussCodeForErasedCrossings(&gaussCodeBefore, &erased);
		EXPECT_TRUE(gaussCodeBefore == gaussCodeAfter);
	}

	void testIdentifyKnot(std::vector<int> gaussCode, std::string expected)
	{
		MockKnotIdentificationBehavior mockKnotIdentificationBehavior("Test IdentifyKnot");
		auto actual = mockKnotIdentificationBehavior.identifyKnot(gaussCode);
		EXPECT_TRUE(expected == actual);
	}

	MockKnotIdentificationBehavior mockKnotIdentificationBehavior3;
};

TEST_F(KnotIdentificationBehaviorTest, ReidmeisterMove1)
{
	{
		int input[] = {1, -2, 3, -1, 2, -3};
		int expected[] = {1, -2, 3, -1, 2, -3};
		testReidmeisterMove1(MAKE_VEC(input), MAKE_VEC(expected));
	}
	{
		int input[] = {1, -2, 3, -3, -1, 2};
		int expected[] = {1, -2, -1, 2};
		int erased[] = {3};
		testReidmeisterMove1(MAKE_VEC(input), MAKE_VEC(expected), MAKE_VEC(erased));
	}
	{
		int input[] = {3, 1, -2, -1, 2, -3};
		int expected[] = {1, -2, -1, 2};
		int erased[] = {3};
		testReidmeisterMove1(MAKE_VEC(input), MAKE_VEC(expected), MAKE_VEC(erased));
	}
	{
		int input[] = {3, 1, 4, -4, -2, 6, -6, -1, -5, 5, 2, -3};
		int expected[] = {1, -2, -1, 2};
		int erased[] = {3, 4, 5, 6};
		testReidmeisterMove1(MAKE_VEC(input), MAKE_VEC(expected), MAKE_VEC(erased));
	}
	{
		int input[] = {3, 1, 4, -4, -2, 6, 7, -6, -7, -1, -5, 5, 2, -3};
		int expected[] = {1, -2, 6, 7, -6, -7, -1, 2};
		int erased[] = {3, 4, 5};
		testReidmeisterMove1(MAKE_VEC(input), MAKE_VEC(expected), MAKE_VEC(erased));
	}
}

TEST_F(KnotIdentificationBehaviorTest, ReidmeisterMove2)
{
	{
		int input[] = {3, 1, -2, -1, 2, -3, 4, 5, -4, -5};
		int expected[] = {3, 1, -2, -1, 2, -3};
		int erased[] = {4, 5};
		testReidmeisterMove2(MAKE_VEC(input), MAKE_VEC(expected), MAKE_VEC(erased));
	}
	{
		int input[] = {-5, 3, 1, -2, -1, 2, -3, 4, 5, -4};
		int expected[] = {3, 1, -2, -1, 2, -3};
		int erased[] = {4, 5};
		testReidmeisterMove2(MAKE_VEC(input), MAKE_VEC(expected), MAKE_VEC(erased));
	}
	{
		int input[] = {3, 1, -2, 7, 6, -1, 2, -3, 4, 5, -7, -6, -4, -5};
		int expected[] = {3, 1, -2, -1, 2, -3, 4, 5, -4, -5};
		int erased[] = {6, 7};
		testReidmeisterMove2(MAKE_VEC(input), MAKE_VEC(expected), MAKE_VEC(erased));
	}
}

TEST_F(KnotIdentificationBehaviorTest, ReidmeisterMove3)
{
	{
		int input[] = {-1, -3, -2, 1, 2, 3};
		int expected[] = {-1, -3, 3, -2, 1, 2};
		testReidmeisterMove3(MAKE_VEC(input), MAKE_VEC(expected), true);
		int expected2[] = {2, -3, -2, 3, 1, -1};
		testReidmeisterMove3(MAKE_VEC(expected), MAKE_VEC(expected2), true);
		int expected3[] = {3, -1, -3, 1, 2, -2};
		testReidmeisterMove3(MAKE_VEC(expected2), MAKE_VEC(expected3), true);
		int expected4[] = {-2, -1, 1, -3, 2, 3};
		testReidmeisterMove3(MAKE_VEC(expected3), MAKE_VEC(expected4), true);
		testReidmeisterMove3(MAKE_VEC(expected4), MAKE_VEC(input), false);
	}
	{
		int input[] = {3, 1, -4, -2, -1, -5, 2, -3, 4, 5};
		int expected[] = {3, 1, -4, -3, -2, -5, -1, 2, 4, 5};
		testReidmeisterMove3(MAKE_VEC(input), MAKE_VEC(expected), true);
	}
}

TEST_F(KnotIdentificationBehaviorTest, AdjustGaussCodeForErasedCrossings)
{
	{
		int input[] = {3, -4, 5, -3, 4, -5};
		int expected[] = {1, -2, 3, -1, 2, -3};
		int erased[] = {1, 2};
		testAdjustGaussCodeForErasedCrossings(MAKE_VEC(input), MAKE_VEC(expected), MAKE_VEC(erased));
	}
	{
		int input[] = {1, -4, 5, -1, 4, -5};
		int expected[] = {1, -2, 3, -1, 2, -3};
		int erased[] = {2, 3};
		testAdjustGaussCodeForErasedCrossings(MAKE_VEC(input), MAKE_VEC(expected), MAKE_VEC(erased));
	}
}

TEST_F(KnotIdentificationBehaviorTest, IdentityKnot)
{
	{
		testIdentifyKnot(std::vector<int>(), "No Knot");
	}
	{
		int input[] = {1, -4, 5, -1, 4, -5};
		testIdentifyKnot(MAKE_VEC(input), "Unknown Knot");
	}
	{
		int input[] = {1, -2, 3, -1, 2, -3};
		testIdentifyKnot(MAKE_VEC(input), "Trefoil Knot");
	}
	{
		int input[] = {-1, 2, -3, 1, -2, 3};
		testIdentifyKnot(MAKE_VEC(input), "Trefoil Knot");
	}
	{
		int input[] = {-3, 1, -2, 3, -1, 2};
		testIdentifyKnot(MAKE_VEC(input), "Trefoil Knot");
	}
}
}
}
