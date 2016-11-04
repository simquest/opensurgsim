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

#include <boost/assign/list_of.hpp>
#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include "SurgSim/Blocks/KnotIdentificationBehavior.h"
#include "SurgSim/Framework/BasicSceneElement.h"
#include "SurgSim/Framework/Runtime.h"
#include "SurgSim/Framework/Scene.h"
#include "SurgSim/Physics/Fem1DRepresentation.h"

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
		return KnotIdentificationBehavior::tryReidmeisterMove3(gaussCode, &data);
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

	KnotIdentificationBehavior::ReidmeisterMove3Data data;
};

class KnotIdentificationBehaviorTest : public testing::Test
{
public:
	KnotIdentificationBehaviorTest() : testing::Test(), mockKnotIdentificationBehavior("Test3")
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
		EXPECT_EQ(gaussCodeBefore, gaussCodeAfter);
		std::sort(erased.begin(), erased.end(), std::greater<int>());
		std::sort(erasedCalculated.begin(), erasedCalculated.end(), std::greater<int>());
		EXPECT_EQ(erased, erasedCalculated);
	}

	void testReidmeisterMove2(std::vector<int> gaussCodeBefore, std::vector<int> gaussCodeAfter,
							  std::vector<int> erased = std::vector<int>())
	{
		MockKnotIdentificationBehavior mockKnotIdentificationBehavior("Test ReidmeisterMove2");
		std::vector<int> erasedCalculated;
		bool expectedReturn = gaussCodeBefore != gaussCodeAfter;
		bool actualReturn = mockKnotIdentificationBehavior.tryReidmeisterMove2(&gaussCodeBefore, &erasedCalculated);
		EXPECT_EQ(expectedReturn, actualReturn);
		EXPECT_EQ(gaussCodeBefore, gaussCodeAfter);
		std::sort(erased.begin(), erased.end(), std::greater<int>());
		std::sort(erasedCalculated.begin(), erasedCalculated.end(), std::greater<int>());
		EXPECT_EQ(erased, erasedCalculated);
	}

	void testReidmeisterMove3(std::vector<int> gaussCodeBefore, std::vector<int> gaussCodeAfter, bool expectedReturn)
	{
		bool actualReturn = mockKnotIdentificationBehavior.tryReidmeisterMove3(&gaussCodeBefore);
		EXPECT_EQ(expectedReturn, actualReturn);
		EXPECT_EQ(gaussCodeBefore, gaussCodeAfter);
	}

	void testAdjustGaussCodeForErasedCrossings(std::vector<int> gaussCodeBefore, std::vector<int> gaussCodeAfter,
		std::vector<int> erased)
	{
		MockKnotIdentificationBehavior mockKnotIdentificationBehavior("Test AdjustGaussCodeForErasedCrossings");
		mockKnotIdentificationBehavior.adjustGaussCodeForErasedCrossings(&gaussCodeBefore, &erased);
		EXPECT_EQ(gaussCodeBefore, gaussCodeAfter);
	}

	void testIdentifyKnot(std::vector<int> gaussCode, std::string expected)
	{
		MockKnotIdentificationBehavior mockKnotIdentificationBehavior("Test IdentifyKnot");
		auto actual = mockKnotIdentificationBehavior.identifyKnot(gaussCode);
		EXPECT_EQ(expected, actual);
	}

	MockKnotIdentificationBehavior mockKnotIdentificationBehavior;
};

TEST_F(KnotIdentificationBehaviorTest, Constructor)
{
	EXPECT_NO_THROW(KnotIdentificationBehavior knotId("KnotId"));
	EXPECT_NO_THROW(new KnotIdentificationBehavior("KnotId"));
	EXPECT_NO_THROW(std::make_shared<KnotIdentificationBehavior>("KnotId"));
}

TEST_F(KnotIdentificationBehaviorTest, GetSetFem1D)
{
	KnotIdentificationBehavior knotId("KnotId");
	EXPECT_EQ(nullptr, knotId.getFem1d());
	auto physics = std::make_shared<SurgSim::Physics::Representation>("Physics Representation");
	EXPECT_THROW(knotId.setFem1d(physics), SurgSim::Framework::AssertionFailure);
	auto fem1D = std::make_shared<SurgSim::Physics::Fem1DRepresentation>("Fem1D");
	EXPECT_NO_THROW(knotId.setFem1d(fem1D));
	EXPECT_EQ(fem1D->getName(), knotId.getFem1d()->getName());
}

TEST_F(KnotIdentificationBehaviorTest, ReidmeisterMove1)
{
	{
		std::vector<int> input = boost::assign::list_of(1)(-2)(3)(-1)(2)(-3);
		std::vector<int> expected = boost::assign::list_of(1)(-2)(3)(-1)(2)(-3);
		testReidmeisterMove1(input, expected);
	}
	{
		std::vector<int> input = boost::assign::list_of(1)(-2)(3)(-3)(-1)(2);
		std::vector<int> expected = boost::assign::list_of(1)(-2)(-1)(2);
		std::vector<int> erased = boost::assign::list_of(3);
		testReidmeisterMove1(input, expected, erased);
	}
	{
		std::vector<int> input = boost::assign::list_of(3)(1)(-2)(-1)(2)(-3);
		std::vector<int> expected = boost::assign::list_of(1)(-2)(-1)(2);
		std::vector<int> erased = boost::assign::list_of(3);
		testReidmeisterMove1(input, expected, erased);
	}
	{
		std::vector<int> input = boost::assign::list_of(3)(1)(4)(-4)(-2)(6)(-6)(-1)(-5)(5)(2)(-3);
		std::vector<int> expected = boost::assign::list_of(1)(-2)(-1)(2);
		std::vector<int> erased = boost::assign::list_of(3)(4)(5)(6);
		testReidmeisterMove1(input, expected, erased);
	}
	{
		std::vector<int> input = boost::assign::list_of(3)(1)(4)(-4)(-2)(6)(7)(-6)(-7)(-1)(-5)(5)(2)(-3);
		std::vector<int> expected = boost::assign::list_of(1)(-2)(6)(7)(-6)(-7)(-1)(2);
		std::vector<int> erased = boost::assign::list_of(3)(4)(5);
		testReidmeisterMove1(input, expected, erased);
	}
}

TEST_F(KnotIdentificationBehaviorTest, ReidmeisterMove2)
{
	{
		std::vector<int> input = boost::assign::list_of(3)(1)(-2)(-1)(2)(-3)(4)(5)(-4)(-5);
		std::vector<int> expected = boost::assign::list_of(3)(1)(-2)(-1)(2)(-3);
		std::vector<int> erased = boost::assign::list_of(4)(5);
		testReidmeisterMove2(input, expected, erased);
	}
	{
		std::vector<int> input = boost::assign::list_of(-5)(3)(1)(-2)(-1)(2)(-3)(4)(5)(-4);
		std::vector<int> expected = boost::assign::list_of(3)(1)(-2)(-1)(2)(-3);
		std::vector<int> erased = boost::assign::list_of(4)(5);
		testReidmeisterMove2(input, expected, erased);
	}
	{
		std::vector<int> input = boost::assign::list_of(3)(1)(-2)(7)(6)(-1)(2)(-3)(4)(5)(-7)(-6)(-4)(-5);
		std::vector<int> expected = boost::assign::list_of(3)(1)(-2)(-1)(2)(-3)(4)(5)(-4)(-5);
		std::vector<int> erased = boost::assign::list_of(6)(7);
		testReidmeisterMove2(input, expected, erased);
	}
}

TEST_F(KnotIdentificationBehaviorTest, ReidmeisterMove3Test1)
{
	std::vector<int> input = boost::assign::list_of(-1)(-3)(-2)(1)(2)(3);
	std::vector<int> expected = boost::assign::list_of(-1)(-3)(3)(-2)(1)(2);
	testReidmeisterMove3(input, expected, true);
	std::vector<int> expected2 = boost::assign::list_of(2)(-3)(-2)(3)(1)(-1);
	testReidmeisterMove3(expected, expected2, true);
	std::vector<int> expected3 = boost::assign::list_of(3)(-1)(-3)(1)(2)(-2);
	testReidmeisterMove3(expected2, expected3, true);
	std::vector<int> expected4 = boost::assign::list_of(-2)(-1)(1)(-3)(2)(3);
	testReidmeisterMove3(expected3, expected4, true);
	testReidmeisterMove3(expected4, input, false);
}

TEST_F(KnotIdentificationBehaviorTest, ReidmeisterMove3Test2)
{
	std::vector<int> input = boost::assign::list_of(3)(1)(-4)(-2)(-1)(-5)(2)(-3)(4)(5);
	std::vector<int> expected = boost::assign::list_of(3)(1)(-4)(-3)(-2)(-5)(-1)(2)(4)(5);
	testReidmeisterMove3(input, expected, true);
}

TEST_F(KnotIdentificationBehaviorTest, AdjustGaussCodeForErasedCrossings)
{
	{
		std::vector<int> input = boost::assign::list_of(3)(-4)(5)(-3)(4)(-5);
		std::vector<int> expected = boost::assign::list_of(1)(-2)(3)(-1)(2)(-3);
		std::vector<int> erased = boost::assign::list_of(1)(2);
		testAdjustGaussCodeForErasedCrossings(input, expected, erased);
	}
	{
		std::vector<int> input = boost::assign::list_of(1)(-4)(5)(-1)(4)(-5);
		std::vector<int> expected = boost::assign::list_of(1)(-2)(3)(-1)(2)(-3);
		std::vector<int> erased = boost::assign::list_of(2)(3);
		testAdjustGaussCodeForErasedCrossings(input, expected, erased);
	}
}

TEST_F(KnotIdentificationBehaviorTest, IdentityKnotTest1)
{
	testIdentifyKnot(std::vector<int>(), "No Knot");
}

TEST_F(KnotIdentificationBehaviorTest, IdentityKnotTest2)
{
	std::vector<int> input = boost::assign::list_of(1)(-4)(5)(-1)(4)(-5);
	testIdentifyKnot(input, "Unknown Knot");
}

TEST_F(KnotIdentificationBehaviorTest, IdentityKnotTest3)
{
	std::vector<int> input = boost::assign::list_of(1)(-2)(3)(-1)(2)(-3);
	testIdentifyKnot(input, "Trefoil Knot");
}

TEST_F(KnotIdentificationBehaviorTest, IdentityKnotTest4)
{
	std::vector<int> input = boost::assign::list_of(-1)(2)(-3)(1)(-2)(3);
	testIdentifyKnot(input, "Trefoil Knot");
}

TEST_F(KnotIdentificationBehaviorTest, IdentityKnotTest5)
{
	std::vector<int> input = boost::assign::list_of(-3)(1)(-2)(3)(-1)(2);
	testIdentifyKnot(input, "Trefoil Knot");
}

TEST_F(KnotIdentificationBehaviorTest, IdentityKnotTest6)
{
	std::vector<int> input = boost::assign::list_of(1)(-2)(3)(-4)(5)(-6)(4)(-5)(6)(-1)(2)(-3);
	testIdentifyKnot(input, "Granny Knot");
}

TEST_F(KnotIdentificationBehaviorTest, IdentityKnotTest7)
{
	std::vector<int> input = boost::assign::list_of(-1)(2)(-3)(1)(-2)(3)(-4)(5)(-6)(4)(-5)(6);
	testIdentifyKnot(input, "Granny Knot");
}

TEST_F(KnotIdentificationBehaviorTest, IdentityKnotTest8)
{
	std::vector<int> input = boost::assign::list_of(4)(-5)(6)(-1)(2)(-3)(1)(-2)(3)(-4)(5)(-6);
	testIdentifyKnot(input, "Granny Knot");
}

TEST_F(KnotIdentificationBehaviorTest, IdentityKnotTest9)
{
	std::vector<int> input = boost::assign::list_of(-1)(2)(-3)(4)(-5)(6)(-4)(5)(-6)(1)(-2)(3);
	testIdentifyKnot(input, "Granny Knot");
}

TEST_F(KnotIdentificationBehaviorTest, IdentityKnotTest10)
{
	std::vector<int> input = boost::assign::list_of(1)(-2)(3)(4)(-5)(6)(-4)(5)(-6)(-1)(2)(-3);
	testIdentifyKnot(input, "Square Knot");
}

}
}
