// This file is a part of the OpenSurgSim project.
// Copyright 2013-2016, SimQuest Solutions Inc.
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

#include "SurgSim/Framework/Runtime.h"
#include "SurgSim/Framework/TransferPropertiesBehavior.h"


namespace {

class A : public SurgSim::Framework::Accessible
{
public:
	explicit A(int initialA = 0, int initialB = 0) : a(initialA), b(initialB)
	{
		SURGSIM_ADD_RW_PROPERTY(A,int,a,getA,setA);
		SURGSIM_ADD_RW_PROPERTY(A,int,b,getB,setB);
	}

	int a;
	int getA() const { return a; }
	void setA(int val) { a = val; }

	int b;
	int getB() const { return b; }
	void setB(int val) { b = val; }
};

};

namespace SurgSim
{
namespace Framework
{

TEST(TransferPropertiesBehaviorTest, InitTest)
{
	ASSERT_NO_THROW({TransferPropertiesBehavior b("TestName");});
}

TEST(TransferPropertiesBehaviorTest, GetSetTargetManagerType)
{
	auto behavior = std::make_shared<TransferPropertiesBehavior>("test");
	auto runtime = std::make_shared<Framework::Runtime>();

	EXPECT_EQ(Framework::MANAGER_TYPE_BEHAVIOR, behavior->getTargetManagerType());

	EXPECT_THROW(behavior->setTargetManagerType(-1), Framework::AssertionFailure);
	EXPECT_THROW(behavior->setTargetManagerType(Framework::MANAGER_TYPE_COUNT), Framework::AssertionFailure);

	EXPECT_NO_THROW(behavior->setTargetManagerType(Framework::MANAGER_TYPE_PHYSICS));
	EXPECT_EQ(Framework::MANAGER_TYPE_PHYSICS, behavior->getTargetManagerType());

	EXPECT_NO_THROW(behavior->setTargetManagerType(Framework::MANAGER_TYPE_BEHAVIOR));
	EXPECT_EQ(Framework::MANAGER_TYPE_BEHAVIOR, behavior->getTargetManagerType());

	behavior->initialize(runtime);
	EXPECT_THROW(behavior->setTargetManagerType(Framework::MANAGER_TYPE_GRAPHICS), Framework::AssertionFailure);
}

TEST(TransferPropertiesBehaviorTest, ValidConnections)
{
	auto a = std::make_shared<A>();
	auto b = std::make_shared<A>();

	TransferPropertiesBehavior behavior("test");

	EXPECT_ANY_THROW(behavior.connect(nullptr, "", a ,"a"));
	EXPECT_ANY_THROW(behavior.connect(a, "a", nullptr, ""));

	EXPECT_ANY_THROW(behavior.connect(a, "a", a, "a"));

	EXPECT_ANY_THROW(behavior.connect(a, "xxx", b, "a"));
	EXPECT_ANY_THROW(behavior.connect(a, "a", b, "xxx"));

	EXPECT_TRUE(behavior.connect(a, "a", b, "b"));
}

TEST(TransferPropertiesBehaviorTest, ValidUpdates)
{
	auto runtime = std::make_shared<Runtime>();
	auto behavior = std::make_shared<TransferPropertiesBehavior>("test");
	auto a = std::make_shared<A>(1,2);
	auto b = std::make_shared<A>(3,4);
	auto c = std::make_shared<A>(5,6);

	EXPECT_TRUE(behavior->initialize(runtime));
	EXPECT_TRUE(behavior->wakeUp());

	EXPECT_TRUE(behavior->connect(a, "a", b, "a"));
	EXPECT_TRUE(behavior->connect(b, "b", c, "b"));
	EXPECT_TRUE(behavior->connect(a, "a", c, "a"));

	behavior->update(0.0);

	EXPECT_EQ(1, a->a);
	EXPECT_EQ(2, a->b);

	EXPECT_EQ(1, b->a);
	EXPECT_EQ(4, b->b);

	EXPECT_EQ(1, c->a);
	EXPECT_EQ(4, c->b);
}

}
}

