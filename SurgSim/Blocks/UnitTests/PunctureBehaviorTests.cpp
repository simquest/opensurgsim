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


#include "SurgSim/Blocks/PunctureBehavior.h"
#include "SurgSim/Framework/BasicSceneElement.h"
#include "SurgSim/Framework/Runtime.h"
#include "SurgSim/Framework/Scene.h"

namespace SurgSim
{
namespace Blocks
{

class PunctureBehaviorTests : public ::testing::Test
{
public:
	void SetUp()
	{
		runtime = std::make_shared<Framework::Runtime>("");
		scene = runtime->getScene();
	}

	void TearDown()
	{
	}

	std::shared_ptr<Framework::Runtime> runtime;
	std::shared_ptr<Framework::Scene> scene;
};

TEST_F(PunctureBehaviorTests, InitTest)
{
	ASSERT_NO_THROW({auto punctureBehavior = std::make_shared<PunctureBehavior>("test");});
}

TEST_F(PunctureBehaviorTests, SetterGetter)
{
	auto element = std::make_shared<SurgSim::Framework::BasicSceneElement>("element");
	auto punctureBehavior = std::make_shared<PunctureBehavior>("punctureBehavior");

	auto suture = std::make_shared<SurgSim::Physics::Fem1DRepresentation>("suture");
	auto tissue = std::make_shared<SurgSim::Physics::Fem2DRepresentation>("tissue");

	ASSERT_NO_THROW(punctureBehavior->setSuture(suture));
	EXPECT_EQ(suture, punctureBehavior->getSuture());

	ASSERT_NO_THROW(punctureBehavior->setTissue(tissue));
	EXPECT_EQ(tissue, punctureBehavior->getTissue());

	ASSERT_NO_THROW(punctureBehavior->setProximity(1.0));
	EXPECT_EQ(1.0, punctureBehavior->getProximity());
}

};
};
