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
#include "MockObjects.h"
#include <SurgSim/Framework/Runtime.h>
#include <SurgSim/Framework/Scene.h>

using namespace SurgSim::Framework;

TEST(SceneTest, ConstructorTest)
{
	ASSERT_NO_THROW({Scene scene;});
}

TEST(SceneTest, ElementManagement)
{
	std::shared_ptr<Scene> scene(new Scene());
	std::shared_ptr<MockSceneElement> element1(new MockSceneElement("one"));
	std::shared_ptr<MockSceneElement> element2(new MockSceneElement("two"));

	EXPECT_EQ(0U, scene->getSceneElements().size());

	EXPECT_TRUE(scene->addSceneElement(element1));
	EXPECT_EQ(1U, scene->getSceneElements().size());
	EXPECT_TRUE(scene->addSceneElement(element2));
	EXPECT_EQ(2U, scene->getSceneElements().size());

	EXPECT_FALSE(scene->addSceneElement(element1));
	EXPECT_EQ(2U, scene->getSceneElements().size());

	EXPECT_EQ(element1, scene->getSceneElement("one"));
	EXPECT_EQ(element2, scene->getSceneElement("two"));
	EXPECT_EQ(nullptr, scene->getSceneElement("nonexistentelement"));
}