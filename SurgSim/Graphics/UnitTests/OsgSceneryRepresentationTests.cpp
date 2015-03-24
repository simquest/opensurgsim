// This file is a part of the OpenSurgSim project.
// Copyright 2012-2013, SimQuest Solutions Inc.
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
/// Unit Tests for the OsgSceneryRepresentation class.

#include <memory>

#include <gtest/gtest.h>

#include "SurgSim/Framework/FrameworkConvert.h"
#include "SurgSim/Framework/Runtime.h"
#include "SurgSim/Framework/Scene.h"
#include "SurgSim/Graphics/OsgManager.h"
#include "SurgSim/Graphics/OsgSceneryRepresentation.h"
#include "SurgSim/Graphics/OsgViewElement.h"
#include "SurgSim/Graphics/OsgModel.h"

using SurgSim::Graphics::OsgSceneryRepresentation;
using SurgSim::Graphics::OsgViewElement;
using SurgSim::Graphics::SceneryRepresentation;

class OsgSceneryRepresentationTest: public ::testing::Test
{
public:
	void SetUp() override
	{
		sceneryObject = std::make_shared<OsgSceneryRepresentation>("test");
		sceneryObject2 = std::make_shared<OsgSceneryRepresentation>("test2");
		runtime = std::make_shared<SurgSim::Framework::Runtime>("config.txt");
		manager = std::make_shared<SurgSim::Graphics::OsgManager>();
		scene = runtime->getScene();
		viewElement = std::make_shared<OsgViewElement>("view element");

		scene->addSceneElement(viewElement);
		runtime->addManager(manager);

	}

	void TearDown() override
	{
	}

	std::shared_ptr<SurgSim::Graphics::OsgSceneryRepresentation> sceneryObject;
	std::shared_ptr<SurgSim::Graphics::OsgSceneryRepresentation> sceneryObject2;
	std::shared_ptr<SurgSim::Framework::Runtime> runtime;
	std::shared_ptr<SurgSim::Graphics::OsgManager> manager;
	std::shared_ptr<SurgSim::Framework::Scene> scene;
	std::shared_ptr<SurgSim::Graphics::OsgViewElement> viewElement;
};

TEST_F(OsgSceneryRepresentationTest, FileNameTest)
{
	sceneryObject->loadModel("OsgSceneryRepresentationTests/Torus.obj");
	EXPECT_EQ("OsgSceneryRepresentationTests/Torus.obj", sceneryObject->getModel()->getFileName());
}

TEST_F(OsgSceneryRepresentationTest, InitTest)
{
	sceneryObject->loadModel("OsgSceneryRepresentationTests/Torus.obj");
	EXPECT_NO_THROW(viewElement->addComponent(sceneryObject));

	sceneryObject2->loadModel("OsgSceneryRepresentationTests/Torus.osgb");
	EXPECT_NO_THROW(viewElement->addComponent(sceneryObject2));
}

TEST_F(OsgSceneryRepresentationTest, AccessibleTest)
{
	std::shared_ptr<SurgSim::Framework::Component> component;
	ASSERT_NO_THROW(component = SurgSim::Framework::Component::getFactory().create(
									"SurgSim::Graphics::OsgSceneryRepresentation",
									"scenery"));

	std::string fileName("OsgSceneryRepresentationTests/Torus.obj");
	component->setValue("ModelFileName", fileName);
	auto asset = component->getValue<std::shared_ptr<SurgSim::Graphics::Model>>("Model");
	EXPECT_EQ(fileName, asset->getFileName());
}

TEST_F(OsgSceneryRepresentationTest, SerializationTests)
{
	std::shared_ptr<SceneryRepresentation> scenery = std::make_shared<OsgSceneryRepresentation>("OsgScenery");

	std::string fileName("OsgSceneryRepresentationTests/Torus.obj");
	scenery->loadModel(fileName);

	YAML::Node node;
	ASSERT_NO_THROW(node = scenery->encode());
	EXPECT_TRUE(node.IsMap());

	std::shared_ptr<SceneryRepresentation> result = std::make_shared<OsgSceneryRepresentation>("OsgScenery");
	ASSERT_NO_THROW(result->decode(node));
	EXPECT_EQ("SurgSim::Graphics::OsgSceneryRepresentation", result->getClassName());
	EXPECT_EQ(fileName, result->getModel()->getFileName());
}
