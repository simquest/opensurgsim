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
/// Unit Tests for the OsgSkeletonRepresentation class.

#include <memory>

#include <gtest/gtest.h>

#include "SurgSim/Framework/FrameworkConvert.h"
#include "SurgSim/Framework/Runtime.h"
#include "SurgSim/Framework/Scene.h"
#include "SurgSim/Graphics/OsgManager.h"
#include "SurgSim/Graphics/OsgSkeletonRepresentation.h"
#include "SurgSim/Graphics/OsgViewElement.h"
#include "SurgSim/Graphics/OsgModel.h"

using SurgSim::Graphics::OsgSkeletonRepresentation;
using SurgSim::Graphics::OsgViewElement;
using SurgSim::Graphics::SkeletonRepresentation;

class OsgSkeletonRepresentationTest: public ::testing::Test
{
public:
	void SetUp() override
	{
		skeletonObject = std::make_shared<OsgSkeletonRepresentation>("test");
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

	std::shared_ptr<SurgSim::Graphics::OsgSkeletonRepresentation> skeletonObject;
	std::shared_ptr<SurgSim::Framework::Runtime> runtime;
	std::shared_ptr<SurgSim::Graphics::OsgManager> manager;
	std::shared_ptr<SurgSim::Framework::Scene> scene;
	std::shared_ptr<SurgSim::Graphics::OsgViewElement> viewElement;
};

TEST_F(OsgSkeletonRepresentationTest, FileNameTest)
{
	skeletonObject->loadModel("OsgSkeletonRepresentationTests/rigged_cylinder.osgt");
	EXPECT_EQ("OsgSkeletonRepresentationTests/rigged_cylinder.osgt", skeletonObject->getModel()->getFileName());
}

TEST_F(OsgSkeletonRepresentationTest, SkinningShaderFileNameTest)
{
	skeletonObject->setSkinningShaderFileName("Shaders/skinning.vert");
	EXPECT_EQ("Shaders/skinning.vert", skeletonObject->getSkinningShaderFileName());
}

TEST_F(OsgSkeletonRepresentationTest, InitTest)
{
	skeletonObject->loadModel("OsgSkeletonRepresentationTests/rigged_cylinder.osgt");
	skeletonObject->setSkinningShaderFileName("Shaders/skinning.vert");
	EXPECT_NO_THROW(viewElement->addComponent(skeletonObject));
}

TEST_F(OsgSkeletonRepresentationTest, AccessibleTest)
{
	std::shared_ptr<SurgSim::Framework::Component> component;
	ASSERT_NO_THROW(component = SurgSim::Framework::Component::getFactory().create(
		"SurgSim::Graphics::OsgSkeletonRepresentation",
		"skeleton"));

	std::string fileName("OsgSkeletonRepresentationTests/rigged_cylinder.osgt");
	component->setValue("ModelFileName", fileName);
	std::string skinningShaderFileName("Shaders/skinning.vert");
	component->setValue("SkinningShaderFileName", skinningShaderFileName);

	auto asset = component->getValue<std::shared_ptr<SurgSim::Graphics::Model>>("Model");
	EXPECT_EQ(fileName, asset->getFileName());
	auto shaderName = component->getValue<std::string>("SkinningShaderFileName");
	EXPECT_EQ(skinningShaderFileName, shaderName);
}

TEST_F(OsgSkeletonRepresentationTest, SerializationTests)
{
	std::shared_ptr<OsgSkeletonRepresentation> skeleton = std::make_shared<OsgSkeletonRepresentation>("OsgScenery");

	std::string fileName("OsgSkeletonRepresentationTests/rigged_cylinder.osgt");
	skeleton->loadModel(fileName);
	std::string skinningShaderFileName("Shaders/skinning.vert");
	skeleton->setSkinningShaderFileName(skinningShaderFileName);

	YAML::Node node;
	ASSERT_NO_THROW(node = skeleton->encode());
	EXPECT_TRUE(node.IsMap());
	EXPECT_EQ(6u, node.size());

	std::shared_ptr<OsgSkeletonRepresentation> result = std::make_shared<OsgSkeletonRepresentation>("OsgScenery");
	ASSERT_NO_THROW(result->decode(node));
	EXPECT_EQ("SurgSim::Graphics::OsgSkeletonRepresentation", result->getClassName());
	EXPECT_EQ(fileName, result->getModel()->getFileName());
	EXPECT_EQ(skinningShaderFileName, result->getSkinningShaderFileName());
}
