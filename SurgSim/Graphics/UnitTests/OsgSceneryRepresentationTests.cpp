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

#include <osg/Group>
#include <osg/Geode>
#include <osg/Geometry>

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
	sceneryObject->loadModel("Geometry/Torus.obj");
	EXPECT_EQ("Geometry/Torus.obj", sceneryObject->getModel()->getFileName());

}

TEST_F(OsgSceneryRepresentationTest, InitTest)
{
	EXPECT_EQ(nullptr, sceneryObject->getModelNode());
	sceneryObject->loadModel("Geometry/Torus.obj");
	EXPECT_NO_THROW(viewElement->addComponent(sceneryObject));
	EXPECT_NE(nullptr, sceneryObject->getModelNode());

	sceneryObject2->loadModel("Geometry/Torus.osgb");
	EXPECT_NO_THROW(viewElement->addComponent(sceneryObject2));
}

TEST_F(OsgSceneryRepresentationTest, AccessibleTest)
{
	std::shared_ptr<SurgSim::Framework::Component> component;
	ASSERT_NO_THROW(component = SurgSim::Framework::Component::getFactory().create(
									"SurgSim::Graphics::OsgSceneryRepresentation",
									"scenery"));

	std::string fileName("Geometry/Torus.obj");
	component->setValue("ModelFileName", fileName);
	auto asset = component->getValue<std::shared_ptr<SurgSim::Graphics::Model>>("Model");
	EXPECT_EQ(fileName, asset->getFileName());
}

TEST_F(OsgSceneryRepresentationTest, SerializationTests)
{
	std::shared_ptr<SceneryRepresentation> scenery = std::make_shared<OsgSceneryRepresentation>("OsgScenery");

	std::string fileName("Geometry/Torus.obj");
	scenery->loadModel(fileName);

	YAML::Node node;
	ASSERT_NO_THROW(node = scenery->encode());
	EXPECT_TRUE(node.IsMap());

	std::shared_ptr<SceneryRepresentation> result = std::make_shared<OsgSceneryRepresentation>("OsgScenery");
	ASSERT_NO_THROW(result->decode(node));
	EXPECT_EQ("SurgSim::Graphics::OsgSceneryRepresentation", result->getClassName());
	EXPECT_EQ(fileName, result->getModel()->getFileName());
}

// This checks a fix for an intermittent bug where on deserialization setGenerateTangents would sometimes be called
// before setModel, in which case no tangents where generated on scenery representations
TEST_F(OsgSceneryRepresentationTest, GenerateTangents)
{
	{
		SCOPED_TRACE("setGenerateTangents before loadModel");
		auto object = std::make_shared<OsgSceneryRepresentation>("representation");

		object->setGenerateTangents(true);
		object->loadModel("Geometry/cube.osgt");

		// Structure from the osgt file, we need to get to the geometry to make sure the tangents where generated
		auto group = object->getModelNode()->asGroup();
		ASSERT_NE(nullptr, group);
		auto geode = group->getChild(0)->asGeode();
		ASSERT_NE(nullptr, geode);
		auto geometry = geode->getDrawable(0)->asGeometry();
		ASSERT_NE(nullptr, geometry);

		ASSERT_NE(nullptr, geometry->getVertexAttribArray(SurgSim::Graphics::TANGENT_VERTEX_ATTRIBUTE_ID));
		ASSERT_NE(nullptr, geometry->getVertexAttribArray(SurgSim::Graphics::BITANGENT_VERTEX_ATTRIBUTE_ID));
	}


	{
		SCOPED_TRACE("setGenerateTangents after loadModel");
		auto object = std::make_shared<OsgSceneryRepresentation>("representation");

		object->loadModel("Geometry/cube.osgt");
		object->setGenerateTangents(true);

		// Structure from the osgt file, we need to get to the geometry to make sure the tangents where generated
		auto group = object->getModelNode()->asGroup();
		ASSERT_NE(nullptr, group);
		auto geode = group->getChild(0)->asGeode();
		ASSERT_NE(nullptr, geode);
		auto geometry = geode->getDrawable(0)->asGeometry();
		ASSERT_NE(nullptr, geometry);

		ASSERT_NE(nullptr, geometry->getVertexAttribArray(SurgSim::Graphics::TANGENT_VERTEX_ATTRIBUTE_ID));
		ASSERT_NE(nullptr, geometry->getVertexAttribArray(SurgSim::Graphics::BITANGENT_VERTEX_ATTRIBUTE_ID));
	}

}
