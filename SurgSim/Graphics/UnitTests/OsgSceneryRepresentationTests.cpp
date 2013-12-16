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

#include "SurgSim/Framework/Runtime.h"
#include "SurgSim/Framework/Scene.h"
#include "SurgSim/Graphics/OsgManager.h"
#include "SurgSim/Graphics/OsgSceneryRepresentation.h"
#include "SurgSim/Graphics/OsgViewElement.h"

#include <memory>

#include <gtest/gtest.h>


namespace SurgSim
{
namespace Graphics
{

class OsgSceneryRepresentationTest: public ::testing::Test
{
public:
	virtual void SetUp() override
	{
		sceneryObject = std::make_shared<OsgSceneryRepresentation>("test");
		sceneryObject2 = std::make_shared<OsgSceneryRepresentation>("test2");
		runtime = std::make_shared<SurgSim::Framework::Runtime>();
		manager = std::make_shared<SurgSim::Graphics::OsgManager>();
		scene = runtime->getScene();
		viewElement = std::make_shared<OsgViewElement>("view element");

		viewElement->addComponent(sceneryObject);
		viewElement->addComponent(sceneryObject2);
		scene->addSceneElement(viewElement);
		runtime->addManager(manager);
		
	}

	virtual void TearDown() override
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
	sceneryObject->setFileName("Data/OsgSceneryRepresentationTests/Torus.obj");
	EXPECT_EQ("Data/OsgSceneryRepresentationTests/Torus.obj", sceneryObject->getFileName());
}

TEST_F(OsgSceneryRepresentationTest, InitTest)
{
	sceneryObject->setFileName("Data/OsgSceneryRepresentationTests/Torus.obj");
	ASSERT_NO_THROW(sceneryObject->initialize(runtime));

	sceneryObject2->setFileName("Data/OsgSceneryRepresentationTests/Torus.osgb");
	ASSERT_NO_THROW(sceneryObject2->initialize(runtime));
}


}  // namespace Graphics
}  // namespace SurgSim