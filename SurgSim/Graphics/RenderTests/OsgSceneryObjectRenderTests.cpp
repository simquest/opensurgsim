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
/// Render Tests for the OsgSceneryObject class.

#include <SurgSim/Framework/ApplicationData.h>
#include <SurgSim/Framework/Runtime.h>
#include <SurgSim/Framework/Scene.h>
#include <SurgSim/Framework/SceneElement.h>
#include <SurgSim/Graphics/OsgManager.h>
#include <SurgSim/Graphics/OsgViewElement.h>
#include <SurgSim/Graphics/OsgSceneryObject.h>

#include <memory>
#include <boost/filesystem.hpp>

#include <gtest/gtest.h>
#include <SurgSim/Graphics/OsgSphereRepresentation.h>

namespace SurgSim
{
namespace Graphics
{

struct OsgSceneryObjectRenderTests : public ::testing::Test
{
	virtual void SetUp()
	{
		runtime = std::make_shared<SurgSim::Framework::Runtime>();
		manager = std::make_shared<SurgSim::Graphics::OsgManager>();

		runtime->addManager(manager);

		scene = std::make_shared<SurgSim::Framework::Scene>();
		runtime->setScene(scene);

		viewElement = std::make_shared<OsgViewElement>("view element");
		scene->addSceneElement(viewElement);

	}

	virtual void TearDown()
	{
		runtime->stop();
	}

	std::shared_ptr<SurgSim::Framework::Runtime> runtime;
	std::shared_ptr<SurgSim::Graphics::OsgManager> manager;
	std::shared_ptr<SurgSim::Framework::Scene> scene;
	std::shared_ptr<OsgViewElement> viewElement;

protected:

};

TEST_F(OsgSceneryObjectRenderTests, RenderTest)
{
	std::shared_ptr<OsgSceneryObject> sceneryObject =
		std::make_shared<OsgSceneryObject>("Table", "Data/OsgSceneryObjectTests/table_extension.obj");
	viewElement->addComponent(sceneryObject);

	runtime->start();
	EXPECT_TRUE(manager->isInitialized());
}


}  // namespace Graphics
}  // namespace SurgSim