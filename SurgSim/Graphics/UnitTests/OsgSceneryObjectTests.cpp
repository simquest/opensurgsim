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
/// Tests for the OsgSceneryObject class.

#include <SurgSim/Framework/ApplicationData.h>
#include <SurgSim/Framework/Runtime.h>
#include <SurgSim/Framework/Scene.h>
#include <SurgSim/Graphics/OsgManager.h>
#include <SurgSim/Graphics/OsgSceneryObject.h>
#include <SurgSim/Graphics/OsgViewElement.h>

#include <memory>
#include <boost/filesystem.hpp>

#include <gtest/gtest.h>


namespace SurgSim
{
namespace Graphics
{

TEST(OsgSceneryObjectTest, InitTest)
{
	auto sceneryObject = std::make_shared<OsgSceneryObject>("test", "Data/OsgSceneryObjectTests/table_extension.obj");
	auto runtime = std::make_shared<SurgSim::Framework::Runtime>();
	auto manager = std::make_shared<SurgSim::Graphics::OsgManager>();
	auto scene = std::make_shared<SurgSim::Framework::Scene>();
	auto viewElement = std::make_shared<OsgViewElement>("view element");

	viewElement->addComponent(sceneryObject);
	scene->addSceneElement(viewElement);
	runtime->addManager(manager);
	runtime->setScene(scene);

	sceneryObject->initialize(runtime);
	EXPECT_NE(nullptr, sceneryObject->getOsgSceneryObject());
}


}  // namespace Graphics
}  // namespace SurgSim