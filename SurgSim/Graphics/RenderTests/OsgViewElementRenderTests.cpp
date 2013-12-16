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
/// Render Tests for the OsgViewElement class.

#include "SurgSim/Graphics/OsgManager.h"
#include "SurgSim/Graphics/OsgView.h"
#include "SurgSim/Graphics/OsgViewElement.h"

#include "SurgSim/Framework/Runtime.h"
#include "SurgSim/Framework/Scene.h"

#include <gtest/gtest.h>

using SurgSim::Framework::Runtime;
using SurgSim::Framework::Scene;

namespace SurgSim
{
namespace Graphics
{

TEST(OsgViewElementRenderTests, MoveAndResizeWindowTest)
{
	std::shared_ptr<Runtime> runtime = std::make_shared<Runtime>();
	std::shared_ptr<OsgManager> manager = std::make_shared<OsgManager>();

	runtime->addManager(manager);

	std::shared_ptr<Scene> scene = runtime->getScene();

	/// Add a graphics component to the scene
	std::shared_ptr<OsgViewElement> viewElement = std::make_shared<OsgViewElement>("test element");
	scene->addSceneElement(viewElement);

	/// Set initial position to (50, 60), dimensions to 200 x 100 and disable the window border
	viewElement->getView()->setPosition(50, 60);
	viewElement->getView()->setDimensions(200, 100);
	viewElement->getView()->setWindowBorderEnabled(false);

	/// Run the thread
	runtime->start();
	EXPECT_TRUE(manager->isInitialized());
	boost::this_thread::sleep(boost::posix_time::milliseconds(1000));

	/// Move the window to (100, 200)
	viewElement->getView()->setPosition(100, 200);
	boost::this_thread::sleep(boost::posix_time::milliseconds(1000));

	/// Enable the window border and resize the window to 400 x 500
	viewElement->getView()->setWindowBorderEnabled(true);
	viewElement->getView()->setDimensions(400, 500);
	boost::this_thread::sleep(boost::posix_time::milliseconds(1000));

	runtime->stop();
}

}  // namespace Graphics
}  // namespace SurgSim
