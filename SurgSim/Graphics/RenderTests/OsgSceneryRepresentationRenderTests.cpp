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
/// Render Tests for OsgSceneryRepresentation class.

#include <SurgSim/Framework/Runtime.h>
#include <SurgSim/Framework/Scene.h>
#include <SurgSim/Framework/SceneElement.h>
#include <SurgSim/Graphics/OsgCamera.h>
#include <SurgSim/Graphics/OsgManager.h>
#include <SurgSim/Graphics/OsgViewElement.h>
#include <SurgSim/Graphics/OsgSceneryRepresentation.h>
#include <SurgSim/Math/Quaternion.h>
#include <SurgSim/Math/RigidTransform.h>
#include <SurgSim/Math/Vector.h>

#include <memory>

#include <gtest/gtest.h>

#include <SurgSim/Graphics/OsgSphereRepresentation.h>

namespace SurgSim
{
namespace Graphics
{

struct OsgSceneryRepresentationRenderTests : public ::testing::Test
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

		manager->getDefaultCamera()->setInitialPose(
			SurgSim::Math::makeRigidTransform(SurgSim::Math::Quaterniond::Identity(),
											  SurgSim::Math::Vector3d(0.0, 0.5, 200.0)));
	}

	virtual void TearDown()
	{
		runtime->stop();
	}

	std::shared_ptr<SurgSim::Framework::Runtime> runtime;
	std::shared_ptr<SurgSim::Graphics::OsgManager> manager;
	std::shared_ptr<SurgSim::Framework::Scene> scene;
	std::shared_ptr<OsgViewElement> viewElement;
};

TEST_F(OsgSceneryRepresentationRenderTests, RenderTest)
{
	std::shared_ptr<OsgSceneryRepresentation> sceneryObject =
		std::make_shared<OsgSceneryRepresentation>("Table");
	sceneryObject->setFileName("OsgSceneryRepresentationTests/table_extension.obj");
	viewElement->addComponent(sceneryObject);

	runtime->start();

	boost::this_thread::sleep(boost::posix_time::milliseconds(3000));
}


}  // namespace Graphics
}  // namespace SurgSim