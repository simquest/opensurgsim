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

#include <random>
#include <gtest/gtest.h>

namespace SurgSim
{
namespace Graphics
{

using SurgSim::Math::Quaterniond;
using SurgSim::Math::RigidTransform3d;
using SurgSim::Math::Vector3d;
using SurgSim::Math::makeRigidTransform;
using SurgSim::Math::makeRotationQuaternion;

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
											  SurgSim::Math::Vector3d(0.0, 0.5, 10.0)));
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
	/// Initial position of object 1
	Vector3d startPosition1(-5.0, 0.0, -2.0);
	/// Final position of object 1
	Vector3d endPosition1(5.0, 0.0, -2.0);

	/// Initial and final angleX of object 1
	double startAngleX1 = 0.0, endAngleX1 = - M_PI / 4.0;
	/// Initial and final angleY of object 1
	double startAngleY1 = 0.0, endAngleY1 = - M_PI / 4.0;
	/// Initial and final angleZ of object 1
	double startAngleZ1 = 0.0, endAngleZ1 = - M_PI / 4.0;

	/// Initial position of object 2
	Vector3d startPosition2(0.0, -5.0, -2.0);
	/// Final position of object 2
	Vector3d endPosition2(0.0, 5.0, -2.0);

	/// Initial and final angleX of object 2
	double startAngleX2 = -M_PI / 2.0, endAngleX2 = M_PI;
	/// Initial and final angleY of object 2
	double startAngleY2 = -M_PI / 2.0, endAngleY2 = M_PI;
	/// Initial and final angleZ of object 2
	double startAngleZ2 = -M_PI / 2.0, endAngleZ2 = M_PI;

	/// Number of times to step the objects' position from start to end.
	/// This number of steps will be done in 1 second.
	int numSteps = 100;

	auto sceneryObject1 = std::make_shared<OsgSceneryRepresentation>("Cube");
	sceneryObject1->setFileName("OsgSceneryRepresentationTests/Torus.obj");
	sceneryObject1->setInitialPose(SurgSim::Math::makeRigidTransform(
								SurgSim::Math::Quaterniond::Identity(),
								startPosition1));
	viewElement->addComponent(sceneryObject1);

	auto sceneryObject2 = std::make_shared<OsgSceneryRepresentation>("Box");
	sceneryObject2->setFileName("OsgSceneryRepresentationTests/Torus.osgb");
	sceneryObject2->setInitialPose(SurgSim::Math::makeRigidTransform(
								SurgSim::Math::Quaterniond::Identity(),
								startPosition2));
	viewElement->addComponent(sceneryObject2);

	runtime->start();
	EXPECT_TRUE(manager->isInitialized());

	for (int i = 0; i < numSteps; ++i)
	{
		/// Calculate t in [0.0, 1.0]
		double t = static_cast<double>(i) / numSteps;
		/// Interpolate position
		sceneryObject1->setPose(makeRigidTransform(
			makeRotationQuaternion((1.0 - t) * startAngleX1 + t * endAngleX1, Vector3d(1.0, 0.0, 0.0)) *
			makeRotationQuaternion((1.0 - t) * startAngleY1 + t * endAngleY1, Vector3d(0.0, 1.0, 0.0)) *
			makeRotationQuaternion((1.0 - t) * startAngleZ1 + t * endAngleZ1, Vector3d(0.0, 0.0, 1.0)),
			(1.0 - t) * startPosition1 + t * endPosition1));
		sceneryObject2->setPose(makeRigidTransform(
			makeRotationQuaternion((1.0 - t) * startAngleX2 + t * endAngleX2, Vector3d(1.0, 0.0, 0.0)) *
			makeRotationQuaternion((1.0 - t) * startAngleY2 + t * endAngleY2, Vector3d(0.0, 1.0, 0.0)) *
			makeRotationQuaternion((1.0 - t) * startAngleZ2 + t * endAngleZ2, Vector3d(0.0, 0.0, 1.0)),
			(1.0 - t) * startPosition2 + t * endPosition2));

		/// The total number of steps should complete in 1 second
		boost::this_thread::sleep(boost::posix_time::milliseconds(1000 / numSteps));
	}
}


}  // namespace Graphics
}  // namespace SurgSim