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
/// Tests for the OsgPlaneActor class.

#include <SurgSim/Graphics/OsgView.h>
#include <osgDB/WriteFile>

#include <SurgSim/Graphics/OsgManager.h>
#include <SurgSim/Graphics/OsgPlaneActor.h>
#include <SurgSim/Graphics/OsgViewElement.h>
#include <SurgSim/Framework/Scene.h>
#include <SurgSim/Framework/SceneElement.h>
#include <SurgSim/Framework/Runtime.h>
#include <SurgSim/Math/Quaternion.h>
#include <SurgSim/Math/Vector.h>

#include <gtest/gtest.h>

#include <random>

using SurgSim::Framework::Runtime;
using SurgSim::Framework::Scene;
using SurgSim::Framework::SceneElement;
using SurgSim::Math::Quaterniond;
using SurgSim::Math::RigidTransform3d;
using SurgSim::Math::Vector3d;
using SurgSim::Math::makeRigidTransform;
using SurgSim::Math::makeRotationQuaternion;

namespace SurgSim
{

namespace Graphics
{

TEST(OsgPlaneActorTests, InitTest)
{
	ASSERT_NO_THROW({std::shared_ptr<Actor> actor = std::make_shared<OsgPlaneActor>("test name");});

	std::shared_ptr<Actor> actor = std::make_shared<OsgPlaneActor>("test name");
	EXPECT_EQ("test name", actor->getName());
}

TEST(OsgPlaneActorTests, VisibilityTest)
{
	std::shared_ptr<Actor> actor = std::make_shared<OsgPlaneActor>("test name");

	actor->setVisible(true);
	EXPECT_TRUE(actor->isVisible());

	actor->setVisible(false);
	EXPECT_FALSE(actor->isVisible());
}

TEST(OsgPlaneActorTests, PoseTest)
{
	std::shared_ptr<Actor> actor = std::make_shared<OsgPlaneActor>("test name");

	EXPECT_TRUE(actor->getPose().isApprox(RigidTransform3d::Identity()));

	/// Create a random rigid body transform
	Vector3d translation = Vector3d::Random();
	Quaterniond quaternion = Quaterniond(SurgSim::Math::Vector4d::Random());
	quaternion.normalize();
	RigidTransform3d transform = SurgSim::Math::makeRigidTransform(quaternion, translation);

	/// Set the transform and make sure it was set correctly
	actor->setPose(transform);
	EXPECT_TRUE(actor->getPose().isApprox(transform));
}

TEST(OsgPlaneActorTests, RenderTest)
{
	/// Initial plane 1 position
	Vector3d startPosition1(0.0, -0.1, 0.0);
	/// Final plane 1 position
	Vector3d endPosition1(0.1, -0.2, 0.0);
	/// Initial plane 1 angle
	double startAngle1 = 0.0;
	/// Final plane 1 angle
	double endAngle1 = - M_PI / 4.0;
	/// Initial plane 2 position
	Vector3d startPosition2(-0.1, -0.2, 0.0);
	/// Final plane 2 position
	Vector3d endPosition2(0.1, -0.1, 0.1);
	/// Initial plane 2 angle
	double startAngle2 = -M_PI / 2.0;
	/// Final plane 2 angle
	double endAngle2 = M_PI;

	/// Number of times to step the sphere position and radius from start to end.
	/// This number of steps will be done in 1 second.
	int numSteps = 100;

	std::shared_ptr<Runtime> runtime = std::make_shared<Runtime>();
	std::shared_ptr<OsgManager> manager = std::make_shared<OsgManager>();

	runtime->addManager(manager);

	std::shared_ptr<Scene> scene = std::make_shared<Scene>();
	runtime->setScene(scene);

	/// Add a graphics view element to the scene
	std::shared_ptr<OsgViewElement> viewElement = std::make_shared<OsgViewElement>("view element");
	scene->addSceneElement(viewElement);

	/// Add the plane actor to the view element so we don't need to make another concrete scene element
	/// \todo	DK-2013-june-03	Use the BasicSceneElement when it gets moved into Framework.
	std::shared_ptr<PlaneActor> planeActor1 = std::make_shared<OsgPlaneActor>("plane actor 1");
	viewElement->addComponent(planeActor1);
	std::shared_ptr<PlaneActor> planeActor2 = std::make_shared<OsgPlaneActor>("plane actor 2");
	viewElement->addComponent(planeActor2);

	/// Run the thread
	runtime->start();
	EXPECT_TRUE(manager->isInitialized());
	boost::this_thread::sleep(boost::posix_time::milliseconds(1000));

	for (int i = 0; i < numSteps; ++i)
	{
		/// Calculate t in [0.0, 1.0]
		double t = static_cast<double>(i) / numSteps;
		/// Interpolate pose
		planeActor1->setPose(makeRigidTransform(makeRotationQuaternion((1.0 - t) * startAngle1 + t * endAngle1,
			Vector3d(1.0, 0.0, 0.0)), (1.0 - t) * startPosition1 + t * endPosition1));
		planeActor2->setPose(makeRigidTransform(makeRotationQuaternion((1.0 - t) * startAngle2 + t * endAngle2,
			Vector3d(0.0, 0.0, 1.0)), (1.0 - t) * startPosition2 + t * endPosition2));
		boost::this_thread::sleep(boost::posix_time::milliseconds(1000 / numSteps));
	}

	runtime->stop();
}

};  // namespace Graphics

};  // namespace SurgSim
