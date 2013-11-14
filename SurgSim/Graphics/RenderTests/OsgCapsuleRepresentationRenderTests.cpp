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
/// Render Tests for the OsgCapsuleRepresentation class.

#include <SurgSim/Graphics/OsgManager.h>
#include <SurgSim/Graphics/OsgCapsuleRepresentation.h>
#include <SurgSim/Graphics/OsgViewElement.h>
#include <SurgSim/Framework/Scene.h>
#include <SurgSim/Framework/SceneElement.h>
#include <SurgSim/Framework/Runtime.h>
#include <SurgSim/Math/Vector.h>
#include <SurgSim/Testing/MathUtilities.h>

#include <gtest/gtest.h>

using SurgSim::Framework::Runtime;
using SurgSim::Framework::Scene;
using SurgSim::Framework::SceneElement;
using SurgSim::Math::RigidTransform3d;
using SurgSim::Math::Vector2d;
using SurgSim::Math::Vector3d;
using SurgSim::Math::makeRigidTransform;
using SurgSim::Math::makeRotationQuaternion;
using SurgSim::Testing::interpolate;
using SurgSim::Testing::interpolatePose;

namespace SurgSim
{

namespace Graphics
{

struct OsgCapsuleRepresentationRenderTests : public ::testing::Test
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



TEST_F(OsgCapsuleRepresentationRenderTests, MovingCapsuleTest)
{
	/// Add the two capsule representation to the view element
	std::shared_ptr<CapsuleRepresentation> capsuleRepresentation1 =
		std::make_shared<OsgCapsuleRepresentation>("capsule representation 1");
	viewElement->addComponent(capsuleRepresentation1);
	std::shared_ptr<CapsuleRepresentation> capsuleRepresentation2 =
		std::make_shared<OsgCapsuleRepresentation>("capsule representation 2");
	viewElement->addComponent(capsuleRepresentation2);

	/// Run the thread
	runtime->start();
	EXPECT_TRUE(manager->isInitialized());
	boost::this_thread::sleep(boost::posix_time::milliseconds(1000));

	enum SetterType {SetterTypeIndividual,
					 SetterTypeTogether,
					 SetterTypeVector2d,
					 // Add more setter types above this line.
					 BoxSetterTypeCount};
	int setterType = 0;

	Vector2d capsule1Size, capsule2Size;

	/// Initial positoin of capsule 1
	Vector3d startPosition1(-0.1, 0.0, -0.2);
	/// Final position of capsule 1
	Vector3d endPosition1(0.1, 0.0, -0.2);
	/// Initial size (radius, hegiht) of capsule 1
	Vector2d startSize1(0.001, 0.011);
	/// Final size (radius, hegiht) of capsule 1
	Vector2d endSize1(0.01, 0.02);
	/// Initial angles (X, Y, Z) of the capsule 1
	Vector3d startAngles1(0.0, 0.0, 0.0);
	/// Final angles (X, Y, Z) of the capsule 1
	Vector3d endAngles1(-M_PI_4, -M_PI_4, -M_PI_4);

	/// Initial position capsule 2
	Vector3d startPosition2(0.0, -0.1, -0.2);
	/// Final position capsule 2
	Vector3d endPosition2(0.0, 0.1, -0.2);
	/// Initial size (radius, hegiht) of capsule 2
	Vector2d startSize2(0.001, 0.01);
	/// Final size (radius, hegiht) of capsule 2
	Vector2d endSize2(0.011, 0.02);
	/// Initial angles (X, Y, Z) of the capsule 2
	Vector3d startAngles2(-M_PI_2, -M_PI_2, -M_PI_2);
	/// Final angles (X, Y, Z) of the capsule 2
	Vector3d endAngles2(M_PI, M_PI, M_PI);

	/// Number of times to step the capsule position and radius from start to end.
	/// This number of steps will be done in 1 second.
	int numSteps = 100;
	for (int i = 0; i < numSteps; ++i)
	{
		/// Calculate t in [0.0, 1.0]
		double t = static_cast<double>(i) / numSteps;
		/// Interpolate position and angle
		capsuleRepresentation1->setPose(interpolatePose(startAngles1, endAngles1, startPosition1, endPosition1, t));
		capsuleRepresentation2->setPose(interpolatePose(startAngles2, endAngles2, startPosition2, endPosition2, t));

		/// Test different ways to set the size of capsule
		if(setterType == static_cast<int>(SetterTypeIndividual))
		{
			capsuleRepresentation1->setRadius(interpolate(startSize1.x(), endSize1.x(), t));
			capsuleRepresentation1->setHeight(interpolate(startSize1.y(), endSize1.y(), t));

			capsuleRepresentation2->setRadius(interpolate(startSize2.x(), endSize2.x(), t));
			capsuleRepresentation2->setHeight(interpolate(startSize2.y(), endSize2.y(), t));
		}
		else if(setterType == static_cast<int>(SetterTypeTogether))
		{
			capsuleRepresentation1->setSize(interpolate(startSize1.x(), endSize1.x(), t),
					interpolate(startSize1.y(), endSize1.y(), t));

			capsuleRepresentation2->setSize(interpolate(startSize2.x(), endSize2.x(), t),
					interpolate(startSize2.y(), endSize2.y(), t));
		}
		else if(setterType == static_cast<int>(SetterTypeVector2d))
		{
			capsule1Size.x() = interpolate(startSize1.x(), endSize1.x(), t);
			capsule1Size.y() = interpolate(startSize1.y(), endSize1.y(), t);
			capsuleRepresentation1->setSize(capsule1Size);

			capsule2Size.x() = interpolate(startSize2.x(), endSize2.x(), t);
			capsule2Size.y() = interpolate(startSize2.y(), endSize2.y(), t);
			capsuleRepresentation2->setSize(capsule2Size);
		}
		setterType = (setterType + 1) % BoxSetterTypeCount;
		/// The total number of steps should complete in 1 second
		boost::this_thread::sleep(boost::posix_time::milliseconds(1000 / numSteps));
	}

}

};  // namespace Graphics

};  // namespace SurgSim
