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
/// Render Tests for the OsgBoxRepresentation class.

#include <SurgSim/Graphics/OsgManager.h>
#include <SurgSim/Graphics/OsgAxesRepresentation.h>
#include <SurgSim/Graphics/OsgBoxRepresentation.h>
#include <SurgSim/Graphics/OsgCapsuleRepresentation.h>
#include <SurgSim/Graphics/OsgCylinderRepresentation.h>
#include <SurgSim/Graphics/OsgSphereRepresentation.h>
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
using SurgSim::Math::Vector2d;
using SurgSim::Math::makeRigidTransform;
using SurgSim::Math::makeRotationQuaternion;

namespace SurgSim
{

namespace Graphics
{

struct OsgRepresentationRenderTests : public ::testing::Test
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

/// This test will put all shape one by one along the X-axis
/// To make sure all shapes are aligned.
/// X-axis points horizontally to the right
/// Y-axis points vertically up
/// Z-axis is perpendicular to the screen and points out
TEST_F(OsgRepresentationRenderTests, RepresentationTest)
{
	///	Box position
	Vector3d boxPosition(0.05, 0.0, -0.2);
	/// Capsule position
	Vector3d capsulePosition(-0.05, 0.0, -0.2);
	/// Cylinder position
	Vector3d cylinderPosition(-0.025, 0.0, -0.2);
	/// Sphere position
	Vector3d spherePosition(0.025, 0.0, -0.2);
	/// Size of the box
	Vector3d boxSize(0.01, 0.015, 0.01);
	/// Size of the capsule (radius, height)
	Vector2d capsuleSize(0.005, 0.015);
	/// Size of the cylinder
	Vector2d cylinderSize(0.005, 0.015);
	/// Radius of the sphere
	double sphereRadius = 0.005;

	/// Add representations to the view element so we don't need to make another concrete scene element
	std::shared_ptr<BoxRepresentation> boxRepresentation =
		std::make_shared<OsgBoxRepresentation>("box representation");
	viewElement->addComponent(boxRepresentation);

	std::shared_ptr<CapsuleRepresentation> capsuleRepresentation =
		std::make_shared<OsgCapsuleRepresentation>("capsule representation");
	viewElement->addComponent(capsuleRepresentation);

	std::shared_ptr<CylinderRepresentation> cylinderRepresentation =
		std::make_shared<OsgCylinderRepresentation>("cylinder representation");
	viewElement->addComponent(cylinderRepresentation);

	std::shared_ptr<SphereRepresentation> sphereRepresentation =
		std::make_shared<OsgSphereRepresentation>("sphere representation");
	viewElement->addComponent(sphereRepresentation);


	std::shared_ptr<AxesRepresentation> axesRepresentation =
		std::make_shared<OsgAxesRepresentation>("axes");
	viewElement->addComponent(axesRepresentation);

	/// Run the thread
	runtime->start();
	EXPECT_TRUE(manager->isInitialized());

	boxRepresentation->setPose(makeRigidTransform(Quaterniond::Identity(), boxPosition ));
	capsuleRepresentation->setPose(makeRigidTransform(Quaterniond::Identity(), capsulePosition ));
	cylinderRepresentation->setPose(makeRigidTransform(Quaterniond::Identity(), cylinderPosition ));
	sphereRepresentation->setPose(makeRigidTransform(Quaterniond::Identity(), spherePosition ));
	axesRepresentation->setPose(makeRigidTransform(Quaterniond::Identity(), Vector3d(0.0,0.0,-0.2)));

	/// Set the size of box
	boxRepresentation->setSize(boxSize.x(), boxSize.y(), boxSize.z());
	/// Set the size of capsule
	/// Capsule should use Y-axis as its axis
	capsuleRepresentation->setSize(capsuleSize.x(), capsuleSize.y());
	/// Set the size of cylinder
	/// Cylinder should use Y-axis as its axis
	cylinderRepresentation->setSize(cylinderSize.x(), cylinderSize.y());
	/// Set the size of sphere
	sphereRepresentation->setRadius(sphereRadius);

	axesRepresentation->setSize(0.01);

	boost::this_thread::sleep(boost::posix_time::milliseconds(3000));
}

};  // namespace Graphics

};  // namespace SurgSim
