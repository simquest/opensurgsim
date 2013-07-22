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
#include <SurgSim/Graphics/OsgBoxRepresentation.h>
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
using SurgSim::Math::makeRigidTransform;
using SurgSim::Math::makeRotationQuaternion;

namespace SurgSim
{

namespace Graphics
{


TEST(OsgRepresentationRenderTests, RepresentationTest)
{
	///	Box position
	Vector3d boxPosition(0.0, 0.0, -0.2);
	/// Cylinder position
	Vector3d cylinderPosition(-0.05, 0.0, -0.2);
	/// Sphere position
	Vector3d spherePosition(0.05, 0.0, -0.2);
	/// Size along X-axis of the box
	double boxSizeX = 0.03;
	/// Size along Y-axis of the box
	double boxSizeY = 0.05;
	/// Size along Z-axis of the box
	double boxSizeZ = 0.05;
	/// Radius along X-axis and Z-axis of the cylinder
	double cylinderRadius= 0.005;
	/// Height along Y-axis of the cylinder
	double cylinderHeight = 0.015;
	/// Radius of the sphere
	double sphereRadius = 0.006;

	std::shared_ptr<Runtime> runtime = std::make_shared<Runtime>();
	std::shared_ptr<OsgManager> manager = std::make_shared<OsgManager>();

	runtime->addManager(manager);

	std::shared_ptr<Scene> scene = std::make_shared<Scene>();
	runtime->setScene(scene);

	/// Add a graphics view element to the scene
	std::shared_ptr<OsgViewElement> viewElement = std::make_shared<OsgViewElement>("view element");
	scene->addSceneElement(viewElement);

	/// Add representations to the view element so we don't need to make another concrete scene element
	std::shared_ptr<BoxRepresentation> boxRepresentation =
		std::make_shared<OsgBoxRepresentation>("box representation");
	viewElement->addComponent(boxRepresentation);
	std::shared_ptr<CylinderRepresentation> cylinderRepresentation =
		std::make_shared<OsgCylinderRepresentation>("cylinder representation");
	viewElement->addComponent(cylinderRepresentation);
	std::shared_ptr<SphereRepresentation> sphereRepresentation =
		std::make_shared<OsgSphereRepresentation>("sphere representation");
	viewElement->addComponent(sphereRepresentation);

	/// Run the thread
	runtime->start();
	EXPECT_TRUE(manager->isInitialized());

	/// Interpolate position for box
	boxRepresentation->setPose(makeRigidTransform(Quaterniond::Identity(), boxPosition ));
	/// Interpolate position for cylinder
	cylinderRepresentation->setPose(makeRigidTransform(Quaterniond::Identity(), cylinderPosition ));
	/// Interpolate position for sphere
	sphereRepresentation->setPose(makeRigidTransform(Quaterniond::Identity(), spherePosition ));

	/// Set the size of box
	boxRepresentation->setSize(boxSizeX, boxSizeY, boxSizeZ);
	/// Set the size of cylinder
	cylinderRepresentation->setSize(cylinderRadius, cylinderHeight);
	/// Set the size of sphere
	sphereRepresentation->setRadius(sphereRadius);

	boost::this_thread::sleep(boost::posix_time::milliseconds(3000));
	runtime->stop();
}

};  // namespace Graphics

};  // namespace SurgSim
