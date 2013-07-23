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
/// Render Tests for the OsgCylinderRepresentation class.

#include <SurgSim/Graphics/OsgManager.h>
#include <SurgSim/Graphics/OsgCylinderRepresentation.h>
#include <SurgSim/Graphics/OsgViewElement.h>
#include <SurgSim/Framework/Scene.h>
#include <SurgSim/Framework/SceneElement.h>
#include <SurgSim/Framework/Runtime.h>
#include <SurgSim/Math/Quaternion.h>
#include <SurgSim/Math/Vector.h>
#include <SurgSim/Testing/MathUtilities.h>


#include <gtest/gtest.h>

#include <random>

using SurgSim::Framework::Runtime;
using SurgSim::Framework::Scene;
using SurgSim::Framework::SceneElement;
using SurgSim::Math::Quaterniond;
using SurgSim::Math::RigidTransform3d;
using SurgSim::Math::Vector2d;
using SurgSim::Math::Vector3d;
using SurgSim::Math::makeRigidTransform;
using SurgSim::Math::makeRotationQuaternion;
using SurgSim::Testing::interpolate;


namespace SurgSim
{

namespace Graphics
{

struct OsgCylinderRepresentationRenderTests : public ::testing::Test
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

TEST_F(OsgCylinderRepresentationRenderTests, MovingCapsuleTest)
{
	/// Add the two cylinder representation to the view element so we don't need to make another concrete scene element
	std::shared_ptr<CylinderRepresentation> cylinderRepresentation1 =
		std::make_shared<OsgCylinderRepresentation>("cylinder representation 1");
	viewElement->addComponent(cylinderRepresentation1);
	std::shared_ptr<CylinderRepresentation> cylinderRepresentation2 =
		std::make_shared<OsgCylinderRepresentation>("cylinder representation 2");
	viewElement->addComponent(cylinderRepresentation2);

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

	Vector2d cylinder1Size, cylinder2Size;

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
    Vector2d startSize2(0.001, 0.011);
    /// Final size (radius, hegiht) of capsule 2
    Vector2d endSize2(0.01, 0.02);
    /// Initial angles (X, Y, Z) of the capsule 2
    Vector3d startAngles2(-M_PI_2, -M_PI_2, -M_PI_2);
    /// Final angles (X, Y, Z) of the capsule 2
    Vector3d endAngles2(M_PI, M_PI, M_PI);

	/// Number of times to step the cylinder position and radius from start to end.
	/// This number of steps will be done in 1 second.
	int numSteps = 100;
	for (int i = 0; i < numSteps; ++i)
	{
		/// Calculate t in [0.0, 1.0]
		double t = static_cast<double>(i) / numSteps;
		/// Interpolate position and radius
		cylinderRepresentation1->setPose(makeRigidTransform(
            makeRotationQuaternion(interpolate(startAngles1.x(), endAngles1.x(), t), Vector3d(1.0, 0.0, 0.0)) *
            makeRotationQuaternion(interpolate(startAngles1.y(), endAngles1.y(), t), Vector3d(0.0, 1.0, 0.0)) *
            makeRotationQuaternion(interpolate(startAngles1.z(), endAngles1.z(), t), Vector3d(0.0, 0.0, 1.0)),
            interpolate(startPosition1, endPosition1, t)));
		cylinderRepresentation2->setPose(makeRigidTransform(
			makeRotationQuaternion(interpolate(startAngles2.x(), endAngles2.x(), t), Vector3d(1.0, 0.0, 0.0)) *
            makeRotationQuaternion(interpolate(startAngles2.y(), endAngles2.y(), t), Vector3d(0.0, 1.0, 0.0)) *
            makeRotationQuaternion(interpolate(startAngles2.z(), endAngles2.z(), t), Vector3d(0.0, 0.0, 1.0)),
            interpolate(startPosition2, endPosition2, t)));
		if(setterType == static_cast<int>(SetterTypeIndividual))
		{
			cylinderRepresentation1->setRadius(interpolate(startSize1.x(), endSize1.x(), t));
			cylinderRepresentation1->setHeight(interpolate(startSize1.y(), endSize1.y(), t));

			cylinderRepresentation2->setRadius(interpolate(startSize2.x(), endSize2.x(), t));
			cylinderRepresentation2->setHeight(interpolate(startSize2.y(), endSize2.y(), t));
		}
		else if(setterType == static_cast<int>(SetterTypeTogether))
		{
			cylinderRepresentation1->setSize(interpolate(startSize1.x(), endSize1.x(), t),
interpolate(startSize1.y(), endSize1.y(), t));

			cylinderRepresentation2->setSize(interpolate(startSize2.x(), endSize2.x(), t),
interpolate(startSize2.y(), endSize2.y(), t));
		}
		else if(setterType == static_cast<int>(SetterTypeVector2d))
		{
			cylinder1Size.x() = interpolate(startSize1.x(), endSize1.x(), t);
			cylinder1Size.y() = interpolate(startSize1.y(), endSize1.y(), t);
			cylinderRepresentation1->setSize(cylinder1Size);

			cylinder2Size.x() = interpolate(startSize2.x(), endSize2.x(), t);
			cylinder2Size.y() = interpolate(startSize2.y(), endSize2.y(), t);
			cylinderRepresentation2->setSize(cylinder2Size);
		}
		setterType = (setterType + 1) % BoxSetterTypeCount;
		/// The total number of steps should complete in 1 second
		boost::this_thread::sleep(boost::posix_time::milliseconds(1000 / numSteps));
	}
}

};  // namespace Graphics

};  // namespace SurgSim
