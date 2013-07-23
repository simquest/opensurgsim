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

namespace SurgSim
{

namespace Graphics
{


TEST(OsgCylinderRepresentationRenderTests, MovingCapsuleTest)
{
	/// Initial cylinder 1 position
	Vector3d startPosition1(-0.1, 0.0, -0.2);
	/// Final cylinder 1 position
	Vector3d endPosition1(0.1, 0.0, -0.2);
	/// Initial cylinder 1 radius;
	double startRadius1 = 0.001;
	/// Final cylinder 1 radius;
	double endRadius1 = 0.01;
	/// Initial cylinder 1 height;
	double startHeight1 = 0.011;
	/// Final cylinder 1 height;
	double endHeight1 = 0.02;
	/// Initial cylinder 1 angleX;
	double startAngleX1 = 0.0;
	/// Final cylinder 1 angleX;
	double endAngleX1 = - M_PI / 4.0;
	/// Initial cylinder 1 angleY;
	double startAngleY1 = 0.0;
	/// Final cylinder 1 angleY;
	double endAngleY1 = - M_PI / 4.0;
	/// Initial cylinder 1 angleZ;
	double startAngleZ1 = 0.0;
	/// Final cylinder 1 angleZ;
	double endAngleZ1 = - M_PI / 4.0;
	/// Initial cylinder 2 position
	Vector3d startPosition2(0.0, -0.1, -0.2);
	/// Final cylinder 2 position
	Vector3d endPosition2(0.0, 0.1, -0.2);
	/// Initial cylinder 2 radius;
	double startRadius2 = 0.001;
	/// Final cylinder 2 radius;
	double endRadius2 = 0.01;
	/// Initial cylinder 2 height;
	double startHeight2 = 0.011;
	/// Final cylinder 2 height;
	double endHeight2 = 0.02;
	/// Initial cylinder 2 angleX;
	double startAngleX2 = -M_PI / 2.0;;
	/// Final cylinder 2 angleX;
	double endAngleX2 = M_PI;
	/// Initial cylinder 2 angleY;
	double startAngleY2 = -M_PI / 2.0;;
	/// Final cylinder 2 angleY;
	double endAngleY2 = M_PI;
	/// Initial cylinder 2 angleZ;
	double startAngleZ2 = -M_PI / 2.0;;
	/// Final cylinder 2 angleZ;
	double endAngleZ2 = M_PI;

	/// Number of times to step the cylinder position and radius from start to end.
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

	for (int i = 0; i < numSteps; ++i)
	{
		/// Calculate t in [0.0, 1.0]
		double t = static_cast<double>(i) / numSteps;
		/// Interpolate position and radius
		cylinderRepresentation1->setPose(makeRigidTransform(
			makeRotationQuaternion((1.0 - t) * startAngleX1 + t * endAngleX1,
			Vector3d(1.0, 0.0, 0.0)) * makeRotationQuaternion((1.0 - t) * startAngleY1 + t * endAngleY1,
			Vector3d(0.0, 1.0, 0.0)) * makeRotationQuaternion((1.0 - t) * startAngleZ1 + t * endAngleZ1,
			Vector3d(0.0, 0.0, 1.0)), (1.0 - t) * startPosition1 + t * endPosition1));
		cylinderRepresentation2->setPose(makeRigidTransform(
			makeRotationQuaternion((1.0 - t) * startAngleX2 + t * endAngleX2,
			Vector3d(1.0, 0.0, 0.0)) * makeRotationQuaternion((1.0 - t) * startAngleY2 + t * endAngleY2,
			Vector3d(0.0, 1.0, 0.0)) * makeRotationQuaternion((1.0 - t) * startAngleZ2 + t * endAngleZ2,
			Vector3d(0.0, 0.0, 1.0)), (1.0 - t) * startPosition2 + t * endPosition2));
		if(setterType == static_cast<int>(SetterTypeIndividual))
		{
			cylinderRepresentation1->setRadius((1 - t) * startRadius1 + t * endRadius1);
			cylinderRepresentation1->setHeight((1 - t) * startHeight1 + t * endHeight1);

			cylinderRepresentation2->setRadius((1 - t) * startRadius2 + t * endRadius2);
			cylinderRepresentation2->setHeight((1 - t) * startHeight2 + t * endHeight2);
		}
		else if(setterType == static_cast<int>(SetterTypeTogether))
		{
			cylinderRepresentation1->setSize((1 - t) * startRadius1 + t * endRadius1,
(1 - t) * startHeight1 + t * endHeight1);

			cylinderRepresentation2->setSize((1 - t) * startRadius2 + t * endRadius2,
(1 - t) * startHeight2 + t * endHeight2);
		}
		else if(setterType == static_cast<int>(SetterTypeVector2d))
		{
			cylinder1Size.x() = (1 - t) * startRadius1 + t * endRadius1;
			cylinder1Size.y() = (1 - t) * startHeight1 + t * endHeight1;
			cylinderRepresentation1->setSize(cylinder1Size);

			cylinder2Size.x() = (1 - t) * startRadius2 + t * endRadius2;
			cylinder2Size.y() = (1 - t) * startHeight2 + t * endHeight2;
			cylinderRepresentation2->setSize(cylinder2Size);
		}
		setterType = (setterType + 1) % BoxSetterTypeCount;
		/// The total number of steps should complete in 1 second
		boost::this_thread::sleep(boost::posix_time::milliseconds(1000 / numSteps));
	}

	runtime->stop();
}

};  // namespace Graphics

};  // namespace SurgSim
