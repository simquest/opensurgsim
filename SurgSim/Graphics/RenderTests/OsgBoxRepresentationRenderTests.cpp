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
#include "SurgSim/Graphics/OsgBoxRepresentation.h"

#include "SurgSim/Framework/Runtime.h"
#include "SurgSim/Framework/Scene.h"
#include "SurgSim/Framework/SceneElement.h"
#include "SurgSim/Graphics/OsgCamera.h"
#include "SurgSim/Graphics/OsgManager.h"
#include "SurgSim/Graphics/OsgView.h"
#include "SurgSim/Graphics/OsgViewElement.h"
#include "SurgSim/Math/Quaternion.h"
#include "SurgSim/Math/Vector.h"
#include "SurgSim/Graphics/RenderTests/RenderTest.h"
#include "SurgSim/Testing/MathUtilities.h"

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
using SurgSim::Testing::interpolatePose;

namespace SurgSim
{

namespace Graphics
{

struct OsgBoxRepresentationRenderTests : public RenderTest
{

};

TEST_F(OsgBoxRepresentationRenderTests, MovingBoxesTest)
{
	/// Initial and final position (X, Y, Z) of box 1
	Vector3d startPosition1(-0.1, 0.0, -0.2), finalPosition1(0.1, 0.0, -0.2);
	/// Initial angles (X, Y, Z) and final of the cylinder 1
	Vector3d startAngles1(0.0, 0.0, 0.0), finalAngles1(-M_PI_4, -M_PI_4, -M_PI_4);
	/// Initial box 1 sizeX;
	double startSizeX1 = 0.001;
	/// Final box 1 sizeX;
	double endSizeX1 = 0.01;
	/// Initial box 1 sizeY;
	double startSizeY1 = 0.011;
	/// Final box 1 sizeY;
	double endSizeY1 = 0.02;
	/// Initial box 1 sizeZ;
	double startSizeZ1 = 0.021;
	/// Final box 1 sizeZ;
	double endSizeZ1 = 0.03;

	/// Initial and final position (X, Y, Z) box 2
	Vector3d startPosition2(0.0, -0.1, -0.2), finalPosition2(0.0, 0.1, -0.2);
	/// Initial and final angles (X, Y, Z) of the box 2
	Vector3d startAngles2(-M_PI_2, -M_PI_2, -M_PI_2), finalAngles2(M_PI, M_PI, M_PI);
	/// Initial box 2 sizeX;
	double startSizeX2 = 0.001;
	/// Final box 2 sizeX;
	double endSizeX2 = 0.01;
	/// Initial box 2 sizeX;
	double startSizeY2 = 0.011;
	/// Final box 2 sizeX;
	double endSizeY2 = 0.02;
	/// Initial box 2 sizeX;
	double startSizeZ2 = 0.021;
	/// Final box 2 sizeX;
	double endSizeZ2 = 0.03;
	/// Initial box 2 angleX;




	/// Number of times to step the box position and radius from start to end.
	/// This number of steps will be done in 1 second.
	int numSteps = 100;

	std::shared_ptr<Scene> scene = runtime->getScene();

	auto boxRepresentation1 = std::make_shared<OsgBoxRepresentation>("box representation 1");
	viewElement->addComponent(boxRepresentation1);

	auto boxRepresentation2 = std::make_shared<OsgBoxRepresentation>("box representation 2");
	viewElement->addComponent(boxRepresentation2);

	/// Run the thread
	runtime->start();
	EXPECT_TRUE(graphicsManager->isInitialized());
	boost::this_thread::sleep(boost::posix_time::milliseconds(1000));

	enum BoxSetterType {BoxSetterTypeIndividual,
						BoxSetterTypeTogether,
						BoxSetterTypeVector3d,
						// Add more setter types above this line.
						BoxSetterTypeCount
					   };
	int boxSetterType = 0;
	Vector3d box1Size, box2Size;

	for (int i = 0; i < numSteps; ++i)
	{
		/// Calculate t in [0.0, 1.0]
		double t = static_cast<double>(i) / numSteps;
		/// Interpolate position and orientation
		boxRepresentation1->setPose(
			interpolatePose(startAngles1, finalAngles1, startPosition1, finalPosition1, t));
		boxRepresentation2->setPose(
			interpolatePose(startAngles2, finalAngles2, startPosition1, finalPosition2, t));

		if (boxSetterType == static_cast<int>(BoxSetterTypeIndividual))
		{
			boxRepresentation1->setSizeX((1 - t) * startSizeX1 + t * endSizeX1);
			boxRepresentation1->setSizeY((1 - t) * startSizeY1 + t * endSizeY1);
			boxRepresentation1->setSizeZ((1 - t) * startSizeZ1 + t * endSizeZ1);

			boxRepresentation2->setSizeX((1 - t) * startSizeX2 + t * endSizeX2);
			boxRepresentation2->setSizeY((1 - t) * startSizeY2 + t * endSizeY2);
			boxRepresentation2->setSizeZ((1 - t) * startSizeZ2 + t * endSizeZ2);
		}
		else if (boxSetterType == static_cast<int>(BoxSetterTypeTogether))
		{
			boxRepresentation1->setSizeXYZ((1 - t) * startSizeX1 + t * endSizeX1,
										   (1 - t) * startSizeY1 + t * endSizeY1,
										   (1 - t) * startSizeZ1 + t * endSizeZ1);

			boxRepresentation2->setSizeXYZ((1 - t) * startSizeX2 + t * endSizeX2,
										   (1 - t) * startSizeY2 + t * endSizeY2,
										   (1 - t) * startSizeZ2 + t * endSizeZ2);
		}
		else if (boxSetterType == static_cast<int>(BoxSetterTypeVector3d))
		{
			box1Size.x() = (1 - t) * startSizeX1 + t * endSizeX1;
			box1Size.y() = (1 - t) * startSizeY1 + t * endSizeY1;
			box1Size.z() = (1 - t) * startSizeZ1 + t * endSizeZ1;
			boxRepresentation1->setSize(box1Size);

			box2Size.x() = (1 - t) * startSizeX2 + t * endSizeX2;
			box2Size.y() = (1 - t) * startSizeY2 + t * endSizeY2;
			box2Size.z() = (1 - t) * startSizeZ2 + t * endSizeZ2;
			boxRepresentation2->setSize(box2Size);
		}
		boxSetterType = (boxSetterType + 1) % BoxSetterTypeCount;
		/// The total number of steps should complete in 1 second
		boost::this_thread::sleep(boost::posix_time::milliseconds(1000 / numSteps));
	}

	runtime->stop();
}

};  // namespace Graphics

};  // namespace SurgSim
