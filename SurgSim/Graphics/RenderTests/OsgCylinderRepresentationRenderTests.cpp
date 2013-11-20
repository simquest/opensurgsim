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

#include <SurgSim/Graphics/OsgCylinderRepresentation.h>
#include <SurgSim/Graphics/RenderTests/RenderTest.h>
#include <SurgSim/Math/Vector.h>
#include <SurgSim/Testing/MathUtilities.h>

using SurgSim::Math::Vector2d;
using SurgSim::Math::Vector3d;
using SurgSim::Testing::interpolate;
using SurgSim::Testing::interpolatePose;

namespace SurgSim
{
namespace Graphics
{

struct OsgCylinderRepresentationRenderTests : public RenderTest
{
};

TEST_F(OsgCylinderRepresentationRenderTests, MovingCylinderTest)
{
	/// Add the two cylinder representation to the view element
	std::shared_ptr<CylinderRepresentation> cylinderRepresentation1 =
		std::make_shared<OsgCylinderRepresentation>("cylinder representation 1");
	viewElement->addComponent(cylinderRepresentation1);
	std::shared_ptr<CylinderRepresentation> cylinderRepresentation2 =
		std::make_shared<OsgCylinderRepresentation>("cylinder representation 2");
	viewElement->addComponent(cylinderRepresentation2);

	/// Run the thread
	runtime->start();
	EXPECT_TRUE(graphicsManager->isInitialized());
	boost::this_thread::sleep(boost::posix_time::milliseconds(1000));

	enum SetterType {SetterTypeIndividual,
					 SetterTypeTogether,
					 SetterTypeVector2d,
					 // Add more setter types above this line.
					 BoxSetterTypeCount};
	int setterType = 0;

	Vector2d cylinder1Size, cylinder2Size;

	/// Initial and final position (X, Y, Z) of cylinder 1
	Vector3d startPosition1(-0.1, 0.0, -0.2), finalPosition1(0.1, 0.0, -0.2);
	/// Initial size (radius, height) and final size of cylinder 1
	Vector2d startSize1(0.001, 0.011), finalSize1(0.01, 0.02);
	/// Initial angles (X, Y, Z) and final of the cylinder 1
	Vector3d startAngles1(0.0, 0.0, 0.0), finalAngles1(-M_PI_4, -M_PI_4, -M_PI_4);

	/// Initial and final position (X, Y, Z) cylinder 2
	Vector3d startPosition2(0.0, -0.1, -0.2), finalPosition2(0.0, 0.1, -0.2);
	/// Initial and final size (radius, height) of cylinder 2
	Vector2d startSize2(0.001, 0.011), finalSize2(0.01, 0.02);
	/// Initial and final angles (X, Y, Z) of the cylinder 2
	Vector3d startAngles2(-M_PI_2, -M_PI_2, -M_PI_2), finalAngles2(M_PI, M_PI, M_PI);

	/// Number of times to step the cylinder position and radius from start to end.
	/// This number of steps will be done in 1 second.
	int numSteps = 100;
	for (int i = 0; i < numSteps; ++i)
	{
		/// Calculate t in [0.0, 1.0]
		double t = static_cast<double>(i) / numSteps;
		/// Interpolate position and radius
		cylinderRepresentation1->setPose(
			interpolatePose(startAngles1, finalAngles1, startPosition1, finalPosition1, t));
		cylinderRepresentation2->setPose(
			interpolatePose(startAngles2, finalAngles2, startPosition1, finalPosition2, t));

		if(setterType == static_cast<int>(SetterTypeIndividual))
		{
			cylinderRepresentation1->setRadius(interpolate(startSize1.x(), finalSize1.x(), t));
			cylinderRepresentation1->setHeight(interpolate(startSize1.y(), finalSize1.y(), t));

			cylinderRepresentation2->setRadius(interpolate(startSize2.x(), finalSize2.x(), t));
			cylinderRepresentation2->setHeight(interpolate(startSize2.y(), finalSize2.y(), t));
		}
		else if(setterType == static_cast<int>(SetterTypeTogether))
		{
			cylinderRepresentation1->setSize(interpolate(startSize1.x(), finalSize1.x(), t),
interpolate(startSize1.y(), finalSize1.y(), t));

			cylinderRepresentation2->setSize(interpolate(startSize2.x(), finalSize2.x(), t),
interpolate(startSize2.y(), finalSize2.y(), t));
		}
		else if(setterType == static_cast<int>(SetterTypeVector2d))
		{
			cylinder1Size.x() = interpolate(startSize1.x(), finalSize1.x(), t);
			cylinder1Size.y() = interpolate(startSize1.y(), finalSize1.y(), t);
			cylinderRepresentation1->setSize(cylinder1Size);

			cylinder2Size.x() = interpolate(startSize2.x(), finalSize2.x(), t);
			cylinder2Size.y() = interpolate(startSize2.y(), finalSize2.y(), t);
			cylinderRepresentation2->setSize(cylinder2Size);
		}
		setterType = (setterType + 1) % BoxSetterTypeCount;
		/// The total number of steps should complete in 1 second
		boost::this_thread::sleep(boost::posix_time::milliseconds(1000 / numSteps));
	}
}

};  // namespace Graphics

};  // namespace SurgSim
