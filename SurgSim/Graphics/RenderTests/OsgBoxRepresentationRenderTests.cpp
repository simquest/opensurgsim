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

#include "SurgSim/Graphics/OsgManager.h"
#include "SurgSim/Graphics/OsgBoxRepresentation.h"
#include "SurgSim/Graphics/OsgViewElement.h"
#include "SurgSim/Framework/Scene.h"
#include "SurgSim/Framework/SceneElement.h"
#include "SurgSim/Framework/Runtime.h"
#include "SurgSim/Math/Quaternion.h"
#include "SurgSim/Math/Vector.h"

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


TEST(OsgBoxRepresentationRenderTests, MovingBoxesTest)
{
	/// Initial box 1 position
	Vector3d startPosition1(-0.1, 0.0, -0.2);
	/// Final box 1 position
	Vector3d endPosition1(0.1, 0.0, -0.2);
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
	/// Initial box 1 angleX;
	double startAngleX1 = 0.0;
	/// Final box 1 angleX;
	double endAngleX1 = - M_PI / 4.0;
	/// Initial box 1 angleY;
	double startAngleY1 = 0.0;
	/// Final box 1 angleY;
	double endAngleY1 = - M_PI / 4.0;
	/// Initial box 1 angleZ;
	double startAngleZ1 = 0.0;
	/// Final box 1 angleZ;
	double endAngleZ1 = - M_PI / 4.0;
	/// Initial box 2 position
	Vector3d startPosition2(0.0, -0.1, -0.2);
	/// Final box 2 position
	Vector3d endPosition2(0.0, 0.1, -0.2);
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
	double startAngleX2 = -M_PI / 2.0;;
	/// Final box 2 angleX;
	double endAngleX2 = M_PI;
	/// Initial box 2 angleY;
	double startAngleY2 = -M_PI / 2.0;;
	/// Final box 2 angleY;
	double endAngleY2 = M_PI;
	/// Initial box 2 angleZ;
	double startAngleZ2 = -M_PI / 2.0;;
	/// Final box 2 angleZ;
	double endAngleZ2 = M_PI;

	/// Number of times to step the box position and radius from start to end.
	/// This number of steps will be done in 1 second.
	int numSteps = 100;

	std::shared_ptr<Runtime> runtime = std::make_shared<Runtime>();
	std::shared_ptr<OsgManager> manager = std::make_shared<OsgManager>();

	runtime->addManager(manager);

	std::shared_ptr<Scene> scene = runtime->getScene();

	/// Add a graphics view element to the scene
	std::shared_ptr<OsgViewElement> viewElement = std::make_shared<OsgViewElement>("view element");
	scene->addSceneElement(viewElement);

	/// Add the two box representation to the view element so we don't need to make another concrete scene element
	/// \todo	DK-2013-june-03	Use the BasicSceneElement when it gets moved into Framework.
	std::shared_ptr<BoxRepresentation> boxRepresentation1 =
		std::make_shared<OsgBoxRepresentation>("box representation 1");
	viewElement->addComponent(boxRepresentation1);
	std::shared_ptr<BoxRepresentation> boxRepresentation2 =
		std::make_shared<OsgBoxRepresentation>("box representation 2");
	viewElement->addComponent(boxRepresentation2);

	/// Run the thread
	runtime->start();
	EXPECT_TRUE(manager->isInitialized());
	boost::this_thread::sleep(boost::posix_time::milliseconds(1000));

	enum BoxSetterType {BoxSetterTypeIndividual,
						BoxSetterTypeTogether,
						BoxSetterTypeVector3d,
						// Add more setter types above this line.
						BoxSetterTypeCount};
	int boxSetterType = 0;
	Vector3d box1Size, box2Size;

	for (int i = 0; i < numSteps; ++i)
	{
		/// Calculate t in [0.0, 1.0]
		double t = static_cast<double>(i) / numSteps;
		/// Interpolate position and radius
		boxRepresentation1->setPose(makeRigidTransform(makeRotationQuaternion((1.0 - t) * startAngleX1 + t * endAngleX1,
			Vector3d(1.0, 0.0, 0.0)) * makeRotationQuaternion((1.0 - t) * startAngleY1 + t * endAngleY1,
			Vector3d(0.0, 1.0, 0.0)) * makeRotationQuaternion((1.0 - t) * startAngleZ1 + t * endAngleZ1,
			Vector3d(0.0, 0.0, 1.0)), (1.0 - t) * startPosition1 + t * endPosition1));
		boxRepresentation2->setPose(makeRigidTransform(makeRotationQuaternion((1.0 - t) * startAngleX2 + t * endAngleX2,
			Vector3d(1.0, 0.0, 0.0)) * makeRotationQuaternion((1.0 - t) * startAngleY2 + t * endAngleY2,
			Vector3d(0.0, 1.0, 0.0)) * makeRotationQuaternion((1.0 - t) * startAngleZ2 + t * endAngleZ2,
			Vector3d(0.0, 0.0, 1.0)), (1.0 - t) * startPosition2 + t * endPosition2));
		if(boxSetterType == static_cast<int>(BoxSetterTypeIndividual))
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
			boxRepresentation1->setSizeXYZ(box1Size);

			box2Size.x() = (1 - t) * startSizeX2 + t * endSizeX2;
			box2Size.y() = (1 - t) * startSizeY2 + t * endSizeY2;
			box2Size.z() = (1 - t) * startSizeZ2 + t * endSizeZ2;
			boxRepresentation2->setSizeXYZ(box2Size);
		}
		boxSetterType = (boxSetterType + 1) % BoxSetterTypeCount;
		/// The total number of steps should complete in 1 second
		boost::this_thread::sleep(boost::posix_time::milliseconds(1000 / numSteps));
	}

	runtime->stop();
}

};  // namespace Graphics

};  // namespace SurgSim
