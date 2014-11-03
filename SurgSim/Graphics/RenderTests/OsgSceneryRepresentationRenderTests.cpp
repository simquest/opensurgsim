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

#include "SurgSim/Graphics/OsgModel.h"
#include "SurgSim/Graphics/OsgSceneryRepresentation.h"
#include "SurgSim/Graphics/RenderTests/RenderTest.h"
#include "SurgSim/Math/Quaternion.h"
#include "SurgSim/Math/RigidTransform.h"
#include "SurgSim/Math/Vector.h"
#include "SurgSim/Testing/MathUtilities.h"

namespace SurgSim
{
namespace Graphics
{

using SurgSim::Math::Vector3d;
using SurgSim::Testing::interpolatePose;

struct OsgSceneryRepresentationRenderTests : public RenderTest
{
};

TEST_F(OsgSceneryRepresentationRenderTests, RenderTest)
{
	/// Initial position of object 1
	Vector3d startPosition1(-5.0, 0.0, -5.0);
	/// Final position of object 1
	Vector3d endPosition1(5.0, 0.0, -5.0);
	/// Initial angles (X, Y, Z) of object 1
	Vector3d startAngles1(0.0, 0.0, 0.0);
	/// Final angles (X, Y, Z) of object 1
	Vector3d endAngles1(-M_PI_4, -M_PI_4, -M_PI_4);

	/// Initial position of object 2
	Vector3d startPosition2(0.0, -5.0, -5.0);
	/// Final position of object 2
	Vector3d endPosition2(0.0, 5.0, -5.0);
	/// Initial angles (X, Y, Z) of object 2
	Vector3d startAngles2(-M_PI_2, -M_PI_2, -M_PI_2);
	/// Final angles (X, Y, Z) of object 2
	Vector3d endAngles2(M_PI, M_PI, M_PI);

	/// Number of times to step the objects' position from start to end.
	/// This number of steps will be done in 1 second.
	int numSteps = 100;

	auto sceneryObject1 = std::make_shared<OsgSceneryRepresentation>("Torus1");
	sceneryObject1->loadModel("OsgSceneryRepresentationTests/Torus.obj");
	sceneryObject1->setLocalPose(SurgSim::Math::makeRigidTransform(
									 SurgSim::Math::Quaterniond::Identity(),	startPosition1));
	viewElement->addComponent(sceneryObject1);

	auto sceneryObject2 = std::make_shared<OsgSceneryRepresentation>("Torus2");
	auto torus = std::make_shared<OsgModel>();
	torus->load("OsgSceneryRepresentationTests/Torus.osgb");
	sceneryObject2->setModel(torus);
	sceneryObject2->setLocalPose(SurgSim::Math::makeRigidTransform(
									 SurgSim::Math::Quaterniond::Identity(),	startPosition2));
	viewElement->addComponent(sceneryObject2);

	runtime->start();

	for (int i = 0; i < numSteps; ++i)
	{
		/// Calculate t in [0.0, 1.0]
		double t = static_cast<double>(i) / numSteps;

		/// Interpolate position
		sceneryObject1->setLocalPose(interpolatePose(startAngles1, endAngles1, startPosition1, endPosition1, t));
		sceneryObject2->setLocalPose(interpolatePose(startAngles2, endAngles2, startPosition2, endPosition2, t));

		/// The total number of steps should complete in 1 second
		boost::this_thread::sleep(boost::posix_time::milliseconds(1000 / numSteps));
	}

	std::cout << sceneryObject2->encode();
}


}  // namespace Graphics
}  // namespace SurgSim
