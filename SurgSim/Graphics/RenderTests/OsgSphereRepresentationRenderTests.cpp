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
/// Render Tests for the OsgSphereRepresentation class.

#include "SurgSim/Graphics/OsgManager.h"
#include "SurgSim/Graphics/OsgSphereRepresentation.h"
#include "SurgSim/Graphics/OsgViewElement.h"
#include "SurgSim/Graphics/OsgCamera.h"
#include "SurgSim/Framework/Scene.h"
#include "SurgSim/Framework/SceneElement.h"
#include "SurgSim/Framework/Runtime.h"
#include "SurgSim/Math/Quaternion.h"
#include "SurgSim/Math/Vector.h"
#include "SurgSim/Graphics/RenderTests/RenderTest.h"

#include <gtest/gtest.h>

#include <random>

using SurgSim::Framework::Runtime;
using SurgSim::Framework::Scene;
using SurgSim::Framework::SceneElement;
using SurgSim::Math::Quaterniond;
using SurgSim::Math::RigidTransform3d;
using SurgSim::Math::Vector3d;
using SurgSim::Math::makeRigidTransform;

namespace SurgSim
{

namespace Graphics
{

struct OsgSphereRepresentationRenderTests : public RenderTest
{
};

/// Pops up a window with two spheres that are translating and changing radius
TEST_F(OsgSphereRepresentationRenderTests, MovingSpheresTest)
{
	/// Initial sphere 1 position
	Vector3d startPosition1(-0.1, 0.0, -0.2);
	/// Final sphere 1 position
	Vector3d endPosition1(0.1, 0.0, -0.2);
	/// Initial sphere 1 radius;
	double startRadius1 = 0.001;
	/// Final sphere 1 radius;
	double endRadius1 = 0.01;
	/// Initial sphere 2 position
	Vector3d startPosition2(0.0, -0.1, -0.2);
	/// Final sphere 2 position
	Vector3d endPosition2(0.0, 0.1, -0.2);
	/// Initial sphere 2 radius;
	double startRadius2 = 0.01;
	/// Final sphere 2 radius;
	double endRadius2 = 0.05;

	/// Number of times to step the sphere position and radius from start to end.
	/// This number of steps will be done in 1 second.
	int numSteps = 100;

	/// Add the sphere representation to the view element, no need to make another scene element
	std::shared_ptr<SphereRepresentation> sphereRepresentation1 =
		std::make_shared<OsgSphereRepresentation>("sphere representation 1");
	viewElement->addComponent(sphereRepresentation1);
	std::shared_ptr<SphereRepresentation> sphereRepresentation2 =
		std::make_shared<OsgSphereRepresentation>("sphere representation 2");
	viewElement->addComponent(sphereRepresentation2);

	/// Run the thread
	runtime->start();

	boost::this_thread::sleep(boost::posix_time::milliseconds(1000));

	for (int i = 0; i < numSteps; ++i)
	{
		/// Calculate t in [0.0, 1.0]
		double t = static_cast<double>(i) / numSteps;
		/// Interpolate position and radius
		sphereRepresentation1->setLocalPose(makeRigidTransform(Quaterniond::Identity(), (1 - t) * startPosition1 +
											t * endPosition1));
		sphereRepresentation1->setRadius((1 - t) * startRadius1 + t * endRadius1);
		sphereRepresentation2->setLocalPose(makeRigidTransform(Quaterniond::Identity(), (1 - t) * startPosition2 +
											t * endPosition2));
		sphereRepresentation2->setRadius((1 - t) * startRadius2 + t * endRadius2);
		/// The total number of steps should complete in 1 second
		boost::this_thread::sleep(boost::posix_time::milliseconds(1000 / numSteps));
	}

	runtime->stop();
}

};  // namespace Graphics

};  // namespace SurgSim
