// This file is a part of the OpenSurgSim project.
// Copyright 2013, SimQuest Solutions Inc.
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

#include <memory>
#include <vector>

#include <gtest/gtest.h>

#include "SurgSim/DataStructures/Vertices.h"
#include "SurgSim/Framework/BasicSceneElement.h"
#include "SurgSim/Graphics/Camera.h"
#include "SurgSim/Graphics/OsgAxesRepresentation.h"
#include "SurgSim/Graphics/OsgLight.h"
#include "SurgSim/Graphics/OsgManager.h"
#include "SurgSim/Graphics/OsgMaterial.h"
#include "SurgSim/Graphics/OsgSkeletonRepresentation.h"
#include "SurgSim/Graphics/OsgShader.h"
#include "SurgSim/Graphics/OsgUniform.h"
#include "SurgSim/Graphics/OsgViewElement.h"
#include "SurgSim/Graphics/RenderTests/RenderTest.h"
#include "SurgSim/Math/Quaternion.h"
#include "SurgSim/Math/RigidTransform.h"
#include "SurgSim/Math/Vector.h"
#include "SurgSim/Testing/MathUtilities.h"
#include "SurgSim/Testing/TestCube.h"

#include <string>

using SurgSim::Math::Vector2d;
using SurgSim::Math::Vector3d;
using SurgSim::Math::Vector4d;
using SurgSim::Math::Vector4f;
using SurgSim::Math::Quaterniond;
using SurgSim::Math::RigidTransform3d;
using SurgSim::Math::makeRigidTransform;
using SurgSim::Math::makeRotationQuaternion;
using SurgSim::Testing::interpolate;

namespace SurgSim
{
namespace Graphics
{

struct OsgSkeletonRepresentationRenderTests : public RenderTest
{
};

TEST_F(OsgSkeletonRepresentationRenderTests, BasicTest)
{
	auto graphics = std::make_shared<SurgSim::Graphics::OsgSkeletonRepresentation>("Skeleton");
	graphics->loadModel("OsgSkeletonRepresentationRenderTests/rigged_cylinder.osgt");
	graphics->setSkinningShaderFileName("Shaders/skinning.vert");

	auto sceneElement = std::make_shared<SurgSim::Framework::BasicSceneElement>("Rigged Cylinder");
	sceneElement->addComponent(graphics);
	scene->addSceneElement(sceneElement);

	viewElement->setPose(SurgSim::Math::makeRigidTransform(
		Vector3d(-0.3, 0.3, -0.3),
		Vector3d(0.0, 0.0, 0.0),
		Vector3d(0.0, 0.0, 1.0)));

	runtime->start();
	boost::this_thread::sleep(boost::posix_time::milliseconds(2000));

	std::pair<RigidTransform3d, RigidTransform3d> rootTransform;
	rootTransform.first =
		makeRigidTransform(makeRotationQuaternion(0.0, Vector3d(1.0, 1.0, 1.0)), Vector3d::Zero());
	rootTransform.second =
		makeRigidTransform(makeRotationQuaternion(0.0, Vector3d(1.0, 1.0, 1.0)), Vector3d::Zero());

	std::pair<RigidTransform3d, RigidTransform3d> boneTransform;
	boneTransform.first =
		makeRigidTransform(makeRotationQuaternion(0.0, Vector3d(1.0, 0.0, 0.0)), Vector3d::Zero());
	boneTransform.second =
		makeRigidTransform(makeRotationQuaternion(0.0, Vector3d(1.0, 0.0, 0.0)), Vector3d(0.0, 4.0, 0.0));

	int numSteps = 1000;

	for (int i = 0; i < numSteps; ++i)
	{
		double t = static_cast<double>(i) / numSteps;

		RigidTransform3d pose = interpolate(rootTransform, t);
		sceneElement->setPose(pose);

		pose = interpolate(boneTransform, t / 3.0);
		graphics->setBonePose("Bone", RigidTransform3d::Identity());
		graphics->setBonePose("Bone_001", pose);
		graphics->setBonePose("Bone_002", pose);
		graphics->setBonePose("Bone_003", pose);

		/// The total number of steps should complete in 5 seconds
		boost::this_thread::sleep(boost::posix_time::milliseconds(5000 / numSteps));
	}

	rootTransform.first =
		makeRigidTransform(makeRotationQuaternion(0.0, Vector3d(1.0, 1.0, 1.0)), Vector3d::Zero());
	rootTransform.second =
		makeRigidTransform(makeRotationQuaternion(M_2_PI, Vector3d(1.0, 1.0, 1.0)), Vector3d::Zero());

	boneTransform.first =
		makeRigidTransform(makeRotationQuaternion(0.0, Vector3d(1.0, 0.0, 0.0)), Vector3d::Zero());
	boneTransform.second =
		makeRigidTransform(makeRotationQuaternion(M_PI, Vector3d(1.0, 0.0, 0.0)), Vector3d::Zero());

	for (int i = 0; i < numSteps; ++i)
	{
		double t = static_cast<double>(i) / numSteps;

		RigidTransform3d pose = interpolate(rootTransform, t);
		sceneElement->setPose(pose);

		pose = interpolate(boneTransform, t / 3.0);
		graphics->setBonePose("Bone", pose);
		graphics->setBonePose("Bone_001", pose);
		graphics->setBonePose("Bone_002", pose);
		graphics->setBonePose("Bone_003", pose);

		/// The total number of steps should complete in 5 seconds
		boost::this_thread::sleep(boost::posix_time::milliseconds(5000 / numSteps));
	}

	runtime->stop();
}

}; // namespace Graphics
}; // namespace SurgSim