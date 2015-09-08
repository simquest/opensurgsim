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
/// Unit Tests for the OsgSkeletonRepresentation class.

#include <gtest/gtest.h>
#include <memory>

#include "SurgSim/Framework/FrameworkConvert.h"
#include "SurgSim/Framework/Runtime.h"
#include "SurgSim/Graphics/OsgManager.h"
#include "SurgSim/Graphics/OsgSkeletonRepresentation.h"
#include "SurgSim/Graphics/OsgViewElement.h"
#include "SurgSim/Graphics/OsgModel.h"

using SurgSim::Graphics::OsgSkeletonRepresentation;
using SurgSim::Graphics::OsgViewElement;
using SurgSim::Graphics::SkeletonRepresentation;
using SurgSim::Math::makeRigidTransform;
using SurgSim::Math::makeRotationQuaternion;
using SurgSim::Math::RigidTransform3d;
using SurgSim::Math::Vector3d;


namespace SurgSim
{
namespace Graphics
{

class OsgSkeletonRepresentationTest: public ::testing::Test
{
public:
	void SetUp() override
	{
		runtime = std::make_shared<SurgSim::Framework::Runtime>("config.txt");
	}

	void TearDown() override
	{
	}

	std::shared_ptr<SurgSim::Framework::Runtime> runtime;
};

TEST_F(OsgSkeletonRepresentationTest, CanConstruct)
{
	EXPECT_NO_THROW({OsgSkeletonRepresentation skeleton("test");});
	EXPECT_NO_THROW({auto skeleton = std::make_shared<OsgSkeletonRepresentation>("test");});
}

TEST_F(OsgSkeletonRepresentationTest, FileNameTest)
{
	auto skeleton = std::make_shared<OsgSkeletonRepresentation>("test");
	skeleton->loadModel("OsgSkeletonRepresentationTests/rigged_cylinder.osgt");
	EXPECT_EQ("OsgSkeletonRepresentationTests/rigged_cylinder.osgt", skeleton->getModel()->getFileName());
}

TEST_F(OsgSkeletonRepresentationTest, SkinningShaderFileNameTest)
{
	auto skeleton = std::make_shared<OsgSkeletonRepresentation>("test");
	skeleton->setSkinningShaderFileName("Shaders/skinning.vert");
	EXPECT_EQ("Shaders/skinning.vert", skeleton->getSkinningShaderFileName());
}

TEST_F(OsgSkeletonRepresentationTest, InitTest)
{
	{
		auto skeleton = std::make_shared<OsgSkeletonRepresentation>("test");
		skeleton->loadModel("OsgSkeletonRepresentationTests/rigged_cylinder.osgt");
		skeleton->setSkinningShaderFileName("Shaders/skinning.vert");
		EXPECT_NO_THROW(skeleton->initialize(runtime));
		EXPECT_TRUE(skeleton->isInitialized()) << "Should initialize with both model and shader.";
	}
	{
		auto skeleton = std::make_shared<OsgSkeletonRepresentation>("test");
		skeleton->setSkinningShaderFileName("Shaders/skinning.vert");
		EXPECT_NO_THROW(skeleton->initialize(runtime));
		EXPECT_FALSE(skeleton->isInitialized()) << "Should not be initialized, no model set";
	}
	{
		auto skeleton = std::make_shared<OsgSkeletonRepresentation>("test");
		skeleton->loadModel("OsgSkeletonRepresentationTests/rigged_cylinder.osgt");
		EXPECT_NO_THROW(skeleton->initialize(runtime));
		EXPECT_FALSE(skeleton->isInitialized()) << "Should not be initialized, no shader set";
	}
}

TEST_F(OsgSkeletonRepresentationTest, PosesTest)
{
	RigidTransform3d pose = makeRigidTransform(makeRotationQuaternion(2.143, Vector3d::UnitZ().eval()),
							Vector3d(2.3, 4.5, 6.7));
	{
		auto skeleton = std::make_shared<OsgSkeletonRepresentation>("test");
		skeleton->loadModel("OsgSkeletonRepresentationTests/rigged_cylinder.osgt");
		skeleton->setSkinningShaderFileName("Shaders/skinning.vert");

		EXPECT_NO_THROW(skeleton->initialize(runtime));
		RigidTransform3d actualPose;
		EXPECT_NO_THROW({actualPose = skeleton->getBonePose("Bone");});
		EXPECT_TRUE(RigidTransform3d::Identity().isApprox(actualPose));
		EXPECT_THROW(skeleton->getBonePose("BadBoneName"), SurgSim::Framework::AssertionFailure);
	}
	{
		auto skeleton = std::make_shared<OsgSkeletonRepresentation>("test");
		skeleton->loadModel("OsgSkeletonRepresentationTests/rigged_cylinder.osgt");
		skeleton->setSkinningShaderFileName("Shaders/skinning.vert");

		EXPECT_NO_THROW(skeleton->setBonePose("Bone", pose));

		RigidTransform3d actualPose;
		EXPECT_NO_THROW({actualPose = skeleton->getBonePose("Bone");});
		EXPECT_TRUE(pose.isApprox(actualPose));

		EXPECT_NO_THROW(skeleton->initialize(runtime));
		EXPECT_NO_THROW({actualPose = skeleton->getBonePose("Bone");});
		EXPECT_TRUE(pose.isApprox(actualPose));
	}
	{
		auto skeleton = std::make_shared<OsgSkeletonRepresentation>("test");
		skeleton->loadModel("OsgSkeletonRepresentationTests/rigged_cylinder.osgt");
		skeleton->setSkinningShaderFileName("Shaders/skinning.vert");

		EXPECT_NO_THROW(skeleton->setBonePose("BadBoneName", pose));

		RigidTransform3d actualPose;
		EXPECT_NO_THROW({actualPose = skeleton->getBonePose("BadBoneName");});
		EXPECT_TRUE(pose.isApprox(actualPose));

		EXPECT_THROW(skeleton->initialize(runtime), SurgSim::Framework::AssertionFailure);
	}
}

TEST_F(OsgSkeletonRepresentationTest, NeutralPosesTest)
{
	RigidTransform3d pose = makeRigidTransform(makeRotationQuaternion(2.143, Vector3d::UnitZ().eval()),
							Vector3d(2.3, 4.5, 6.7));
	{
		auto skeleton = std::make_shared<OsgSkeletonRepresentation>("test");
		skeleton->loadModel("OsgSkeletonRepresentationTests/rigged_cylinder.osgt");
		skeleton->setSkinningShaderFileName("Shaders/skinning.vert");

		EXPECT_NO_THROW(skeleton->initialize(runtime));
		RigidTransform3d actualPose;
		EXPECT_NO_THROW({actualPose = skeleton->getNeutralBonePose("Bone");});
		EXPECT_TRUE(RigidTransform3d::Identity().isApprox(actualPose));
		EXPECT_THROW(skeleton->getNeutralBonePose("BadBoneName"), SurgSim::Framework::AssertionFailure);
	}
	{
		auto skeleton = std::make_shared<OsgSkeletonRepresentation>("test");
		skeleton->loadModel("OsgSkeletonRepresentationTests/rigged_cylinder.osgt");
		skeleton->setSkinningShaderFileName("Shaders/skinning.vert");

		EXPECT_NO_THROW(skeleton->setNeutralBonePose("Bone", pose));

		RigidTransform3d actualPose;
		EXPECT_NO_THROW({actualPose = skeleton->getNeutralBonePose("Bone");});
		EXPECT_TRUE(pose.isApprox(actualPose));

		EXPECT_NO_THROW(skeleton->initialize(runtime));
		EXPECT_NO_THROW({actualPose = skeleton->getNeutralBonePose("Bone");});
		EXPECT_TRUE(pose.isApprox(actualPose));
	}
	{
		auto skeleton = std::make_shared<OsgSkeletonRepresentation>("test");
		skeleton->loadModel("OsgSkeletonRepresentationTests/rigged_cylinder.osgt");
		skeleton->setSkinningShaderFileName("Shaders/skinning.vert");

		EXPECT_NO_THROW(skeleton->setNeutralBonePose("BadBoneName", pose));

		RigidTransform3d actualPose;
		EXPECT_NO_THROW({actualPose = skeleton->getNeutralBonePose("BadBoneName");});
		EXPECT_TRUE(pose.isApprox(actualPose));

		EXPECT_THROW(skeleton->initialize(runtime), SurgSim::Framework::AssertionFailure);
	}
}

TEST_F(OsgSkeletonRepresentationTest, AccessibleTest)
{
	std::shared_ptr<SurgSim::Framework::Component> component;
	ASSERT_NO_THROW(component = SurgSim::Framework::Component::getFactory().create(
									"SurgSim::Graphics::OsgSkeletonRepresentation", "skeleton"));

	std::string fileName("OsgSkeletonRepresentationTests/rigged_cylinder.osgt");
	component->setValue("ModelFileName", fileName);
	std::string skinningShaderFileName("Shaders/skinning.vert");
	component->setValue("SkinningShaderFileName", skinningShaderFileName);

	auto asset = component->getValue<std::shared_ptr<Model>>("Model");
	EXPECT_EQ(fileName, asset->getFileName());
	auto shaderName = component->getValue<std::string>("SkinningShaderFileName");
	EXPECT_EQ(skinningShaderFileName, shaderName);
}

TEST_F(OsgSkeletonRepresentationTest, SerializationTests)
{
	auto skeleton = std::make_shared<OsgSkeletonRepresentation>("test");

	std::string fileName("OsgSkeletonRepresentationTests/rigged_cylinder.osgt");
	skeleton->loadModel(fileName);
	std::string skinningShaderFileName("Shaders/skinning.vert");
	skeleton->setSkinningShaderFileName(skinningShaderFileName);
	RigidTransform3d pose = makeRigidTransform(makeRotationQuaternion(2.143,
							Vector3d(2.463, 6.346, 7.135).normalized()),
							Vector3d(2.3, 4.5, 6.7));
	skeleton->setNeutralBonePose("Bone", pose);

	YAML::Node node;
	node = skeleton->encode();
	ASSERT_NO_THROW(node = skeleton->encode());
	EXPECT_TRUE(node.IsMap());

	std::shared_ptr<OsgSkeletonRepresentation> result = std::make_shared<OsgSkeletonRepresentation>("Skeleton");
	ASSERT_NO_THROW(result->decode(node));
	EXPECT_EQ("SurgSim::Graphics::OsgSkeletonRepresentation", result->getClassName());
	EXPECT_EQ(fileName, result->getModel()->getFileName());
	EXPECT_EQ(skinningShaderFileName, result->getSkinningShaderFileName());
	EXPECT_TRUE(pose.isApprox(result->getNeutralBonePose("Bone")));
}

};
};
