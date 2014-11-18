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

/// \file
/// Tests for the DriveElementFromInputBehavior class.

#include <gtest/gtest.h>

#include "SurgSim/Blocks/DriveElementFromInputBehavior.h"
#include "SurgSim/Devices/IdentityPoseDevice/IdentityPoseDevice.h"
#include "SurgSim/Input/InputComponent.h"
#include "SurgSim/Input/OutputComponent.h"
#include "SurgSim/Framework/Assert.h"
#include "SurgSim/Framework/BasicSceneElement.h"
#include "SurgSim/Math/RigidTransform.h"
#include "SurgSim/Math/Vector.h"

using SurgSim::Device::IdentityPoseDevice;
using SurgSim::Input::InputComponent;
using SurgSim::Input::OutputComponent;
using SurgSim::Framework::BasicSceneElement;
using SurgSim::Math::makeRigidTranslation;
using SurgSim::Math::RigidTransform3d;
using SurgSim::Math::Vector3d;


namespace SurgSim
{
namespace Blocks
{

TEST(DriveElementFromInputBehaviorTest, Constructor)
{
	EXPECT_NO_THROW(DriveElementFromInputBehavior behavior("DriveElementFromInputBehavior"));
}

TEST(DriveElementFromInputBehaviorTest, SetGetSource)
{
	auto behavior = std::make_shared<DriveElementFromInputBehavior>("DriveElementFromInputBehavior");
	EXPECT_EQ(nullptr, behavior->getSource());

	auto invalidInputComponent = std::make_shared<OutputComponent>("InvalidInputComponent");
	EXPECT_THROW(behavior->setSource(invalidInputComponent), SurgSim::Framework::AssertionFailure);

	auto inputComponent = std::make_shared<InputComponent>("InputComponent");
	EXPECT_NO_THROW(behavior->setSource(inputComponent));
	EXPECT_EQ(inputComponent, behavior->getSource());
}

TEST(DriveElementFromInputBehaviorTest, SetGetName)
{
	auto behavior = std::make_shared<DriveElementFromInputBehavior>("DriveElementFromInputBehavior");
	EXPECT_EQ("pose", behavior->getPoseName());

	behavior->setPoseName("new_name");
	EXPECT_EQ("new_name", behavior->getPoseName());
}

TEST(DriveElementFromInputBehaviorTest, Update)
{
	auto behavior = std::make_shared<DriveElementFromInputBehavior>("DriveElementFromInputBehavior");
	auto element = std::make_shared<BasicSceneElement>("SceneElement");
	auto device = std::make_shared<IdentityPoseDevice>("IdentityPoseDevice");
	auto inputComponent = std::make_shared<InputComponent>("InputComponent");
	
	inputComponent->connectDevice(device);
	behavior->setSource(inputComponent);
	element->addComponent(behavior);
	element->addComponent(inputComponent);

	element->initialize();
	behavior->wakeUp();
	inputComponent->wakeUp();

	element->setPose(makeRigidTranslation(Vector3d(-1.0, -2.0, -3.0)));
	behavior->update(0.1);
	EXPECT_TRUE(element->getPose().isApprox(RigidTransform3d::Identity()));

	RigidTransform3d inputOffset = makeRigidTranslation(Vector3d(10.0, 20.0, 30.0));
	inputComponent->setLocalPose(inputOffset);
	behavior->update(0.1);
	EXPECT_TRUE(element->getPose().isApprox(inputOffset));
}

}; //namespace Blocks
}; //namespace SurgSim
