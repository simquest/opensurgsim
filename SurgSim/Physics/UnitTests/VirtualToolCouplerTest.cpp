//// This file is a part of the OpenSurgSim project.
//// Copyright 2013, SimQuest Solutions Inc.
////
//// Licensed under the Apache License, Version 2.0 (the "License");
//// you may not use this file except in compliance with the License.
//// You may obtain a copy of the License at
////
////     http://www.apache.org/licenses/LICENSE-2.0
////
//// Unless required by applicable law or agreed to in writing, software
//// distributed under the License is distributed on an "AS IS" BASIS,
//// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//// See the License for the specific language governing permissions and
//// limitations under the License.

#include <gtest/gtest.h>

#include "SurgSim/Devices/IdentityPoseDevice/IdentityPoseDevice.h"
#include "SurgSim/Input/InputComponent.h"
#include "SurgSim/Math/Matrix.h"
#include "SurgSim/Math/Quaternion.h"
#include "SurgSim/Math/RigidTransform.h"
#include "SurgSim/Math/Vector.h"
#include "SurgSim/Physics/RigidRepresentation.h"
#include "SurgSim/Physics/RigidRepresentationParameters.h"
#include "SurgSim/Physics/RigidRepresentationState.h"
#include "SurgSim/Physics/VirtualToolCoupler.h"

using SurgSim::Device::IdentityPoseDevice;
using SurgSim::Input::InputComponent;
using SurgSim::Math::Matrix33d;
using SurgSim::Math::Quaterniond;
using SurgSim::Math::RigidTransform3d;
using SurgSim::Math::SphereShape;
using SurgSim::Math::Vector3d;

namespace SurgSim
{
namespace Physics
{

const double epsilon = 1e-4;

struct VirtualToolCouplerTest : public ::testing::Test
{
	virtual void SetUp()
	{
		rigidBody = std::make_shared<RigidRepresentation>("Rigid Representation");

		RigidRepresentationParameters parameters;
		parameters.setDensity(700.0);
		parameters.setAngularDamping(0.0);
		parameters.setLinearDamping(0.0);
		parameters.setShapeUsedForMassInertia(std::make_shared<SphereShape>(0.1));
		rigidBody->setCurrentParameters(parameters);

		RigidRepresentationState state;
		state.setAngularVelocity(Vector3d::Zero());
		state.setLinearVelocity(Vector3d::Zero());
		state.setPose(RigidTransform3d::Identity());
		rigidBody->setInitialState(state);

		input = std::make_shared<InputComponent>("Input");
		input->setConnectedDeviceName("Device");
		device = std::make_shared<IdentityPoseDevice>("Device");
		input->connectDevice(device);

		virtualToolCoupler = std::make_shared<VirtualToolCoupler>("Virtual Tool Coupler", input, rigidBody);
		const double mass = parameters.getMass();
		virtualToolCoupler->setAngularDamping(mass * 1.0 );
		virtualToolCoupler->setAngularStiffness(mass * 200);
		virtualToolCoupler->setLinearDamping(mass * 50);
		virtualToolCoupler->setLinearStiffness(mass * 200);
	}

	virtual void TearDown()
	{
	}

	std::shared_ptr<RigidRepresentation> rigidBody;
	std::shared_ptr<VirtualToolCoupler> virtualToolCoupler;
	std::shared_ptr<InputComponent> input;
	std::shared_ptr<IdentityPoseDevice> device;

protected:
	void runSystem(const int numSteps, const double dt = 0.001)
	{
		for(int step=0; step<numSteps; step++)
		{
			virtualToolCoupler->update(dt);
			rigidBody->beforeUpdate(dt);
			rigidBody->update(dt);
			rigidBody->afterUpdate(dt);
		}
	}
};

TEST_F(VirtualToolCouplerTest, LinearDisplacement)
{
	RigidTransform3d initialPose = RigidTransform3d::Identity();
	initialPose.translation() = Vector3d(0.1, 0.0, 0.0);
	rigidBody->setInitialPose(initialPose);
	rigidBody->setIsGravityEnabled(false);

	EXPECT_TRUE(rigidBody->isActive());
	runSystem(2000);
	EXPECT_TRUE(rigidBody->isActive());

	RigidRepresentationState state = rigidBody->getCurrentState();
	EXPECT_TRUE(state.getLinearVelocity().isZero(epsilon));
	EXPECT_TRUE(state.getAngularVelocity().isZero(epsilon));
	EXPECT_TRUE(state.getPose().translation().isZero(epsilon));

	Eigen::AngleAxisd angleAxis = Eigen::AngleAxisd(state.getPose().linear());
	EXPECT_NEAR(0.0, angleAxis.angle(), epsilon);
}

TEST_F(VirtualToolCouplerTest, AngularDisplacement)
{
	RigidTransform3d initialPose = RigidTransform3d::Identity();
	initialPose.linear() = Matrix33d(Eigen::AngleAxisd(M_PI/4.0, Vector3d::UnitY()));
	rigidBody->setInitialPose(initialPose);
	rigidBody->setIsGravityEnabled(false);

	EXPECT_TRUE(rigidBody->isActive());
	runSystem(2000);
	EXPECT_TRUE(rigidBody->isActive());

	RigidRepresentationState state = rigidBody->getCurrentState();
	EXPECT_TRUE(state.getLinearVelocity().isZero(epsilon));
	EXPECT_TRUE(state.getAngularVelocity().isZero(epsilon));
	EXPECT_TRUE(state.getPose().translation().isZero(epsilon));

	Eigen::AngleAxisd angleAxis = Eigen::AngleAxisd(state.getPose().linear());
	EXPECT_NEAR(0.0, angleAxis.angle(), epsilon);
}

TEST_F(VirtualToolCouplerTest, WithGravity)
{
	RigidTransform3d initialPose = RigidTransform3d::Identity();
	initialPose.translation() = Vector3d(0.0, 0.0, 0.0);
	rigidBody->setInitialPose(initialPose);
	rigidBody->setIsGravityEnabled(true);

	const double stiffness = 1000;
	virtualToolCoupler->setLinearStiffness(stiffness);

	EXPECT_TRUE(rigidBody->isActive());
	runSystem(2000);
	EXPECT_TRUE(rigidBody->isActive());

	RigidRepresentationState state = rigidBody->getCurrentState();
	EXPECT_TRUE(state.getLinearVelocity().isZero(epsilon));
	EXPECT_TRUE(state.getAngularVelocity().isZero(epsilon));

	RigidRepresentationParameters parameters = rigidBody->getCurrentParameters();
	Vector3d g = Vector3d(0.0, -9.81, 0.0);
	Vector3d expectedPosition = parameters.getMass() * g / stiffness;
	EXPECT_TRUE(state.getPose().translation().isApprox(expectedPosition, epsilon));

	Eigen::AngleAxisd angleAxis = Eigen::AngleAxisd(state.getPose().linear());
	EXPECT_NEAR(0.0, angleAxis.angle(), epsilon);
}

}; // namespace Physics
}; // namespace SurgSim
