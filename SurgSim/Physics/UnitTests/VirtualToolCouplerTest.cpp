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

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <math.h>

#include "SurgSim/Collision/CollisionPair.h"
#include "SurgSim/DataStructures/Location.h"
#include "SurgSim/Devices/IdentityPoseDevice/IdentityPoseDevice.h"
#include "SurgSim/Input/InputComponent.h"
#include "SurgSim/Input/OutputComponent.h"
#include "SurgSim/Framework/FrameworkConvert.h"
#include "SurgSim/Framework/Runtime.h"
#include "SurgSim/Math/Matrix.h"
#include "SurgSim/Math/Quaternion.h"
#include "SurgSim/Math/RigidTransform.h"
#include "SurgSim/Math/SphereShape.h"
#include "SurgSim/Math/Vector.h"
#include "SurgSim/Physics/RigidCollisionRepresentation.h"
#include "SurgSim/Physics/RigidRepresentation.h"
#include "SurgSim/Physics/RigidState.h"
#include "SurgSim/Physics/UnitTests/MockObjects.h"
#include "SurgSim/Physics/VirtualToolCoupler.h"
#include "SurgSim/Testing/MockInputComponent.h"

using SurgSim::Device::IdentityPoseDevice;
using SurgSim::Input::InputComponent;
using SurgSim::Framework::Runtime;
using SurgSim::Math::makeRigidTranslation;
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
		representationName = "Rigid Representation";
		inputDeviceName = "Device";

		rigidBody = std::make_shared<RigidRepresentation>(representationName);
		rigidBody->setDensity(700.0);
		rigidBody->setAngularDamping(0.0);
		rigidBody->setLinearDamping(0.0);
		rigidBody->setShape(std::make_shared<SphereShape>(0.1));

		RigidState state;
		state.setAngularVelocity(Vector3d::Zero());
		state.setLinearVelocity(Vector3d::Zero());
		state.setPose(RigidTransform3d::Identity());
		rigidBody->setInitialState(state);

		input = std::make_shared<InputComponent>("Input");
		input->setDeviceName(inputDeviceName);
		device = std::make_shared<IdentityPoseDevice>(inputDeviceName);
		input->connectDevice(device);

		virtualToolCoupler = std::make_shared<MockVirtualToolCoupler>();
		virtualToolCoupler->setInput(input);
		virtualToolCoupler->setRepresentation(rigidBody);
	}

	virtual void TearDown()
	{
	}

	std::shared_ptr<RigidRepresentation> rigidBody;
	std::shared_ptr<MockVirtualToolCoupler> virtualToolCoupler;
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

	void checkLinearIsCriticallyDamped()
	{
		RigidTransform3d initialPose = RigidTransform3d::Identity();
		initialPose.translation() = Vector3d(0.1, 0.0, 0.0);
		rigidBody->setLocalPose(initialPose);
		rigidBody->setIsGravityEnabled(false);

		std::shared_ptr<Runtime> runtime = std::make_shared<Runtime>();
		virtualToolCoupler->initialize(runtime);
		rigidBody->initialize(runtime);
		virtualToolCoupler->wakeUp();
		rigidBody->wakeUp();

		// Critically damped mass-damper systems settle to within 5% of their
		// equilibrium when:
		//      t = 4.744 * naturalFrequency;
		// we will run the system to this point, and then check that the
		// final position is 5.1% of its initial position
		double mass = rigidBody->getMass();
		double stiffness = virtualToolCoupler->getLinearStiffness();
		double naturalFrequency = sqrt(stiffness / mass);
		double expectedSettlingTime = 4.744 / naturalFrequency;
		const int NUM_STEPS = 1000;
		double dt = expectedSettlingTime / NUM_STEPS;
		Vector3d previousPosition = initialPose.translation();
		Vector3d currentPosition;
		for(int step=0; step<NUM_STEPS; step++)
		{
			virtualToolCoupler->update(dt);
			rigidBody->beforeUpdate(dt);
			rigidBody->update(dt);
			rigidBody->afterUpdate(dt);
			currentPosition = rigidBody->getCurrentState().getPose().translation();
			// Check for exponential behavior. The position should monotonically decrease and not oscillate.
			ASSERT_TRUE((previousPosition.array() >= currentPosition.array()).all());
			previousPosition = currentPosition;
		}
		Vector3d expectedPosition = 0.051 * initialPose.translation();
		EXPECT_TRUE((expectedPosition.array() >= currentPosition.array()).all());
	}

	void checkAngularIsCriticallyDamped()
	{
		double initialAngle = M_PI_4;
		RigidTransform3d initialPose = RigidTransform3d::Identity();
		initialPose.linear() = Matrix33d(Eigen::AngleAxisd(initialAngle, Vector3d::UnitY()));
		rigidBody->setLocalPose(initialPose);
		rigidBody->setIsGravityEnabled(false);

		std::shared_ptr<Runtime> runtime = std::make_shared<Runtime>();
		virtualToolCoupler->initialize(runtime);
		rigidBody->initialize(runtime);
		virtualToolCoupler->wakeUp();
		rigidBody->wakeUp();

		// Critically damped mass-damper systems settle to within 5% of their
		// equilibrium when:
		//      t = 4.744 * naturalFrequency;
		// we will run the system to this point, and then check that the
		// final position is 5.1% of its initial position
		double inertia = rigidBody->getLocalInertia()(1,1);
		double stiffness = virtualToolCoupler->getAngularStiffness();
		double naturalFrequency = sqrt(stiffness / inertia);
		double expectedSettlingTime = 4.744 / naturalFrequency;
		const int NUM_STEPS = 500;
		double dt = expectedSettlingTime / NUM_STEPS;
		double previousAngle = initialAngle;
		double currentAngle;
		for(int step=0; step<NUM_STEPS; step++)
		{
			virtualToolCoupler->update(dt);
			rigidBody->beforeUpdate(dt);
			rigidBody->update(dt);
			rigidBody->afterUpdate(dt);
			RigidTransform3d currentPose = rigidBody->getCurrentState().getPose();
			currentAngle = atan2(-currentPose(2, 0), currentPose(0, 0));
			// Check for exponential behavior. The angle should monotonically decrease and not oscillate.
			ASSERT_GT(previousAngle, currentAngle);
			previousAngle = currentAngle;
		}
		EXPECT_GT(0.051 * initialAngle, currentAngle);
	}

	std::string representationName;
	std::string inputDeviceName;
};

TEST_F(VirtualToolCouplerTest, LinearDisplacement)
{
	const double mass = rigidBody->getMass();
	virtualToolCoupler->overrideAngularDamping(mass * 1.0);
	virtualToolCoupler->overrideAngularStiffness(mass * 200);
	virtualToolCoupler->overrideLinearDamping(mass * 50);
	virtualToolCoupler->overrideLinearStiffness(mass * 200);

	RigidTransform3d initialPose = RigidTransform3d::Identity();
	initialPose.translation() = Vector3d(0.1, 0.0, 0.0);
	rigidBody->setLocalPose(initialPose);
	rigidBody->setIsGravityEnabled(false);

	std::shared_ptr<Runtime> runtime = std::make_shared<Runtime>();
	virtualToolCoupler->initialize(runtime);
	rigidBody->initialize(runtime);
	virtualToolCoupler->wakeUp();
	rigidBody->wakeUp();

	EXPECT_FALSE(rigidBody->getCurrentState().getPose().translation().isZero(epsilon));

	EXPECT_TRUE(rigidBody->isActive());
	runSystem(2);
	EXPECT_TRUE(rigidBody->isActive());

	EXPECT_FALSE(rigidBody->getCurrentState().getLinearVelocity().isZero(epsilon));
	EXPECT_TRUE(rigidBody->getCurrentState().getAngularVelocity().isZero(epsilon));

	runSystem(2000);
	EXPECT_TRUE(rigidBody->isActive());

	RigidState state = rigidBody->getCurrentState();
	EXPECT_TRUE(state.getLinearVelocity().isZero(epsilon));
	EXPECT_TRUE(state.getAngularVelocity().isZero(epsilon));
	EXPECT_TRUE(state.getPose().translation().isZero(epsilon));

	Eigen::AngleAxisd angleAxis = Eigen::AngleAxisd(state.getPose().linear());
	EXPECT_NEAR(0.0, angleAxis.angle(), epsilon);
}

TEST_F(VirtualToolCouplerTest, LinearDisplacementWithOffset)
{
	const double mass = rigidBody->getMass();
	virtualToolCoupler->overrideAngularDamping(mass * 1.0);
	virtualToolCoupler->overrideAngularStiffness(mass * 200);
	virtualToolCoupler->overrideLinearDamping(mass * 50);
	virtualToolCoupler->overrideLinearStiffness(mass * 200);
	rigidBody->setIsGravityEnabled(false);

	auto device = std::make_shared<IdentityPoseDevice>("IdentityPoseDevice");
	auto inputComponent = std::make_shared<InputComponent>("InputComponent");
	inputComponent->connectDevice(device);
	virtualToolCoupler->setInput(inputComponent);

	RigidTransform3d inputOffset = makeRigidTranslation(Vector3d(0.1, 0.0, 0.0));
	inputComponent->setLocalPose(inputOffset);

	std::shared_ptr<Runtime> runtime = std::make_shared<Runtime>();
	virtualToolCoupler->initialize(runtime);
	rigidBody->initialize(runtime);
	virtualToolCoupler->wakeUp();
	rigidBody->wakeUp();

	runSystem(2500);
	EXPECT_TRUE(rigidBody->isActive());

	RigidState state = rigidBody->getCurrentState();
	EXPECT_TRUE(state.getLinearVelocity().isZero(epsilon));
	EXPECT_TRUE(state.getAngularVelocity().isZero(epsilon));
	EXPECT_TRUE(state.getPose().translation().isApprox(inputOffset.translation(), epsilon))
		<< "Rigid Body Position is incorrect" << std::endl
		<< "  Actual: " << state.getPose().translation().transpose() << std::endl
		<< "Expected: " << inputOffset.translation().transpose() << std::endl;
}

TEST_F(VirtualToolCouplerTest, LinearDisplacementWithInertialTorques)
{
	const double mass = rigidBody->getMass();
	virtualToolCoupler->overrideAngularDamping(mass * 2);
	virtualToolCoupler->overrideAngularStiffness(mass * 20);
	virtualToolCoupler->overrideLinearDamping(mass * 5);
	virtualToolCoupler->overrideLinearStiffness(mass * 20);

	const Vector3d attachmentPoint = Vector3d::UnitY();
	virtualToolCoupler->overrideAttachmentPoint(attachmentPoint);
	virtualToolCoupler->setCalculateInertialTorques(true);

	RigidTransform3d expectedFinalPose = RigidTransform3d::Identity();
	expectedFinalPose.translation() = -attachmentPoint;
	RigidTransform3d initialPose = RigidTransform3d::Identity();
	initialPose.translation() = Vector3d(0.01, 0.0, 0.0) + expectedFinalPose.translation();
	rigidBody->setLocalPose(initialPose);
	rigidBody->setIsGravityEnabled(false);

	std::shared_ptr<Runtime> runtime = std::make_shared<Runtime>();
	virtualToolCoupler->initialize(runtime);
	rigidBody->initialize(runtime);
	virtualToolCoupler->wakeUp();
	rigidBody->wakeUp();

	RigidState state = rigidBody->getCurrentState();
	EXPECT_TRUE(state.getAngularVelocity().isZero(epsilon));
	Eigen::AngleAxisd angleAxis = Eigen::AngleAxisd(state.getPose().linear());
	EXPECT_NEAR(0.0, angleAxis.angle(), epsilon);

	EXPECT_TRUE(rigidBody->isActive());
	runSystem(2);
	EXPECT_TRUE(rigidBody->isActive());
	EXPECT_FALSE(rigidBody->getCurrentState().getAngularVelocity().isZero(epsilon));

	runSystem(6000);
	EXPECT_TRUE(rigidBody->isActive());

	state = rigidBody->getCurrentState();
	EXPECT_TRUE(state.getLinearVelocity().isZero(epsilon));
	EXPECT_TRUE(state.getAngularVelocity().isZero(epsilon));
	EXPECT_TRUE(state.getPose().isApprox(expectedFinalPose, epsilon));
}

TEST_F(VirtualToolCouplerTest, AngularDisplacement)
{
	const double mass = rigidBody->getMass();
	virtualToolCoupler->overrideAngularDamping(mass * 1.0);
	virtualToolCoupler->overrideAngularStiffness(mass * 200);
	virtualToolCoupler->overrideLinearDamping(mass * 50);
	virtualToolCoupler->overrideLinearStiffness(mass * 200);

	RigidTransform3d initialPose = RigidTransform3d::Identity();
	initialPose.linear() = Matrix33d(Eigen::AngleAxisd(M_PI/4.0, Vector3d::UnitY()));
	rigidBody->setLocalPose(initialPose);
	rigidBody->setIsGravityEnabled(false);

	std::shared_ptr<Runtime> runtime = std::make_shared<Runtime>();
	virtualToolCoupler->initialize(runtime);
	rigidBody->initialize(runtime);
	virtualToolCoupler->wakeUp();
	rigidBody->wakeUp();

	Eigen::AngleAxisd angleAxis = Eigen::AngleAxisd(rigidBody->getCurrentState().getPose().linear());
	EXPECT_NEAR(M_PI_4, angleAxis.angle(), epsilon);

	EXPECT_TRUE(rigidBody->isActive());
	runSystem(2);
	EXPECT_TRUE(rigidBody->isActive());
	EXPECT_FALSE(rigidBody->getCurrentState().getAngularVelocity().isZero(epsilon));

	runSystem(2000);
	EXPECT_TRUE(rigidBody->isActive());

	RigidState state = rigidBody->getCurrentState();
	EXPECT_TRUE(state.getLinearVelocity().isZero(epsilon));
	EXPECT_TRUE(state.getAngularVelocity().isZero(epsilon));
	EXPECT_TRUE(state.getPose().translation().isZero(epsilon));

	angleAxis = Eigen::AngleAxisd(state.getPose().linear());
	EXPECT_NEAR(0.0, angleAxis.angle(), epsilon);
}

TEST_F(VirtualToolCouplerTest, WithGravity)
{
	const double mass = rigidBody->getMass();
	virtualToolCoupler->overrideAngularDamping(mass * 1.0 );
	virtualToolCoupler->overrideAngularStiffness(mass * 200);
	virtualToolCoupler->overrideLinearDamping(mass * 50);
	virtualToolCoupler->overrideLinearStiffness(mass * 200);

	RigidTransform3d initialPose = RigidTransform3d::Identity();
	initialPose.translation() = Vector3d(0.0, 0.0, 0.0);
	rigidBody->setLocalPose(initialPose);
	rigidBody->setIsGravityEnabled(true);

	const double stiffness = 1000;
	virtualToolCoupler->overrideLinearStiffness(stiffness);

	std::shared_ptr<Runtime> runtime = std::make_shared<Runtime>();
	virtualToolCoupler->initialize(runtime);
	rigidBody->initialize(runtime);
	virtualToolCoupler->wakeUp();
	rigidBody->wakeUp();

	EXPECT_TRUE(rigidBody->getCurrentState().getPose().translation().isApprox(Vector3d::Zero(), epsilon));

	EXPECT_TRUE(rigidBody->isActive());
	runSystem(2000);
	EXPECT_TRUE(rigidBody->isActive());

	RigidState state = rigidBody->getCurrentState();
	EXPECT_TRUE(state.getLinearVelocity().isZero(epsilon));
	EXPECT_TRUE(state.getAngularVelocity().isZero(epsilon));

	Vector3d g = Vector3d(0.0, -9.81, 0.0);
	Vector3d expectedPosition = rigidBody->getMass() * g / stiffness;
	EXPECT_TRUE(state.getPose().translation().isApprox(expectedPosition, epsilon));

	Eigen::AngleAxisd angleAxis = Eigen::AngleAxisd(state.getPose().linear());
	EXPECT_NEAR(0.0, angleAxis.angle(), epsilon);
}

TEST_F(VirtualToolCouplerTest, DefaultLinearParameters)
{
	SCOPED_TRACE("Default Linear Parameters");
	checkLinearIsCriticallyDamped();
}

TEST_F(VirtualToolCouplerTest, SetLinearStiffness)
{
	SCOPED_TRACE("Set Linear Stiffness");
	virtualToolCoupler->overrideLinearStiffness(1.234);
	checkLinearIsCriticallyDamped();
}

TEST_F(VirtualToolCouplerTest, SetLinearDamping)
{
	SCOPED_TRACE("Set Linear Damping");
	virtualToolCoupler->overrideLinearDamping(500.2);
	checkLinearIsCriticallyDamped();
}

TEST_F(VirtualToolCouplerTest, DefaultAngularParameters)
{
	SCOPED_TRACE("Default Linear Parameters");
	checkAngularIsCriticallyDamped();
}

TEST_F(VirtualToolCouplerTest, SetAngularStiffness)
{
	SCOPED_TRACE("Set Angular Stiffness");
	virtualToolCoupler->overrideAngularStiffness(1234.6);
	checkAngularIsCriticallyDamped();
}

TEST_F(VirtualToolCouplerTest, SetAngularDamping)
{
	SCOPED_TRACE("Set Angular Damping");
	virtualToolCoupler->overrideAngularDamping(0.1235);
	checkAngularIsCriticallyDamped();
}

TEST_F(VirtualToolCouplerTest, SetHapticOutputOnlyWhenColliding)
{
	RigidTransform3d initialPose =
		Math::makeRigidTransform(Math::makeRotationQuaternion(10.0, Vector3d::UnitX().eval()), Vector3d(0.1, 0.0, 0.0));
	rigidBody->setLocalPose(initialPose);
	rigidBody->setIsGravityEnabled(false);
	auto input = std::make_shared<Testing::MockInputComponent>("input");
	DataStructures::DataGroupBuilder builder;
	builder.addPose(DataStructures::Names::POSE);
	DataStructures::DataGroup inputData = builder.createData();
	inputData.poses().set(DataStructures::Names::POSE, RigidTransform3d::Identity());
	input->setData(inputData);
	virtualToolCoupler->setInput(input);
	auto output = std::make_shared<Input::OutputComponent>("output");
	std::shared_ptr<Runtime> runtime = std::make_shared<Runtime>();
	virtualToolCoupler->initialize(runtime);
	rigidBody->initialize(runtime);
	input->initialize(runtime);
	output->initialize(runtime);
	virtualToolCoupler->wakeUp();
	rigidBody->wakeUp();
	input->wakeUp();
	output->wakeUp();

	EXPECT_FALSE(virtualToolCoupler->isHapticOutputOnlyWhenColliding());
	virtualToolCoupler->setHapticOutputOnlyWhenColliding(true);
	EXPECT_TRUE(virtualToolCoupler->isHapticOutputOnlyWhenColliding());

	// no OutputComponent
	virtualToolCoupler->update(0.1);
	DataStructures::DataGroup data = virtualToolCoupler->getOutputData();
	ASSERT_TRUE(data.vectors().hasEntry(DataStructures::Names::FORCE));
	EXPECT_FALSE(data.vectors().hasData(DataStructures::Names::FORCE));
	ASSERT_TRUE(data.vectors().hasEntry(DataStructures::Names::TORQUE));
	EXPECT_FALSE(data.vectors().hasData(DataStructures::Names::TORQUE));
	ASSERT_TRUE(data.matrices().hasEntry(DataStructures::Names::DAMPER_JACOBIAN));
	EXPECT_FALSE(data.matrices().hasData(DataStructures::Names::DAMPER_JACOBIAN));
	ASSERT_TRUE(data.matrices().hasEntry(DataStructures::Names::SPRING_JACOBIAN));
	EXPECT_FALSE(data.matrices().hasData(DataStructures::Names::SPRING_JACOBIAN));

	// no collision representation
	virtualToolCoupler->setOutput(output);
	virtualToolCoupler->update(0.1);
	data = virtualToolCoupler->getOutputData();
	Vector3d force;
	Vector3d torque;
	ASSERT_TRUE(data.vectors().get(DataStructures::Names::FORCE, &force));
	EXPECT_GT(force.norm(), 0.0);
	ASSERT_TRUE(data.vectors().get(DataStructures::Names::TORQUE, &torque));
	EXPECT_GT(torque.norm(), 0.0);
	EXPECT_TRUE(data.matrices().hasData(SurgSim::DataStructures::Names::DAMPER_JACOBIAN));
	EXPECT_TRUE(data.matrices().hasData(SurgSim::DataStructures::Names::SPRING_JACOBIAN));

	// forces disabled in free motion
	auto collision = std::make_shared<RigidCollisionRepresentation>("collision");
	rigidBody->setCollisionRepresentation(collision);
	virtualToolCoupler->update(0.1);
	data = virtualToolCoupler->getOutputData();
	ASSERT_TRUE(data.vectors().get(DataStructures::Names::FORCE, &force));
	EXPECT_EQ(force, Vector3d::Zero());
	ASSERT_TRUE(data.vectors().get(DataStructures::Names::TORQUE, &torque));
	EXPECT_EQ(torque, Vector3d::Zero());
	EXPECT_FALSE(data.matrices().hasData(DataStructures::Names::DAMPER_JACOBIAN));
	EXPECT_FALSE(data.matrices().hasData(DataStructures::Names::SPRING_JACOBIAN));

	// no flag
	virtualToolCoupler->setHapticOutputOnlyWhenColliding(false);
	virtualToolCoupler->update(0.1);
	data = virtualToolCoupler->getOutputData();
	ASSERT_TRUE(data.vectors().get(DataStructures::Names::FORCE, &force));
	EXPECT_GT(force.norm(), 0.0);
	ASSERT_TRUE(data.vectors().get(DataStructures::Names::TORQUE, &torque));
	EXPECT_GT(torque.norm(), 0.0);
	EXPECT_TRUE(data.matrices().hasData(DataStructures::Names::DAMPER_JACOBIAN));
	EXPECT_TRUE(data.matrices().hasData(DataStructures::Names::SPRING_JACOBIAN));

	// forces enabled if any collisions
	virtualToolCoupler->setHapticOutputOnlyWhenColliding(true);
	auto& collisions = collision->getCollisions().unsafeGet();
	collisions[std::make_shared<RigidCollisionRepresentation>("collision2")].push_back(
		std::make_shared<Collision::Contact>(0.1, Vector3d::UnitX().eval(), Vector3d::UnitY().eval(),
		std::make_pair(DataStructures::Location(), DataStructures::Location())));
	virtualToolCoupler->update(0.1);
	data = virtualToolCoupler->getOutputData();
	ASSERT_TRUE(data.vectors().get(DataStructures::Names::FORCE, &force));
	EXPECT_GT(force.norm(), 0.0);
	ASSERT_TRUE(data.vectors().get(DataStructures::Names::TORQUE, &torque));
	EXPECT_GT(torque.norm(), 0.0);
	EXPECT_TRUE(data.matrices().hasData(DataStructures::Names::DAMPER_JACOBIAN));
	EXPECT_TRUE(data.matrices().hasData(DataStructures::Names::SPRING_JACOBIAN));
}

TEST_F(VirtualToolCouplerTest, GetInput)
{
	EXPECT_EQ(input, virtualToolCoupler->getInput());
}

TEST_F(VirtualToolCouplerTest, GetPoseName)
{
	EXPECT_EQ(SurgSim::DataStructures::Names::POSE, virtualToolCoupler->getPoseName());
}

TEST_F(VirtualToolCouplerTest, GetRigid)
{
	EXPECT_EQ(rigidBody, virtualToolCoupler->getRepresentation());
}

typedef SurgSim::DataStructures::OptionalValue<double> OptionalValued;
typedef SurgSim::DataStructures::OptionalValue<Vector3d> OptionalValueVec3;

TEST_F(VirtualToolCouplerTest, OptionalParams)
{
	double num = 2.56527676;
	Vector3d vec(1.0, 2.0, 3.0);
	{
		SCOPED_TRACE("Getters");
		EXPECT_FALSE(virtualToolCoupler->getOptionalLinearStiffness().hasValue());
		virtualToolCoupler->overrideLinearStiffness(num);
		const OptionalValued& linearStiffness = virtualToolCoupler->getOptionalLinearStiffness();
		EXPECT_TRUE(linearStiffness.hasValue());
		EXPECT_EQ(num, linearStiffness.getValue());

		EXPECT_FALSE(virtualToolCoupler->getOptionalLinearDamping().hasValue());
		virtualToolCoupler->overrideLinearDamping(num);
		const OptionalValued& linearDamping = virtualToolCoupler->getOptionalLinearDamping();
		EXPECT_TRUE(linearDamping.hasValue());
		EXPECT_EQ(num, linearDamping.getValue());

		EXPECT_FALSE(virtualToolCoupler->getOptionalAngularStiffness().hasValue());
		virtualToolCoupler->overrideAngularStiffness(num);
		const OptionalValued& angularStiffness = virtualToolCoupler->getOptionalAngularStiffness();
		EXPECT_TRUE(angularStiffness.hasValue());
		EXPECT_EQ(num, angularStiffness.getValue());

		EXPECT_FALSE(virtualToolCoupler->getOptionalAngularDamping().hasValue());
		virtualToolCoupler->overrideAngularDamping(num);
		const OptionalValued& angularDamping = virtualToolCoupler->getOptionalAngularDamping();
		EXPECT_TRUE(angularDamping.hasValue());
		EXPECT_EQ(num, angularDamping.getValue());

		EXPECT_FALSE(virtualToolCoupler->getOptionalAttachmentPoint().hasValue());
		virtualToolCoupler->overrideAttachmentPoint(vec);
		const OptionalValueVec3& attachmentPoint = virtualToolCoupler->getOptionalAttachmentPoint();
		EXPECT_TRUE(attachmentPoint.hasValue());
		EXPECT_TRUE(vec.isApprox(attachmentPoint.getValue()));
	}
	{
		SCOPED_TRACE("Setters");
		OptionalValued optionalNum;
		optionalNum.setValue(num);

		OptionalValueVec3 optionalVec;
		optionalVec.setValue(vec);

		virtualToolCoupler->overrideLinearStiffness(0.0);
		EXPECT_NEAR(0.0, virtualToolCoupler->getOptionalLinearStiffness().getValue(), 1e-9);
		virtualToolCoupler->setOptionalLinearStiffness(optionalNum);
		const OptionalValued& linearStiffness = virtualToolCoupler->getOptionalLinearStiffness();
		EXPECT_EQ(num, linearStiffness.getValue());

		virtualToolCoupler->overrideLinearDamping(0.0);
		EXPECT_NEAR(0.0, virtualToolCoupler->getOptionalLinearDamping().getValue(), 1e-9);
		virtualToolCoupler->setOptionalLinearDamping(optionalNum);
		const OptionalValued& linearDamping = virtualToolCoupler->getOptionalLinearDamping();
		EXPECT_TRUE(linearDamping.hasValue());
		EXPECT_EQ(num, linearDamping.getValue());

		virtualToolCoupler->overrideAngularStiffness(0.0);
		EXPECT_NEAR(0.0, virtualToolCoupler->getOptionalAngularStiffness().getValue(), 1e-9);
		virtualToolCoupler->setOptionalAngularStiffness(optionalNum);
		const OptionalValued& angularStiffness = virtualToolCoupler->getOptionalAngularStiffness();
		EXPECT_TRUE(angularStiffness.hasValue());
		EXPECT_EQ(num, angularStiffness.getValue());

		virtualToolCoupler->overrideAngularDamping(0.0);
		EXPECT_NEAR(0.0, virtualToolCoupler->getOptionalAngularDamping().getValue(), 1e-9);
		virtualToolCoupler->setOptionalAngularDamping(optionalNum);
		const OptionalValued& angularDamping = virtualToolCoupler->getOptionalAngularDamping();
		EXPECT_TRUE(angularDamping.hasValue());
		EXPECT_EQ(num, angularDamping.getValue());

		virtualToolCoupler->overrideAttachmentPoint(Vector3d::Zero());
		EXPECT_TRUE(virtualToolCoupler->getOptionalAttachmentPoint().getValue().isApprox(Vector3d::Zero()));
		virtualToolCoupler->setOptionalAttachmentPoint(optionalVec);
		const OptionalValueVec3& attachmentPoint = virtualToolCoupler->getOptionalAttachmentPoint();
		EXPECT_TRUE(attachmentPoint.hasValue());
		EXPECT_TRUE(vec.isApprox(attachmentPoint.getValue()));
	}
}

TEST_F(VirtualToolCouplerTest, Serialization)
{
	double num = 3.6415;
	Vector3d vec(28.4, -37.2, 91.8);

	OptionalValued optionalNum;
	optionalNum.setValue(num);
	OptionalValueVec3 optionalVec;
	optionalVec.setValue(vec);
	virtualToolCoupler->setOptionalLinearStiffness(optionalNum);
	virtualToolCoupler->setOptionalLinearDamping(optionalNum);
	virtualToolCoupler->setOptionalAngularStiffness(optionalNum);
	virtualToolCoupler->setOptionalAngularDamping(optionalNum);
	virtualToolCoupler->setOptionalAttachmentPoint(optionalVec);
	virtualToolCoupler->setCalculateInertialTorques(true);

	virtualToolCoupler->setHapticOutputOnlyWhenColliding(true);

	// Encode
	YAML::Node node;
	EXPECT_NO_THROW(node = YAML::convert<SurgSim::Framework::Component>::encode(*virtualToolCoupler););

	// Decode
	std::shared_ptr<SurgSim::Physics::VirtualToolCoupler> newVirtualToolCoupler;
	EXPECT_NO_THROW(newVirtualToolCoupler = std::dynamic_pointer_cast<VirtualToolCoupler>(
		node.as<std::shared_ptr<SurgSim::Framework::Component>>()));

	// Verify
	newVirtualToolCoupler->initialize(std::make_shared<SurgSim::Framework::Runtime>());
	newVirtualToolCoupler->wakeUp();

	EXPECT_EQ(num, newVirtualToolCoupler->getLinearStiffness());
	EXPECT_EQ(num, newVirtualToolCoupler->getLinearDamping());
	EXPECT_EQ(num, newVirtualToolCoupler->getAngularStiffness());
	EXPECT_EQ(num, newVirtualToolCoupler->getAngularDamping());
	EXPECT_TRUE(vec.isApprox(newVirtualToolCoupler->getAttachmentPoint()));
	EXPECT_TRUE(newVirtualToolCoupler->getCalculateInertialTorques());
	EXPECT_TRUE(virtualToolCoupler->isHapticOutputOnlyWhenColliding());

	EXPECT_NE(nullptr, newVirtualToolCoupler->getInput());
	EXPECT_NE(nullptr, newVirtualToolCoupler->getRepresentation());
	EXPECT_EQ(nullptr, newVirtualToolCoupler->getOutput());

	YAML::Node inputNode;
	EXPECT_NO_THROW(inputNode = YAML::convert<SurgSim::Framework::Component>::encode(*input););

	YAML::Node representationNode;
	EXPECT_NO_THROW(representationNode = YAML::convert<SurgSim::Framework::Component>::encode(*rigidBody););

	EXPECT_EQ(inputNode[input->getClassName()]["Id"].as<std::string>(),
		node[virtualToolCoupler->getClassName()][input->getName()][input->getClassName()]["Id"].as<std::string>());
}

TEST_F(VirtualToolCouplerTest, OutputDataEntries)
{
	std::shared_ptr<Runtime> runtime = std::make_shared<Runtime>();
	ASSERT_TRUE(virtualToolCoupler->initialize(runtime));
	auto data = virtualToolCoupler->getOutputData();

	EXPECT_EQ(1, data.poses().getNumEntries());
	EXPECT_TRUE(data.poses().hasEntry(SurgSim::DataStructures::Names::INPUT_POSE));

	EXPECT_EQ(4, data.vectors().getNumEntries());
	EXPECT_TRUE(data.vectors().hasEntry(SurgSim::DataStructures::Names::FORCE));
	EXPECT_TRUE(data.vectors().hasEntry(SurgSim::DataStructures::Names::TORQUE));
	EXPECT_TRUE(data.vectors().hasEntry(SurgSim::DataStructures::Names::INPUT_LINEAR_VELOCITY));
	EXPECT_TRUE(data.vectors().hasEntry(SurgSim::DataStructures::Names::INPUT_ANGULAR_VELOCITY));

	EXPECT_EQ(2, data.matrices().getNumEntries());
	EXPECT_TRUE(data.matrices().hasEntry(SurgSim::DataStructures::Names::SPRING_JACOBIAN));
	EXPECT_TRUE(data.matrices().hasEntry(SurgSim::DataStructures::Names::DAMPER_JACOBIAN));

	EXPECT_EQ(0, data.scalars().getNumEntries());
	EXPECT_EQ(0, data.integers().getNumEntries());
	EXPECT_EQ(0, data.booleans().getNumEntries());
	EXPECT_EQ(0, data.strings().getNumEntries());
	EXPECT_EQ(0, data.customData().getNumEntries());
}

}; // namespace Physics
}; // namespace SurgSim
