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
#include <yaml-cpp/yaml.h>

#include <math.h>

#include "SurgSim/Devices/IdentityPoseDevice/IdentityPoseDevice.h"
#include "SurgSim/Input/InputComponent.h"
#include "SurgSim/Framework/FrameworkConvert.h"
#include "SurgSim/Framework/Runtime.h"
#include "SurgSim/Math/Matrix.h"
#include "SurgSim/Math/Quaternion.h"
#include "SurgSim/Math/RigidTransform.h"
#include "SurgSim/Math/SphereShape.h"
#include "SurgSim/Math/Vector.h"
#include "SurgSim/Physics/RigidRepresentation.h"
#include "SurgSim/Physics/RigidRepresentationParameters.h"
#include "SurgSim/Physics/RigidRepresentationState.h"
#include "SurgSim/Physics/UnitTests/MockObjects.h"
#include "SurgSim/Physics/VirtualToolCoupler.h"

using SurgSim::Device::IdentityPoseDevice;
using SurgSim::Input::InputComponent;
using SurgSim::Framework::Runtime;
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

		RigidRepresentationParameters parameters;
		parameters.setDensity(700.0);
		parameters.setAngularDamping(0.0);
		parameters.setLinearDamping(0.0);
		parameters.setShapeUsedForMassInertia(std::make_shared<SphereShape>(0.1));
		rigidBody->setInitialParameters(parameters);

		RigidRepresentationState state;
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
		double mass = rigidBody->getCurrentParameters().getMass();
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
		double initialAngle = M_PI/4.0;
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
		double inertia = rigidBody->getCurrentParameters().getLocalInertia()(1,1);
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
	const double mass = rigidBody->getCurrentParameters().getMass();
	virtualToolCoupler->overrideAngularDamping(mass * 1.0 );
	virtualToolCoupler->overrideAngularStiffness(mass * 200);
	virtualToolCoupler->overrideLinearDamping(mass * 50);
	virtualToolCoupler->overrideLinearStiffness(mass * 200);

	RigidTransform3d initialPose = RigidTransform3d::Identity();
	initialPose.translation() = Vector3d(0.1, 0.0, 0.0);
	rigidBody->setLocalPose(initialPose);
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
	const double mass = rigidBody->getCurrentParameters().getMass();
	virtualToolCoupler->overrideAngularDamping(mass * 1.0 );
	virtualToolCoupler->overrideAngularStiffness(mass * 200);
	virtualToolCoupler->overrideLinearDamping(mass * 50);
	virtualToolCoupler->overrideLinearStiffness(mass * 200);


	RigidTransform3d initialPose = RigidTransform3d::Identity();
	initialPose.linear() = Matrix33d(Eigen::AngleAxisd(M_PI/4.0, Vector3d::UnitY()));
	rigidBody->setLocalPose(initialPose);
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
	const double mass = rigidBody->getCurrentParameters().getMass();
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

TEST_F(VirtualToolCouplerTest, OptionalParams)
{
	double num = 2.56527676;
	{
		SCOPED_TRACE("Getters");
		virtualToolCoupler->overrideLinearStiffness(num);
		virtualToolCoupler->overrideLinearDamping(num);
		virtualToolCoupler->overrideAngularStiffness(num);
		virtualToolCoupler->overrideAngularDamping(num);

		const OptionalValued& linearStiffness = virtualToolCoupler->getOptionalLinearStiffness();
		const OptionalValued& linearDamping = virtualToolCoupler->getOptionalLinearDamping();
		const OptionalValued& angularStiffness = virtualToolCoupler->getOptionalAngularStiffness();
		const OptionalValued& angularDamping = virtualToolCoupler->getOptionalAngularDamping();

		EXPECT_TRUE(linearStiffness.hasValue());
		EXPECT_TRUE(linearDamping.hasValue());
		EXPECT_TRUE(angularStiffness.hasValue());
		EXPECT_TRUE(angularDamping.hasValue());

		EXPECT_EQ(num, linearStiffness.getValue());
		EXPECT_EQ(num, linearDamping.getValue());
		EXPECT_EQ(num, angularStiffness.getValue());
		EXPECT_EQ(num, angularDamping.getValue());
	}
	{
		SCOPED_TRACE("Setters");
		virtualToolCoupler->overrideLinearStiffness(0.0);
		virtualToolCoupler->overrideLinearDamping(0.0);
		virtualToolCoupler->overrideAngularStiffness(0.0);
		virtualToolCoupler->overrideAngularDamping(0.0);

		OptionalValued optionalNum;
		optionalNum.setValue(num);

		virtualToolCoupler->setOptionalLinearStiffness(optionalNum);
		virtualToolCoupler->setOptionalLinearDamping(optionalNum);
		virtualToolCoupler->setOptionalAngularStiffness(optionalNum);
		virtualToolCoupler->setOptionalAngularDamping(optionalNum);

		const OptionalValued& linearStiffness = virtualToolCoupler->getOptionalLinearStiffness();
		const OptionalValued& linearDamping = virtualToolCoupler->getOptionalLinearDamping();
		const OptionalValued& angularStiffness = virtualToolCoupler->getOptionalAngularStiffness();
		const OptionalValued& angularDamping = virtualToolCoupler->getOptionalAngularDamping();

		EXPECT_TRUE(linearStiffness.hasValue());
		EXPECT_TRUE(linearDamping.hasValue());
		EXPECT_TRUE(angularStiffness.hasValue());
		EXPECT_TRUE(angularDamping.hasValue());

		EXPECT_EQ(num, linearStiffness.getValue());
		EXPECT_EQ(num, linearDamping.getValue());
		EXPECT_EQ(num, angularStiffness.getValue());
		EXPECT_EQ(num, angularDamping.getValue());
	}
}

TEST_F(VirtualToolCouplerTest, OutputScaling)
{
	double num = 2.56527676;
	virtualToolCoupler->setOutputForceScaling(num);
	virtualToolCoupler->setOutputTorqueScaling(num);
	EXPECT_EQ(num, virtualToolCoupler->getOutputForceScaling());
	EXPECT_EQ(num, virtualToolCoupler->getOutputTorqueScaling());
}

TEST_F(VirtualToolCouplerTest, Attachment)
{
	Vector3d attachment(28.4, -37.2, 91.8);
	virtualToolCoupler->setAttachment(attachment);
	EXPECT_TRUE(attachment.isApprox(virtualToolCoupler->getAttachment()));
}

TEST_F(VirtualToolCouplerTest, Serialization)
{
	double num = 3.6415;
	Vector3d attachment(28.4, -37.2, 91.8);

	OptionalValued optionalNum;
	optionalNum.setValue(num);
	virtualToolCoupler->setOptionalLinearStiffness(optionalNum);
	virtualToolCoupler->setOptionalLinearDamping(optionalNum);
	virtualToolCoupler->setOptionalAngularStiffness(optionalNum);
	virtualToolCoupler->setOptionalAngularDamping(optionalNum);
	virtualToolCoupler->setOutputForceScaling(num);
	virtualToolCoupler->setOutputTorqueScaling(num);
	virtualToolCoupler->setAttachment(attachment);

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

	EXPECT_EQ(num, newVirtualToolCoupler->getOutputForceScaling());
	EXPECT_EQ(num, newVirtualToolCoupler->getOutputTorqueScaling());

	EXPECT_NE(nullptr, newVirtualToolCoupler->getInput());
	EXPECT_NE(nullptr, newVirtualToolCoupler->getRepresentation());
	EXPECT_EQ(nullptr, newVirtualToolCoupler->getOutput());

	EXPECT_TRUE(attachment.isApprox(newVirtualToolCoupler->getAttachment()));

	YAML::Node inputNode;
	EXPECT_NO_THROW(inputNode = YAML::convert<SurgSim::Framework::Component>::encode(*input););

	YAML::Node representationNode;
	EXPECT_NO_THROW(representationNode = YAML::convert<SurgSim::Framework::Component>::encode(*rigidBody););

	EXPECT_EQ(inputNode[input->getClassName()]["Id"].as<std::string>(),
		node[virtualToolCoupler->getClassName()][input->getName()][input->getClassName()]["Id"].as<std::string>());
}

}; // namespace Physics
}; // namespace SurgSim
