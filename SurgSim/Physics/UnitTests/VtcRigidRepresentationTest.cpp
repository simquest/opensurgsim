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

#include <string>

#include <SurgSim/Physics/VtcRigidRepresentation.h>
using SurgSim::Physics::VtcRigidRepresentation;
using SurgSim::Physics::VtcRigidParameters;
using SurgSim::Physics::RigidRepresentationParameters;
using SurgSim::Physics::RigidRepresentationState;
using SurgSim::Physics::SphereShape;

#include <SurgSim/Math/Vector.h>
#include <SurgSim/Math/Matrix.h>
#include <SurgSim/Math/Quaternion.h>
#include <SurgSim/Math/RigidTransform.h>

class VtcRigidRepresentationTest : public ::testing::Test
{
public:
	void SetUp()
	{
		m_dt = 1e-3;
		m_dtDivergenceTest = 1e+3;

		double radius = 0.1;
		m_param.setDensity(9000.0);
		m_param.setAngularDamping(0.0);
		m_param.setLinearDamping(0.0);
		m_param.setShapeUsedForMassInertia(std::make_shared<SphereShape>(radius));

		{
			SurgSim::Math::Quaterniond   q(0.5, 0.4, 0.3, 0.2);
			SurgSim::Math::Vector3d linVel(3.0, 2.0, 1.0);
			SurgSim::Math::Vector3d angVel(1.0, 2.0, 3.0);
			SurgSim::Math::Vector3d      t(1.2, 2.1, 12.21);
			m_state.setAngularVelocity(angVel);
			m_state.setLinearVelocity(linVel);
			q.normalize();
			m_state.setPose(SurgSim::Math::makeRigidTransform(q, t));
		}

		// State to be used to test divergence
		const SurgSim::Math::Vector3d max(DBL_MAX, DBL_MAX, DBL_MAX);
		m_stateDivergence.setAngularVelocity(max);

		m_vtcParam.setVtcAngularDamping(5.0);
		m_vtcParam.setVtcAngularStiffness(1000.0);
		m_vtcParam.setVtcLinearDamping(1.0);
		m_vtcParam.setVtcLinearStiffness(1000.0);

		{
			SurgSim::Math::Quaterniond   q(0.5, 0.4, 0.3, 0.2);
			SurgSim::Math::Vector3d angVel(1.0, 2.0, 3.0);
			SurgSim::Math::Vector3d linVel(1.0, 2.0, 3.0);
			SurgSim::Math::Vector3d      t(1.0, 2.0, 3.0);
			m_vtcState.setAngularVelocity(angVel);
			m_vtcState.setLinearVelocity(linVel);
			q.normalize();
			m_vtcState.setPose(SurgSim::Math::makeRigidTransform(q, t));
		}

		m_maxNumSimulationStepTest = 1;
	}

	void TearDown()
	{
	}

	// Time step
	double m_dt;
	double m_dtDivergenceTest;

	// Rigid representation parameters
	RigidRepresentationParameters m_param;

	// Rigid representation default parameters
	RigidRepresentationParameters m_defaultParameters;

	// Vtc default parameters
	VtcRigidParameters m_vtcParamDefault;

	// Vtc current parameters
	VtcRigidParameters m_vtcParam;

	// Vtc default states
	RigidRepresentationState m_vtcStateDefault;

	// Vtc current states
	RigidRepresentationState m_vtcState;

	// Rigid representation state
	RigidRepresentationState m_state;

	// Rigid representation state for divergence test
	RigidRepresentationState m_stateDivergence;

	// Rigid representation default state
	RigidRepresentationState m_defaultState;

	// Max number of simulation step for testing
	int m_maxNumSimulationStepTest;
};


TEST_F(VtcRigidRepresentationTest, ConstructorTest)
{
	ASSERT_NO_THROW( {VtcRigidRepresentation rigidBody("Rigid Vtc");});
}

TEST_F(VtcRigidRepresentationTest, ResetTest)
{
	SurgSim::Math::RigidTransform3d id4x4 = SurgSim::Math::RigidTransform3d::Identity();

	// Create the rigid body
	std::shared_ptr<VtcRigidRepresentation> rigidBody = std::make_shared<VtcRigidRepresentation>("Rigid Vtc");

	rigidBody->setInitialState(m_state);
	rigidBody->setCurrentParameters(m_param);
	rigidBody->setCurrentVtcParameters(m_vtcParam);
	rigidBody->setInitialVtcState(m_vtcState);
	rigidBody->setIsActive(false);
	rigidBody->setIsGravityEnabled(false);
	rigidBody->setPose(id4x4);

	// reset the representation state
	rigidBody->resetState();

	// Parameters unchanged
	EXPECT_EQ(m_param, rigidBody->getCurrentParameters());
	// Vtc initial parameters unchanged
	EXPECT_EQ(m_vtcParamDefault, rigidBody->getInitialVtcParameters());
	// Vtc Parameters unchanged
	EXPECT_EQ(m_vtcParam, rigidBody->getCurrentVtcParameters());

	// isActive flag unchanged
	EXPECT_FALSE(rigidBody->isActive());
	// isGravityEnable flag unchanged
	EXPECT_FALSE(rigidBody->isGravityEnabled());
	// current state = initial state
	EXPECT_EQ(rigidBody->getInitialState(), rigidBody->getCurrentState());
	// current pose = initial pose != id4x4
	{
		using SurgSim::Math::RigidTransform3d;
		const RigidTransform3d& pose = rigidBody->getCurrentState().getPose();
		const RigidTransform3d& initialPose = rigidBody->getInitialState().getPose();

		EXPECT_NE(id4x4.translation(), pose.translation());
		EXPECT_NE(id4x4.rotation(), pose.rotation());
		EXPECT_EQ(initialPose.translation(), pose.translation());
		EXPECT_EQ(initialPose.rotation(), pose.rotation());
	}
	// previous state = initial state
	EXPECT_EQ(rigidBody->getInitialState(), rigidBody->getPreviousState());
	// Vtc current state unchanged
	EXPECT_TRUE(rigidBody->getCurrentVtcState().getPose().isApprox(id4x4));
	// Vtc previous state unchanged
	EXPECT_EQ(m_vtcState, rigidBody->getPreviousVtcState());

	// reset the representation parameters
	rigidBody->resetParameters();

	// Parameters reset to initial
	EXPECT_EQ(m_defaultParameters, rigidBody->getCurrentParameters());
	// Vtc Parameters unchanged
	EXPECT_EQ(m_vtcParam, rigidBody->getCurrentVtcParameters());

	// reset the Vtc parameters
	rigidBody->resetVtcParameters();

	// Vtc Parameters reset to initial
	EXPECT_EQ(m_vtcParamDefault, rigidBody->getCurrentVtcParameters());
}

TEST_F(VtcRigidRepresentationTest, SetGetAndDefaultValueTest)
{
	// Create the rigid body
	std::shared_ptr<VtcRigidRepresentation> rigidBody = std::make_shared<VtcRigidRepresentation>("Rigid Vtc");

	// Get state (current, initial)
	EXPECT_EQ(m_vtcStateDefault, rigidBody->getCurrentVtcState());
	EXPECT_EQ(m_vtcStateDefault, rigidBody->getPreviousVtcState());
	EXPECT_EQ(m_vtcStateDefault, rigidBody->getInitialVtcState());
	rigidBody->setInitialVtcState(m_vtcState);
	EXPECT_EQ(m_vtcState, rigidBody->getInitialVtcState());
	EXPECT_EQ(m_vtcState, rigidBody->getCurrentVtcState());
	EXPECT_EQ(m_vtcState, rigidBody->getPreviousVtcState());

	// Get Vtc parameters (current, initial)
	EXPECT_EQ(m_vtcParamDefault, rigidBody->getCurrentVtcParameters());
	EXPECT_EQ(m_vtcParamDefault, rigidBody->getInitialVtcParameters());
	rigidBody->setInitialVtcParameters(m_vtcParam);
	EXPECT_EQ(m_vtcParam, rigidBody->getInitialVtcParameters());
	EXPECT_EQ(m_vtcParam, rigidBody->getCurrentVtcParameters());

	// Get state (current, initial)
	EXPECT_EQ(m_defaultState, rigidBody->getCurrentState());
	EXPECT_EQ(m_defaultState, rigidBody->getInitialState());
	rigidBody->setInitialState(m_state);
	EXPECT_EQ(m_state, rigidBody->getInitialState());
	EXPECT_EQ(m_state, rigidBody->getCurrentState());

	// Get parameters (current, initial)
	EXPECT_EQ(m_defaultParameters, rigidBody->getCurrentParameters());
	EXPECT_EQ(m_defaultParameters, rigidBody->getInitialParameters());
	rigidBody->setInitialParameters(m_param);
	EXPECT_EQ(m_param, rigidBody->getInitialParameters());
	EXPECT_EQ(m_param, rigidBody->getCurrentParameters());

	// Get/Set active flag [default = true]
	EXPECT_TRUE(rigidBody->isActive());
	rigidBody->setIsActive(false);
	ASSERT_FALSE(rigidBody->isActive());
	rigidBody->setIsActive(true);
	ASSERT_TRUE(rigidBody->isActive());

	// Get numDof = 6
	ASSERT_EQ(6u, rigidBody->getNumDof());

	// Set/Get isGravityEnabled [default = true]
	EXPECT_TRUE(rigidBody->isGravityEnabled());
	rigidBody->setIsGravityEnabled(false);
	ASSERT_FALSE(rigidBody->isGravityEnabled());
	rigidBody->setIsGravityEnabled(true);
	ASSERT_TRUE(rigidBody->isGravityEnabled());
}

TEST_F(VtcRigidRepresentationTest, NoForceTorqueTest)
{
	using SurgSim::Math::Vector3d;
	using SurgSim::Math::Matrix33d;
	using SurgSim::Math::Quaterniond;

	// Create the rigid body
	std::shared_ptr<VtcRigidRepresentation> rigidBody = std::make_shared<VtcRigidRepresentation>("Rigid Vtc");

	// Setup phase
	rigidBody->setIsActive(true);
	rigidBody->setIsGravityEnabled(false);
	rigidBody->setInitialParameters(m_param);

	// Run few time steps
	for (int timeStep = 0; timeStep < m_maxNumSimulationStepTest; timeStep++)
	{
		rigidBody->beforeUpdate(m_dt);
		rigidBody->update(m_dt);
		rigidBody->afterUpdate(m_dt);
	}

	const RigidRepresentationState& state = rigidBody->getCurrentState();

	const Vector3d    G = state.getPose().translation();
	const Matrix33d&  R = state.getPose().rotation();
	const Quaterniond q = Quaterniond(R);
	const Vector3d    v = state.getLinearVelocity();
	const Vector3d    w = state.getAngularVelocity();

	ASSERT_EQ(Vector3d::Zero(), G);
	ASSERT_TRUE(q.isApprox(Quaterniond::Identity()));
	ASSERT_EQ(Vector3d::Zero(), v);
	ASSERT_EQ(Vector3d::Zero(), w);
}

// The rigid representation and the vtc are both initialized with
// position (0 0 0) orientation (0 0 0 1)
// velocity linear (0 0 0)/angular(0 0 0)
// The Vtc does not move, holding the rigid representation in place,
// fighting the gravity force.
TEST_F(VtcRigidRepresentationTest, GravityTest)
{
	using SurgSim::Math::Vector3d;
	using SurgSim::Math::Matrix33d;
	using SurgSim::Math::Quaterniond;

	// Create the rigid body
	std::shared_ptr<VtcRigidRepresentation> rigidBody = std::make_shared<VtcRigidRepresentation>("Rigid Vtc");

	// Setup phase
	rigidBody->setIsActive(true);
	rigidBody->setIsGravityEnabled(true);
	rigidBody->setInitialParameters(m_param);
	rigidBody->setInitialVtcParameters(m_vtcParam);

	// Run few time steps
	for (int timeStep = 0; timeStep < m_maxNumSimulationStepTest; timeStep++)
	{
		rigidBody->beforeUpdate(m_dt);
		rigidBody->update(m_dt);
		rigidBody->afterUpdate(m_dt);

		const RigidRepresentationState &state = rigidBody->getCurrentState();
		const RigidRepresentationState &vtcState = rigidBody->getCurrentVtcState();

		const Vector3d    G = state.getPose().translation();
		const Matrix33d&  R = state.getPose().rotation();
		const Quaterniond q = Quaterniond(R);
		const Vector3d    w = state.getAngularVelocity();

		// 1mm margin with a stiff Vtc to keep the virtual object close by
		double epsilon = 1e-3;
		Vector3d diff = G - vtcState.getPose().translation();
		ASSERT_NEAR(0.0, diff.norm(), epsilon);
		// We do not test the linear velocity as it varies to get the position
		// close to the Vtc target
		// Angular velocity should not be affected, nor the orientation
		ASSERT_TRUE(q.isApprox(Quaterniond::Identity()));
		ASSERT_EQ(Vector3d::Zero(), w);
	}
}

TEST_F(VtcRigidRepresentationTest, DisableWhenDivergeTest)
{
	// Create the rigid body
	std::shared_ptr<VtcRigidRepresentation> rigidBody = std::make_shared<VtcRigidRepresentation>("Rigid Vtc");

	// Setup phase
	rigidBody->setIsActive(true);
	rigidBody->setIsGravityEnabled(true);
	rigidBody->setInitialParameters(m_param);
	rigidBody->setInitialState(m_stateDivergence);

	// Run 1 time step and make sure that the rigid body has been disabled
	// The rotation explode under the angular velocity too strong !
	{
		ASSERT_TRUE(rigidBody->isActive());

		rigidBody->beforeUpdate(m_dtDivergenceTest);
		rigidBody->update(m_dtDivergenceTest);
		rigidBody->afterUpdate(m_dtDivergenceTest);

		ASSERT_FALSE(rigidBody->isActive());
	}
}
