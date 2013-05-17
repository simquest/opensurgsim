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

#include <SurgSim/Physics/Actors/RigidActor.h>
using namespace SurgSim::Physics;

#include <SurgSim/Math/Vector.h>
#include <SurgSim/Math/Matrix.h>
#include <SurgSim/Math/Quaternion.h>
#include <SurgSim/Math/RigidTransform.h>
using SurgSim::Math::Vector3d;
using SurgSim::Math::Matrix33d;
using SurgSim::Math::Quaterniond;
using SurgSim::Math::RigidTransform3d;

class RigidActorTest : public ::testing::Test
{
public:
	void SetUp()
	{
		m_dt = 1e-3;

		double radius = 0.1;
		m_param.setDensity(9000.0);
		m_param.setAngularDamping(0.0);
		m_param.setLinearDamping(0.0);
		m_param.setShapeUsedForMassInertia(std::make_shared<SphereShape>(radius));

		Quaterniond q(0.5, 0.4, 0.3, 0.2);
		q.normalize();
		Vector3d t(1.2, 2.1, 12.21);
		m_state.setAngularVelocity(Vector3d(1, 2, 3));
		m_state.setLinearVelocity(Vector3d(3, 2, 1));
		m_state.setPose(SurgSim::Math::makeRigidTransform(q,t ));

		// State to be used for divergence test
		m_stateDivergence.setAngularVelocity(Vector3d(DBL_MAX, DBL_MAX, DBL_MAX));

		m_maxNumSimulationStepTest = 100;
	}

	void TearDown()
	{
	}

	// Time step
	double m_dt;

	// Rigid actor parameters
	RigidActorParameters m_param;

	// Rigid actor default parameters
	RigidActorParameters m_defaultParameters;

	// Rigid actor state
	RigidActorState m_state;
	
	// Rigid actor state for divergence test
	RigidActorState m_stateDivergence;
	
	// Rigid actor default state
	RigidActorState m_defaultState;

	// Max number of simulation step for testing
	int m_maxNumSimulationStepTest;
};

TEST_F(RigidActorTest, ConstructorTest)
{
	ASSERT_NO_THROW( {RigidActor rigidBody("Rigid");});
}

TEST_F(RigidActorTest, ResetTest)
{
	// Create the rigid body
	std::shared_ptr<RigidActor> rigidBody = std::make_shared<RigidActor>("Rigid");

	rigidBody->setInitialParameters(m_defaultParameters);
	rigidBody->setCurrentParameters(m_param);
	rigidBody->setInitialState(m_state);
	rigidBody->setIsActive(false);
	rigidBody->setIsGravityEnabled(false);
	rigidBody->setPose(RigidTransform3d::Identity());

	// reset the actor state
	rigidBody->resetState();

	// Parameters unchanged
	EXPECT_EQ(m_param, rigidBody->getCurrentParameters());
	// isActive unchanged
	EXPECT_FALSE(rigidBody->isActive());
	// isGravityEnable flag unchanged
	EXPECT_FALSE(rigidBody->isGravityEnabled());
	// current state = initial state
	EXPECT_EQ(rigidBody->getInitialState(), rigidBody->getCurrentState());
	// previous state = initial state
	EXPECT_EQ(rigidBody->getInitialState(), rigidBody->getPreviousState());

	// reset the actor parameters
	rigidBody->resetParameters();

	// Parameters reset to initial
	EXPECT_EQ(rigidBody->getInitialParameters(), rigidBody->getCurrentParameters());
	EXPECT_EQ(m_defaultParameters, rigidBody->getCurrentParameters());
}

TEST_F(RigidActorTest, SetGetAndDefaultValueTest)
{
	// Create the rigid body
	std::shared_ptr<RigidActor> rigidBody = std::make_shared<RigidActor>("Rigid");

	// Get state (current, initial)
	EXPECT_EQ(m_defaultState, rigidBody->getCurrentState());
	EXPECT_EQ(m_defaultState, rigidBody->getPreviousState());
	EXPECT_EQ(m_defaultState, rigidBody->getInitialState());
	rigidBody->setInitialState(m_state);
	EXPECT_EQ(m_state, rigidBody->getInitialState());
	EXPECT_EQ(m_state, rigidBody->getCurrentState());
	EXPECT_EQ(m_state, rigidBody->getPreviousState());

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
	ASSERT_EQ(6, rigidBody->getNumDof());

	// Set/Get isGravityEnabled [default = true]
	EXPECT_TRUE(rigidBody->isGravityEnabled());
	rigidBody->setIsGravityEnabled(false);
	ASSERT_FALSE(rigidBody->isGravityEnabled());
	rigidBody->setIsGravityEnabled(true);
	ASSERT_TRUE(rigidBody->isGravityEnabled());
}

TEST_F(RigidActorTest, NoForceTorqueTest)
{
	// Create the rigid body
	std::shared_ptr<RigidActor> rigidBody = std::make_shared<RigidActor>("Rigid");

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

	const Vector3d    G = rigidBody->getCurrentState().getPose().translation();
	const Matrix33d&  R = rigidBody->getCurrentState().getPose().rotation();
	const Quaterniond q = Quaterniond(R);
	const Vector3d    v = rigidBody->getCurrentState().getLinearVelocity();
	const Vector3d    w = rigidBody->getCurrentState().getAngularVelocity();
	ASSERT_EQ(Vector3d::Zero(), G);
	ASSERT_TRUE(q.isApprox(Quaterniond::Identity()));
	ASSERT_EQ(Vector3d::Zero(), v);
	ASSERT_EQ(Vector3d::Zero(), w);
}

TEST_F(RigidActorTest, GravityTest)
{
	// Create the rigid body
	std::shared_ptr<RigidActor> rigidBody = std::make_shared<RigidActor>("Rigid");
	Vector3d gravity(0.0, -9.81, 0.0);

	// Setup phase
	rigidBody->setIsActive(true);
	rigidBody->setIsGravityEnabled(true);
	rigidBody->setInitialParameters(m_param);

	// Run few time steps
	for (int timeStep = 0; timeStep < m_maxNumSimulationStepTest; timeStep++)
	{
		rigidBody->beforeUpdate(m_dt);
		rigidBody->update(m_dt);
		rigidBody->afterUpdate(m_dt);

		const Vector3d    G = rigidBody->getCurrentState().getPose().translation();
		const Matrix33d&  R = rigidBody->getCurrentState().getPose().rotation();
		const Quaterniond q = Quaterniond(R);
		const Vector3d    v = rigidBody->getCurrentState().getLinearVelocity();
		const Vector3d    w = rigidBody->getCurrentState().getAngularVelocity();

		const Vector3d    Gprev = rigidBody->getPreviousState().getPose().translation();
		const Matrix33d&  Rprev = rigidBody->getPreviousState().getPose().rotation();
		const Quaterniond qprev = Quaterniond(Rprev);
		const Vector3d    vprev = rigidBody->getPreviousState().getLinearVelocity();
		const Vector3d    wprev = rigidBody->getPreviousState().getAngularVelocity();

		// Implicit numerical integration gives v(t+dt) = v(t) + dt.a(t+dt) = v(t) + dt.g
		//                                      p(t+dt) = p(t) + dt.v(t+dt) = p(t) + dt.v(t) + dt^2.g
		Vector3d tmpV = vprev + gravity * m_dt;
		double diffV = (v-tmpV).norm();
		Vector3d tmpG = Gprev + tmpV * m_dt;
		double diffG = (G-tmpG).norm();

		double epsilon = 1e-15;
		ASSERT_NEAR(0.0, diffG, epsilon);
		ASSERT_TRUE(q.isApprox(Quaterniond::Identity()));
		ASSERT_NEAR(0.0, diffV, epsilon);
		ASSERT_EQ(Vector3d::Zero(), w);
	}
}

TEST_F(RigidActorTest, PreviousStateDifferentFromCurrentTest)
{
	// Create the rigid body
	std::shared_ptr<RigidActor> rigidBody = std::make_shared<RigidActor>("Rigid");
	Vector3d gravity(0.0, -9.81, 0.0);

	// Setup phase
	rigidBody->setIsActive(true);
	rigidBody->setIsGravityEnabled(true);
	rigidBody->setInitialParameters(m_param);

	// Run few time steps
	for (int timeStep = 0; timeStep < m_maxNumSimulationStepTest; timeStep++)
	{
		rigidBody->beforeUpdate(m_dt);
		rigidBody->update(m_dt);
		rigidBody->afterUpdate(m_dt);

		ASSERT_NE(rigidBody->getPreviousState(), rigidBody->getCurrentState());
	}
}

TEST_F(RigidActorTest, DisableWhenDivergeTest)
{
	// Create the rigid body
	std::shared_ptr<RigidActor> rigidBody = std::make_shared<RigidActor>("Rigid");

	// Setup phase
	rigidBody->setIsActive(true);
	rigidBody->setIsGravityEnabled(true);
	rigidBody->setInitialParameters(m_param);
	rigidBody->setInitialState(m_stateDivergence);

	// Run 1 time step and make sure that the rigid body has been disabled
	// The rotation explode under the angular velocity too strong !
	{
		ASSERT_TRUE(rigidBody->isActive());
		
		rigidBody->beforeUpdate(m_dt);
		rigidBody->update(m_dt);
		rigidBody->afterUpdate(m_dt);

		ASSERT_FALSE(rigidBody->isActive());
	}
}
