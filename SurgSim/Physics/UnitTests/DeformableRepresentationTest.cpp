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

#include <string>

#include <gtest/gtest.h>

#include "SurgSim/Framework/Runtime.h"
#include "SurgSim/Math/Matrix.h"
#include "SurgSim/Math/Vector.h"
#include "SurgSim/Math/OdeSolver.h" // Need access to the enum IntegrationScheme
#include "SurgSim/Math/OdeSolverEulerExplicit.h"
#include "SurgSim/Math/OdeSolverEulerExplicitModified.h"
#include "SurgSim/Math/OdeSolverEulerImplicit.h"
#include "SurgSim/Math/OdeSolverLinearEulerExplicit.h"
#include "SurgSim/Math/OdeSolverLinearEulerExplicitModified.h"
#include "SurgSim/Math/OdeSolverLinearEulerImplicit.h"
#include "SurgSim/Math/OdeSolverLinearStatic.h"
#include "SurgSim/Math/OdeSolverStatic.h"
#include "SurgSim/Physics/DeformableCollisionRepresentation.h"
#include "SurgSim/Physics/DeformableRepresentation.h"
#include "SurgSim/Physics/DeformableRepresentationState.h"
#include "SurgSim/Physics/UnitTests/MockObjects.h"

using SurgSim::Physics::DeformableCollisionRepresentation;
using SurgSim::Physics::DeformableRepresentation;
using SurgSim::Physics::DeformableRepresentationState;
using SurgSim::Physics::MockDeformableRepresentation;

using SurgSim::Math::Vector3d;
using SurgSim::Math::Vector;
using SurgSim::Math::Matrix;

namespace
{
const unsigned int numNodes = 1;
const double epsilon = 1e-10;
}; // anonymous namespace

class DeformableRepresentationTest: public MockDeformableRepresentation, public ::testing::Test
{
public:
	DeformableRepresentationTest()
		: MockDeformableRepresentation()
	{
	}

	virtual ~DeformableRepresentationTest()
	{
	}

	/// Setup the test case
	void SetUp() override
	{
		m_localInitialState = std::make_shared<DeformableRepresentationState>();
		m_localInitialState->setNumDof(getNumDofPerNode(), numNodes);
		m_localInitialState->getPositions().setLinSpaced(0.0, static_cast<double>(getNumDofPerNode() * numNodes- 1));
		m_localInitialState->getVelocities().setOnes();

		SurgSim::Math::Quaterniond q(0.1, 0.4, 0.5, 0.2);
		q.normalize();
		Vector3d t(1.45, 5.4, 2.42);
		m_nonIdentityTransform = SurgSim::Math::makeRigidTransform(q, t);
		m_identityTransform = SurgSim::Math::RigidTransform3d::Identity();
	}

protected:
	// Initial state
	std::shared_ptr<DeformableRepresentationState> m_localInitialState;

	// Identity and nonIdentity (but still valid) transforms
	SurgSim::Math::RigidTransform3d m_identityTransform;
	SurgSim::Math::RigidTransform3d m_nonIdentityTransform;
};

TEST_F(DeformableRepresentationTest, ConstructorTest)
{
	// Test the constructor normally
	ASSERT_NO_THROW({MockDeformableRepresentation deformable;});

	// Test the object creation through the operator new
	ASSERT_NO_THROW({MockDeformableRepresentation* deformable = new MockDeformableRepresentation; delete deformable;});

	// Test the object creation through the operator new []
	ASSERT_NO_THROW({MockDeformableRepresentation* deformable = new MockDeformableRepresentation[10];\
		delete [] deformable;});

	// Test the object creation through a shared_ptr
	ASSERT_NO_THROW({std::shared_ptr<MockDeformableRepresentation> deformable =\
		std::make_shared<MockDeformableRepresentation>(); });
}

TEST_F(DeformableRepresentationTest, SetGetTest)
{
	// Test setLocalPose/getLocalPose
	setLocalPose(m_nonIdentityTransform);
	EXPECT_TRUE(getLocalPose().isApprox(m_nonIdentityTransform, epsilon));
	EXPECT_FALSE(getLocalPose().isApprox(m_identityTransform, epsilon));
	EXPECT_FALSE(getPose().isApprox(m_identityTransform, epsilon));

	setLocalPose(m_identityTransform);
	EXPECT_FALSE(getLocalPose().isApprox(m_nonIdentityTransform, epsilon));
	EXPECT_TRUE(getLocalPose().isApprox(m_identityTransform, epsilon));
	EXPECT_TRUE(getPose().isApprox(m_identityTransform, epsilon));

	// Test set/get states
	// Note that the initialState is in OdeEquation but is set in DeformableRepresentation
	// Its getter is actually in OdeEquation (considered tested here)
	setInitialState(m_localInitialState);
	doWakeUp();

	EXPECT_TRUE(*m_initialState     == *m_localInitialState);
	EXPECT_TRUE(*m_currentState     == *m_localInitialState);
	EXPECT_TRUE(*m_previousState    == *m_localInitialState);
	EXPECT_TRUE(*m_finalState       == *m_localInitialState);
	EXPECT_TRUE(*getInitialState()  == *m_localInitialState);
	EXPECT_TRUE(*getPreviousState() == *m_localInitialState);
	EXPECT_TRUE(*getCurrentState()  == *m_localInitialState);
	EXPECT_TRUE(*getFinalState()    == *m_localInitialState);

	// Test getNumDofPerNode
	EXPECT_EQ(3, getNumDofPerNode());

	// Test getNumDof (needs to be tested after setInitialState has been called)
	EXPECT_EQ(getNumDofPerNode() * numNodes, getNumDof());

	/// Set/Get the numerical integration scheme
	setIntegrationScheme(SurgSim::Math::INTEGRATIONSCHEME_EXPLICIT_EULER);
	EXPECT_EQ(SurgSim::Math::INTEGRATIONSCHEME_EXPLICIT_EULER, getIntegrationScheme());
	setIntegrationScheme(SurgSim::Math::INTEGRATIONSCHEME_MODIFIED_EXPLICIT_EULER);
	EXPECT_EQ(SurgSim::Math::INTEGRATIONSCHEME_MODIFIED_EXPLICIT_EULER, getIntegrationScheme());
	setIntegrationScheme(SurgSim::Math::INTEGRATIONSCHEME_IMPLICIT_EULER);
	EXPECT_EQ(SurgSim::Math::INTEGRATIONSCHEME_IMPLICIT_EULER, getIntegrationScheme());
	setIntegrationScheme(SurgSim::Math::INTEGRATIONSCHEME_STATIC);
	EXPECT_EQ(SurgSim::Math::INTEGRATIONSCHEME_STATIC, getIntegrationScheme());
	setIntegrationScheme(SurgSim::Math::INTEGRATIONSCHEME_RUNGE_KUTTA_4);
	EXPECT_EQ(SurgSim::Math::INTEGRATIONSCHEME_RUNGE_KUTTA_4, getIntegrationScheme());

	setIntegrationScheme(SurgSim::Math::INTEGRATIONSCHEME_LINEAR_EXPLICIT_EULER);
	EXPECT_EQ(SurgSim::Math::INTEGRATIONSCHEME_LINEAR_EXPLICIT_EULER, getIntegrationScheme());
	setIntegrationScheme(SurgSim::Math::INTEGRATIONSCHEME_LINEAR_MODIFIED_EXPLICIT_EULER);
	EXPECT_EQ(SurgSim::Math::INTEGRATIONSCHEME_LINEAR_MODIFIED_EXPLICIT_EULER, getIntegrationScheme());
	setIntegrationScheme(SurgSim::Math::INTEGRATIONSCHEME_LINEAR_IMPLICIT_EULER);
	EXPECT_EQ(SurgSim::Math::INTEGRATIONSCHEME_LINEAR_IMPLICIT_EULER, getIntegrationScheme());
	setIntegrationScheme(SurgSim::Math::INTEGRATIONSCHEME_LINEAR_STATIC);
	EXPECT_EQ(SurgSim::Math::INTEGRATIONSCHEME_LINEAR_STATIC, getIntegrationScheme());
	setIntegrationScheme(SurgSim::Math::INTEGRATIONSCHEME_LINEAR_RUNGE_KUTTA_4);
	EXPECT_EQ(SurgSim::Math::INTEGRATIONSCHEME_LINEAR_RUNGE_KUTTA_4, getIntegrationScheme());
}

TEST_F(DeformableRepresentationTest, GetComplianceMatrix)
{
	double dt = 1e-3;

	EXPECT_NO_THROW(setInitialState(m_localInitialState));
	EXPECT_NO_THROW(EXPECT_TRUE(initialize(std::make_shared<SurgSim::Framework::Runtime>())));
	EXPECT_NO_THROW(EXPECT_TRUE(wakeUp()));

	// This call solves the Ode equation and computes the compliance matrix using the default ode solver
	// Explicit euler => M.a(t+dt) = F(t) <=> M/dt.deltaV = F(t)
	// So the compliance matrix will be (M/dt)^-1
	// In our case, M = Identity, so the compliance matrix will be Identity*dt
	EXPECT_NO_THROW(update(dt));

	EXPECT_NO_THROW(EXPECT_TRUE(getComplianceMatrix().isApprox(Matrix::Identity(3,3) * dt)));
}

TEST_F(DeformableRepresentationTest, ResetStateTest)
{
	// setInitialState sets all 4 states (tested in method above !)
	setInitialState(m_localInitialState);

	// Initialize and wake-up the deformable component
	EXPECT_NO_THROW(EXPECT_TRUE(initialize(std::make_shared<SurgSim::Framework::Runtime>())));
	EXPECT_NO_THROW(EXPECT_TRUE(wakeUp()));

	// 1st time step
	beforeUpdate(1e-3);
	update(1e-3);
	afterUpdate(1e-3);

	// 2nd time step
	beforeUpdate(1e-3);
	update(1e-3);
	afterUpdate(1e-3);

	EXPECT_TRUE(*m_localInitialState == *m_initialState);
	EXPECT_FALSE(*m_localInitialState == *m_previousState);
	EXPECT_FALSE(*m_localInitialState == *m_currentState);
	EXPECT_FALSE(*m_localInitialState == *m_finalState);
	EXPECT_TRUE(*m_localInitialState == *getInitialState());
	EXPECT_FALSE(*m_localInitialState == *getPreviousState());
	EXPECT_FALSE(*m_localInitialState == *getCurrentState());
	EXPECT_FALSE(*m_localInitialState == *getFinalState());
	resetState();
	// reset should re-initialized current, previous and final to initial
	EXPECT_TRUE(*m_localInitialState == *m_initialState);
	EXPECT_TRUE(*m_localInitialState == *m_previousState);
	EXPECT_TRUE(*m_localInitialState == *m_currentState);
	EXPECT_TRUE(*m_localInitialState == *m_finalState);
	EXPECT_TRUE(*m_localInitialState == *getInitialState());
	EXPECT_TRUE(*m_localInitialState == *getPreviousState());
	EXPECT_TRUE(*m_localInitialState == *getCurrentState());
	EXPECT_TRUE(*m_localInitialState == *getFinalState());
}

TEST_F(DeformableRepresentationTest, UpdateTest)
{
	// update assert on m_odeSolver (wakeUp) and m_initialState (setInitialState)
	EXPECT_THROW(update(1e-3), SurgSim::Framework::AssertionFailure);

	// setInitialState sets all 4 states (tested in method above !)
	setInitialState(m_localInitialState);

	// update assert on m_odeSolver (wakeUp) and m_initialState (setInitialState)
	EXPECT_THROW(update(1e-3), SurgSim::Framework::AssertionFailure);

	// Initialize and wake-up the deformable component
	EXPECT_NO_THROW(EXPECT_TRUE(initialize(std::make_shared<SurgSim::Framework::Runtime>())));
	EXPECT_NO_THROW(EXPECT_TRUE(wakeUp()));

	// update should backup current into previous and change current
	EXPECT_NO_THROW(update(1e-3));

	EXPECT_TRUE(*m_localInitialState == *m_initialState);
	EXPECT_TRUE(*m_localInitialState == *m_previousState);
	EXPECT_FALSE(*m_localInitialState == *m_currentState);
	EXPECT_TRUE(*m_localInitialState == *m_finalState);
	EXPECT_TRUE(*m_localInitialState == *getInitialState());
	EXPECT_TRUE(*m_localInitialState == *getPreviousState());
	EXPECT_FALSE(*m_localInitialState == *getCurrentState());
	EXPECT_TRUE(*m_localInitialState == *getFinalState());
	EXPECT_FALSE(*m_previousState     == *m_currentState);
	EXPECT_FALSE(*getCurrentState()   == *getPreviousState());
}

TEST_F(DeformableRepresentationTest, AfterUpdateTest)
{
	// setInitialState sets all 4 states (tested in method above !)
	setInitialState(m_localInitialState);

	// Initialize and wake-up the deformable component
	EXPECT_NO_THROW(EXPECT_TRUE(initialize(std::make_shared<SurgSim::Framework::Runtime>())));
	EXPECT_NO_THROW(EXPECT_TRUE(wakeUp()));

	// update should backup current into previous and change current
	EXPECT_NO_THROW(update(1e-3));
	// afterUpdate should backup current into final
	EXPECT_NO_THROW(afterUpdate(1e-3));

	EXPECT_TRUE(*m_localInitialState  == *m_initialState);
	EXPECT_TRUE(*m_localInitialState  == *m_previousState);
	EXPECT_FALSE(*m_localInitialState == *m_currentState);
	EXPECT_FALSE(*m_localInitialState == *m_finalState);
	EXPECT_TRUE(*m_localInitialState  == *getInitialState());
	EXPECT_TRUE(*m_localInitialState  == *getPreviousState());
	EXPECT_FALSE(*m_localInitialState == *getCurrentState());
	EXPECT_FALSE(*m_localInitialState == *getFinalState());
	EXPECT_FALSE(*m_currentState      == *m_previousState);
	EXPECT_TRUE(*m_currentState       == *m_finalState);
	EXPECT_FALSE(*getCurrentState()   == *getPreviousState());
	EXPECT_TRUE(*getCurrentState()    == *getFinalState());
}

TEST_F(DeformableRepresentationTest, SetCollisionRepresentationTest)
{
	// setCollisionRepresentation requires the object to be a shared_ptr (using getShared())
	std::shared_ptr<MockDeformableRepresentation> object = std::make_shared<MockDeformableRepresentation>();

	auto collisionRep = std::make_shared<DeformableCollisionRepresentation>("DeformableCollisionRepresentation");
	EXPECT_NE(nullptr, collisionRep);

	// Test default collision representation to be nullptr
	EXPECT_EQ(nullptr, object->getCollisionRepresentation());

	// Test setting a valid non null collision rep
	EXPECT_NO_THROW(object->setCollisionRepresentation(collisionRep));
	EXPECT_EQ(collisionRep, object->getCollisionRepresentation());

	// Test setting a null collision rep
	EXPECT_NO_THROW(object->setCollisionRepresentation(nullptr));
	EXPECT_EQ(nullptr, object->getCollisionRepresentation());
}

TEST_F(DeformableRepresentationTest, DoWakeUpTest)
{
	using SurgSim::Math::OdeSolverEulerExplicit;
	using SurgSim::Math::OdeSolverEulerExplicitModified;
	using SurgSim::Math::OdeSolverEulerImplicit;
	using SurgSim::Math::OdeSolverStatic;
	using SurgSim::Math::OdeSolverLinearEulerExplicit;
	using SurgSim::Math::OdeSolverLinearEulerExplicitModified;
	using SurgSim::Math::OdeSolverLinearEulerImplicit;
	using SurgSim::Math::OdeSolverLinearStatic;
	using SurgSim::Math::LinearSolveAndInverseDenseMatrix;

	// setInitialState sets all 4 states (tested in method above !)
	setLocalPose(m_nonIdentityTransform);
	setInitialState(m_localInitialState);

	EXPECT_NO_THROW(EXPECT_TRUE(initialize(std::make_shared<SurgSim::Framework::Runtime>())));
	EXPECT_NO_THROW(EXPECT_TRUE(wakeUp()));

	// Test the initial transformation applied to all the states
	for (size_t nodeId = 0; nodeId < numNodes; nodeId++)
	{
		Vector3d expectedPosition = m_nonIdentityTransform * Vector3d::LinSpaced(nodeId * 3, (nodeId + 1) * 3 - 1);
		Vector3d expectedVelocity = m_nonIdentityTransform.rotation() * Vector3d::Ones();
		EXPECT_TRUE(getInitialState()->getPosition(nodeId).isApprox(expectedPosition));
		EXPECT_TRUE(getInitialState()->getVelocity(nodeId).isApprox(expectedVelocity));
	}
	EXPECT_EQ(*getPreviousState(), *getInitialState());
	EXPECT_EQ(*getCurrentState(), *getInitialState());
	EXPECT_EQ(*getFinalState(), *getInitialState());

	// Test the Ode Solver
	ASSERT_NE(nullptr, m_odeSolver);
	ASSERT_NE(nullptr, m_odeSolver->getLinearSolver());
	std::shared_ptr<LinearSolveAndInverseDenseMatrix> expectedLinearSolverType;
	expectedLinearSolverType = std::dynamic_pointer_cast<LinearSolveAndInverseDenseMatrix>
		(m_odeSolver->getLinearSolver());
	ASSERT_NE(nullptr, expectedLinearSolverType);

	typedef OdeSolverEulerExplicit<DeformableRepresentationState> EESolver;
	EESolver* explicitEuler;
	explicitEuler = dynamic_cast<EESolver*>(m_odeSolver.get());
	ASSERT_NE(nullptr, explicitEuler);

	typedef OdeSolverEulerExplicitModified<DeformableRepresentationState> MEESolver;
	MEESolver* modifiedExplicitEuler;
	modifiedExplicitEuler = dynamic_cast<MEESolver*>(m_odeSolver.get());
	ASSERT_EQ(nullptr, modifiedExplicitEuler);

	typedef OdeSolverEulerImplicit<DeformableRepresentationState> IESolver;
	IESolver* implicitEuler;
	implicitEuler = dynamic_cast<IESolver*>(m_odeSolver.get());
	ASSERT_EQ(nullptr, implicitEuler);

	typedef OdeSolverStatic<DeformableRepresentationState> StaticSolver;
	StaticSolver* staticSolver;
	staticSolver = dynamic_cast<StaticSolver*>(m_odeSolver.get());
	ASSERT_EQ(nullptr, staticSolver);

	typedef OdeSolverLinearEulerExplicit<DeformableRepresentationState> EELinearSolver;
	EELinearSolver* explicitEulerLinear;
	explicitEulerLinear = dynamic_cast<EELinearSolver*>(m_odeSolver.get());
	ASSERT_EQ(nullptr, explicitEulerLinear);

	typedef OdeSolverLinearEulerExplicitModified<DeformableRepresentationState>
		MEELinearSolver;
	MEELinearSolver* modifiedExplicitEulerLinear;
	modifiedExplicitEulerLinear = dynamic_cast<MEELinearSolver*>(m_odeSolver.get());
	ASSERT_EQ(nullptr, modifiedExplicitEulerLinear);

	typedef OdeSolverLinearEulerImplicit<DeformableRepresentationState> IELinearSolver;
	IELinearSolver* implicitEulerLinear;
	implicitEulerLinear = dynamic_cast<IELinearSolver*>(m_odeSolver.get());
	ASSERT_EQ(nullptr, implicitEulerLinear);

	typedef OdeSolverLinearStatic<DeformableRepresentationState> StaticLinearSolver;
	StaticLinearSolver* staticLinearSolver;
	staticLinearSolver = dynamic_cast<StaticLinearSolver*>(m_odeSolver.get());
	ASSERT_EQ(nullptr, staticLinearSolver);
}
