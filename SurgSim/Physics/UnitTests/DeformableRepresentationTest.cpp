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

#include <string>

#include "SurgSim/Physics/DeformableRepresentation.h"
#include "SurgSim/Physics/DeformableRepresentationState.h"
#include "SurgSim/Math/Vector.h"
#include "SurgSim/Math/Matrix.h"
#include "SurgSim/Math/OdeSolver.h" // Need access to the enum IntegrationScheme
#include "SurgSim/Math/OdeSolverEulerExplicit.h"
#include "SurgSim/Math/OdeSolverEulerExplicitModified.h"
#include "SurgSim/Math/OdeSolverEulerImplicit.h"
#include "SurgSim/Math/OdeSolverStatic.h"
#include "SurgSim/Math/OdeSolverLinearEulerExplicit.h"
#include "SurgSim/Math/OdeSolverLinearEulerExplicitModified.h"
#include "SurgSim/Math/OdeSolverLinearEulerImplicit.h"
#include "SurgSim/Math/OdeSolverLinearStatic.h"

using SurgSim::Physics::DeformableRepresentation;
using SurgSim::Physics::DeformableRepresentationState;

using SurgSim::Math::Vector3d;
using SurgSim::Math::Vector;
using SurgSim::Math::Matrix;

namespace
{
const unsigned int numNodes = 100;
const unsigned int numDofPerNode = 3;
const unsigned int numDof = numDofPerNode* numNodes;
const double epsilon = 1e-10;
};

class MockObject : public DeformableRepresentation<Matrix, Matrix, Matrix, Matrix>
{
public:
	MockObject()
		: DeformableRepresentation<Matrix, Matrix, Matrix, Matrix>("MockObject")
	{
		this->m_numDofPerNode = numDofPerNode;
	}

	virtual ~MockObject()
	{
	}

	/// Query the representation type
	/// \return the RepresentationType for this representation
	/// \note DeformableRepresentation is abstract because there is really no deformable behind this class !
	/// \note For the test, we simply set the type to INVALID
	virtual SurgSim::Physics::RepresentationType getType() const override
	{
		return SurgSim::Physics::REPRESENTATION_TYPE_INVALID;
	}

	/// before update method
	/// \param dt The time step for the current update
	virtual void beforeUpdate(double dt) override
	{
		DeformableRepresentation::beforeUpdate(dt);

		// Backup the current state into the previous state
		*m_previousState = *m_currentState;
	}

	/// update method
	/// \param dt The time step for the current update
	virtual void update(double dt) override
	{
		// Update the current state with something... (+=)
		for (unsigned int i = 0; i < m_currentState->getNumDof(); i++)
		{
			m_currentState->getVelocities()[i] += static_cast<double>(i);
			m_currentState->getPositions()[i]  += m_currentState->getVelocities()[i] * dt;
		}
	}

	/// after update method
	/// \param dt The time step for the current update
	virtual void afterUpdate(double dt) override
	{
		// Backup the current state into the final state
		*m_finalState = *m_currentState;
	}

	/// OdeEquation API (empty) is not tested here as DeformableRep does not provide an implementation
	/// This API will be tested in derived classes when the API will be provided
	Vector& computeF(const DeformableRepresentationState& state) override
	{
		static Vector F;
		return F;
	}

	/// OdeEquation API (empty) is not tested here as DeformableRep does not provide an implementation
	/// This API will be tested in derived classes when the API will be provided
	const Matrix& computeM(const DeformableRepresentationState& state) override
	{
		static Matrix M;
		return M;
	}

	/// OdeEquation API (empty) is not tested here as DeformableRep does not provide an implementation
	/// This API will be tested in derived classes when the API will be provided
	const Matrix& computeD(const DeformableRepresentationState& state) override
	{
		static Matrix D;
		return D;
	}

	/// OdeEquation API (empty) is not tested here as DeformableRep does not provide an implementation
	/// This API will be tested in derived classes when the API will be provided
	const Matrix& computeK(const DeformableRepresentationState& state) override
	{
		static Matrix K;
		return K;
	}

	/// OdeEquation API (empty) is not tested here as DeformableRep does not provide an implementation
	/// This API will be tested in derived classes when the API will be provided
	void computeFMDK(const DeformableRepresentationState& state,
					 Vector** f, Matrix** M, Matrix** D, Matrix** K) override
	{
	}

protected:
	void transformState(std::shared_ptr<DeformableRepresentationState> state,
						const SurgSim::Math::RigidTransform3d& transform) override
	{
		using SurgSim::Math::setSubVector;
		using SurgSim::Math::getSubVector;

		Vector& x = state->getPositions();
		for (unsigned int nodeId = 0; nodeId < numNodes; nodeId++)
		{
			Vector3d xi = getSubVector(x, nodeId, 3);
			Vector3d xiTransformed = transform * xi;
			setSubVector(xiTransformed, nodeId, 3, &x);
		}
	}
};

class DeformableRepresentationTest: public MockObject, public ::testing::Test
{
public:
	DeformableRepresentationTest()
		: MockObject()
	{
	}

	virtual ~DeformableRepresentationTest()
	{
	}

	/// Setup the test case
	void SetUp() override
	{
		m_localInitialState = std::make_shared<DeformableRepresentationState>();
		m_localInitialState->setNumDof(numDofPerNode, numNodes);
		for (unsigned int i = 0; i < numDof; i++)
		{
			m_localInitialState->getPositions()[i] = static_cast<double>(i);
			m_localInitialState->getVelocities()[i] = 1.0;
		}

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
	ASSERT_NO_THROW({MockObject deformable;});

	// Test the object creation through the operator new
	ASSERT_NO_THROW({MockObject* deformable = new MockObject; delete deformable;});

	// Test the object creation through the operator new []
	ASSERT_NO_THROW({MockObject* deformable = new MockObject[10]; delete [] deformable;});

	// Test the object creation through a shared_ptr
	ASSERT_NO_THROW({std::shared_ptr<MockObject> deformable = std::make_shared<MockObject>(); });
}

TEST_F(DeformableRepresentationTest, SetGetTest)
{
	// Test setInitialPose/getInitialPose
	setInitialPose(m_nonIdentityTransform);
	EXPECT_TRUE(getInitialPose().isApprox(m_nonIdentityTransform, epsilon));
	EXPECT_FALSE(getInitialPose().isApprox(m_identityTransform, epsilon));
	setInitialPose(m_identityTransform);
	EXPECT_FALSE(getInitialPose().isApprox(m_nonIdentityTransform, epsilon));
	EXPECT_TRUE(getInitialPose().isApprox(m_identityTransform, epsilon));

	// Test setPose/getPose (always return Identity)
	EXPECT_THROW(setPose(m_nonIdentityTransform), SurgSim::Framework::AssertionFailure);
	EXPECT_THROW(setPose(m_identityTransform), SurgSim::Framework::AssertionFailure);
	EXPECT_TRUE(getPose().isApprox(m_identityTransform, epsilon));

	// Test set/get states
	// Note that the initialState is in OdeEquation but is set in DeformableRepresentation
	// Its getter is actually in OdeEquation (considered tested here)
	setInitialState(m_localInitialState);
	EXPECT_TRUE(*m_initialState     == *m_localInitialState);
	EXPECT_TRUE(*m_currentState     == *m_localInitialState);
	EXPECT_TRUE(*m_previousState    == *m_localInitialState);
	EXPECT_TRUE(*m_finalState       == *m_localInitialState);
	EXPECT_TRUE(*getInitialState()  == *m_localInitialState);
	EXPECT_TRUE(*getPreviousState() == *m_localInitialState);
	EXPECT_TRUE(*getCurrentState()  == *m_localInitialState);
	EXPECT_TRUE(*getFinalState()    == *m_localInitialState);

	// Test getNumDof (needs to be tested after setInitialState has been called)
	EXPECT_EQ(numDof, getNumDof());

	// Test getNumDofPerNode
	EXPECT_EQ(numDofPerNode, getNumDofPerNode());

	/// Set/Get the numerical integration scheme
	setIntegrationScheme(SurgSim::Math::INTEGRATIONSCHEME_EXPLICIT_EULER);
	EXPECT_EQ(SurgSim::Math::INTEGRATIONSCHEME_EXPLICIT_EULER, getIntegrationScheme());
	setIntegrationScheme(SurgSim::Math::INTEGRATIONSCHEME_MODIFIED_EXPLICIT_EULER);
	EXPECT_EQ(SurgSim::Math::INTEGRATIONSCHEME_MODIFIED_EXPLICIT_EULER, getIntegrationScheme());
	setIntegrationScheme(SurgSim::Math::INTEGRATIONSCHEME_IMPLICIT_EULER);
	EXPECT_EQ(SurgSim::Math::INTEGRATIONSCHEME_IMPLICIT_EULER, getIntegrationScheme());
}

TEST_F(DeformableRepresentationTest, BeforeUpdateInitializesOdeSolverTest)
{
	using SurgSim::Math::OdeSolverEulerExplicit;
	using SurgSim::Math::OdeSolverEulerExplicitModified;
	using SurgSim::Math::OdeSolverEulerImplicit;
	using SurgSim::Math::OdeSolverStatic;
	using SurgSim::Math::OdeSolverLinearEulerExplicit;
	using SurgSim::Math::OdeSolverLinearEulerExplicitModified;
	using SurgSim::Math::OdeSolverLinearEulerImplicit;
	using SurgSim::Math::OdeSolverLinearStatic;

	// setInitialState sets all 4 states (tested in method above !)
	setInitialState(m_localInitialState);

	// beforeUpdate should initialize the odeSolver with the default integration scheme (Euler Explicit)
	beforeUpdate(1e-3);
	ASSERT_NE(nullptr, m_odeSolver);

	typedef OdeSolverEulerExplicit<DeformableRepresentationState, Matrix, Matrix, Matrix, Matrix> EESolver;
	EESolver* explicitEuler;
	explicitEuler = dynamic_cast<EESolver*>(m_odeSolver.get());
	ASSERT_NE(nullptr, explicitEuler);

	typedef OdeSolverEulerExplicitModified<DeformableRepresentationState, Matrix, Matrix, Matrix, Matrix> MEESolver;
	MEESolver* modifiedExplicitEuler;
	modifiedExplicitEuler = dynamic_cast<MEESolver*>(m_odeSolver.get());
	ASSERT_EQ(nullptr, modifiedExplicitEuler);

	typedef OdeSolverEulerImplicit<DeformableRepresentationState, Matrix, Matrix, Matrix, Matrix> IESolver;
	IESolver* implicitEuler;
	implicitEuler = dynamic_cast<IESolver*>(m_odeSolver.get());
	ASSERT_EQ(nullptr, implicitEuler);

	typedef OdeSolverStatic<DeformableRepresentationState, Matrix, Matrix, Matrix, Matrix> StaticSolver;
	StaticSolver* staticSolver;
	staticSolver = dynamic_cast<StaticSolver*>(m_odeSolver.get());
	ASSERT_EQ(nullptr, staticSolver);

	typedef OdeSolverLinearEulerExplicit<DeformableRepresentationState, Matrix, Matrix, Matrix, Matrix> EELinearSolver;
	EELinearSolver* explicitEulerLinear;
	explicitEulerLinear = dynamic_cast<EELinearSolver*>(m_odeSolver.get());
	ASSERT_EQ(nullptr, explicitEulerLinear);

	typedef OdeSolverLinearEulerExplicitModified<DeformableRepresentationState, Matrix, Matrix, Matrix, Matrix>
		MEELinearSolver;
	MEELinearSolver* modifiedExplicitEulerLinear;
	modifiedExplicitEulerLinear = dynamic_cast<MEELinearSolver*>(m_odeSolver.get());
	ASSERT_EQ(nullptr, modifiedExplicitEulerLinear);

	typedef OdeSolverLinearEulerImplicit<DeformableRepresentationState, Matrix, Matrix, Matrix, Matrix> IELinearSolver;
	IELinearSolver* implicitEulerLinear;
	implicitEulerLinear = dynamic_cast<IELinearSolver*>(m_odeSolver.get());
	ASSERT_EQ(nullptr, implicitEulerLinear);

	typedef OdeSolverLinearStatic<DeformableRepresentationState, Matrix, Matrix, Matrix, Matrix> StaticLinearSolver;
	StaticLinearSolver* staticLinearSolver;
	staticLinearSolver = dynamic_cast<StaticLinearSolver*>(m_odeSolver.get());
	ASSERT_EQ(nullptr, staticLinearSolver);
}

TEST_F(DeformableRepresentationTest, UpdateChangesStateTest)
{
	// setInitialState sets all 4 states (tested in method above !)
	setInitialState(m_localInitialState);

	// beforeUpdate should backup current (=initial) into previous
	beforeUpdate(1e-3);
	// update should change current
	update(1e-3);
	// afterUpdate should backup current into final
	afterUpdate(1e-3);
	EXPECT_TRUE(*m_localInitialState == *m_initialState);
	EXPECT_TRUE(*m_localInitialState == *m_previousState);
	EXPECT_FALSE(*m_localInitialState == *m_currentState);
	EXPECT_FALSE(*m_localInitialState == *m_finalState);
	EXPECT_TRUE(*m_localInitialState == *getInitialState());
	EXPECT_TRUE(*m_localInitialState == *getPreviousState());
	EXPECT_FALSE(*m_localInitialState == *getCurrentState());
	EXPECT_FALSE(*m_localInitialState == *getFinalState());
	EXPECT_FALSE(*m_previousState     == *m_currentState);
	EXPECT_FALSE(*getCurrentState()   == *getPreviousState());

	// beforeUpdate should backup current (!=initial) into previous
	beforeUpdate(1e-3);
	// update should change current
	update(1e-3);
	// afterUpdate should backup current into final
	afterUpdate(1e-3);
	EXPECT_TRUE(*m_localInitialState == *m_initialState);
	EXPECT_FALSE(*m_localInitialState == *m_previousState);
	EXPECT_FALSE(*m_localInitialState == *m_currentState);
	EXPECT_FALSE(*m_localInitialState == *m_finalState);
	EXPECT_TRUE(*m_localInitialState == *getInitialState());
	EXPECT_FALSE(*m_localInitialState == *getPreviousState());
	EXPECT_FALSE(*m_localInitialState == *getCurrentState());
	EXPECT_FALSE(*m_localInitialState == *getFinalState());
	EXPECT_FALSE(*m_previousState     == *m_currentState);
	EXPECT_FALSE(*getCurrentState()   == *getPreviousState());
}

TEST_F(DeformableRepresentationTest, ResetStateTest)
{
	// setInitialState sets all 4 states (tested in method above !)
	setInitialState(m_localInitialState);

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
