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

/// \file Fem2DRepresentationTests.cpp
/// This file tests the functionalities of the class Fem2DRepresentation.

#include <gtest/gtest.h>

#include "SurgSim/Framework/Runtime.h"
#include "SurgSim/Math/Matrix.h"
#include "SurgSim/Math/Quaternion.h"
#include "SurgSim/Math/RigidTransform.h"
#include "SurgSim/Math/Vector.h"
#include "SurgSim/Physics/Fem2DRepresentation.h"
#include "SurgSim/Physics/FemElement2DTriangle.h"

using SurgSim::Math::Matrix;
using SurgSim::Math::Vector;
using SurgSim::Math::Vector3d;

namespace
{

static const double dt = 1e-3;

}

namespace SurgSim
{

namespace Physics
{

class Fem2DRepresentationTests : public ::testing::Test
{
public:
	std::shared_ptr<Fem2DRepresentation> m_fem;

	SurgSim::Math::RigidTransform3d m_initialPose;
	std::shared_ptr<DeformableRepresentationState> m_initialState;

	Vector m_expectedTransformedPositions;
	Vector m_expectedTransformedVelocities;
	Vector m_expectedTransformedAccelerations;

private:
	// Physical properties
	double m_rho;
	double m_nu;
	double m_E;

	// Geometric properties
	double m_thickness;

protected:
	virtual void SetUp() override
	{
		using SurgSim::Math::getSubVector;

		// Physical properties
		m_rho = 2000.0;
		m_nu = 0.45;
		m_E = 1e6;

		// Geometric properties
		m_thickness = 1e-3;

		// Initial Pose
		SurgSim::Math::Quaterniond q(1.9, 4.2, 9.3, 2.1);
		q.normalize();
		Vector3d t(0.1, 0.2, 0.3);
		m_initialPose = SurgSim::Math::makeRigidTransform(q, t);

		// Define fem
		m_fem = std::make_shared<Fem2DRepresentation>("name");

		// Initial state
		m_initialState = std::make_shared<DeformableRepresentationState>();
		m_initialState->setNumDof(m_fem->getNumDofPerNode(), 3);

		Vector& x = m_initialState->getPositions();
		x << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
			 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
			 0.0, 1.0, 0.0, 0.0, 0.0, 0.0;
		Vector& v = m_initialState->getVelocities();
		v << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
			 2.0, 0.0, 0.0, 0.0, 0.0, 0.0,
			 3.0, 0.0, 0.0, 0.0, 0.0, 0.0;
		Vector& a = m_initialState->getAccelerations();
		a << 2.0, 0.0, 0.0, 0.0, 0.0, 0.0,
			 4.0, 0.0, 0.0, 0.0, 0.0, 0.0,
			 6.0, 0.0, 0.0, 0.0, 0.0, 0.0;

		// Expected transformed values
		m_expectedTransformedPositions.resize(m_initialState->getNumDof());
		m_expectedTransformedVelocities.resize(m_initialState->getNumDof());
		m_expectedTransformedAccelerations.resize(m_initialState->getNumDof());

		getSubVector(m_expectedTransformedPositions, 0, 3) = m_initialPose * Vector3d(getSubVector(x, 0, 3));
		getSubVector(m_expectedTransformedPositions, 1, 3) = getSubVector(x, 1, 3);
		getSubVector(m_expectedTransformedPositions, 2, 3) = m_initialPose * Vector3d(getSubVector(x, 2, 3));
		getSubVector(m_expectedTransformedPositions, 3, 3) = getSubVector(x, 3, 3);
		getSubVector(m_expectedTransformedPositions, 4, 3) = m_initialPose * Vector3d(getSubVector(x, 4, 3));
		getSubVector(m_expectedTransformedPositions, 5, 3) = getSubVector(x, 5, 3);

		getSubVector(m_expectedTransformedVelocities, 0, 3) = m_initialPose.linear() * getSubVector(v, 0, 3);
		getSubVector(m_expectedTransformedVelocities, 1, 3) = getSubVector(v, 1, 3);
		getSubVector(m_expectedTransformedVelocities, 2, 3) = m_initialPose.linear() * getSubVector(v, 2, 3);
		getSubVector(m_expectedTransformedVelocities, 3, 3) = getSubVector(v, 3, 3);
		getSubVector(m_expectedTransformedVelocities, 4, 3) = m_initialPose.linear() * getSubVector(v, 4, 3);
		getSubVector(m_expectedTransformedVelocities, 5, 3) = getSubVector(v, 5, 3);

		getSubVector(m_expectedTransformedAccelerations, 0, 3) = m_initialPose.linear() * getSubVector(a, 0, 3);
		getSubVector(m_expectedTransformedAccelerations, 1, 3) = getSubVector(a, 1, 3);
		getSubVector(m_expectedTransformedAccelerations, 2, 3) = m_initialPose.linear() * getSubVector(a, 2, 3);
		getSubVector(m_expectedTransformedAccelerations, 3, 3) = getSubVector(a, 3, 3);
		getSubVector(m_expectedTransformedAccelerations, 4, 3) = m_initialPose.linear() * getSubVector(a, 4, 3);
		getSubVector(m_expectedTransformedAccelerations, 5, 3) = getSubVector(a, 5, 3);

		// Create FemElement2DBeam
		std::array<unsigned int, 3> nodeIds = {0, 1, 2};
		auto element = std::make_shared<FemElement2DTriangle>(nodeIds);
		element->setMassDensity(m_rho);
		element->setYoungModulus(m_E);
		element->setPoissonRatio(m_nu);
		element->setThickness(m_thickness);

		// Add element to fem
		m_fem->addFemElement(element);
	}
};

TEST_F(Fem2DRepresentationTests, ConstructorTest)
{
	ASSERT_NO_THROW({Fem2DRepresentation("name");});
}

TEST_F(Fem2DRepresentationTests, GetTypeTest)
{
	EXPECT_EQ(REPRESENTATION_TYPE_FEM2D, m_fem->getType());
}

TEST_F(Fem2DRepresentationTests, TransformInitialStateTest)
{
	m_fem->setInitialPose(m_initialPose);
	m_fem->setInitialState(m_initialState);

	EXPECT_TRUE(m_fem->getInitialState()->getPositions().isApprox(m_expectedTransformedPositions));
	EXPECT_TRUE(m_fem->getInitialState()->getVelocities().isApprox(m_expectedTransformedVelocities));
	EXPECT_TRUE(m_fem->getInitialState()->getAccelerations().isApprox(m_expectedTransformedAccelerations));
}

TEST_F(Fem2DRepresentationTests, UpdateTest)
{
	// Need to call beforeUpdate() prior to calling update()
	// + Need to call setInitialState() prior to calling beforeUpdate()
	ASSERT_ANY_THROW(m_fem->update(dt));

	m_fem->setInitialState(m_initialState);
	m_fem->beforeUpdate(dt);
	// Need to call Initialize after addFemElement and setInitialState to initialize the mass information
	ASSERT_ANY_THROW(m_fem->update(dt));

	ASSERT_TRUE(m_fem->initialize(std::make_shared<SurgSim::Framework::Runtime>()));
	ASSERT_NO_THROW(m_fem->update(dt));

	// Previous and current state should contains the proper information
	// Note that the default integration scheme is Explicit Euler: x(t+dt) = x(t) + dt. v(t)
	// Note that the previous state should be the initial state, but current state should be different
	Vector expectCurrentPositions = m_fem->getPreviousState()->getPositions();
	expectCurrentPositions += dt * m_fem->getPreviousState()->getVelocities();
	EXPECT_TRUE(*m_fem->getPreviousState() == *m_fem->getInitialState());
	EXPECT_TRUE(*m_fem->getCurrentState() != *m_fem->getInitialState());
	EXPECT_TRUE(m_fem->getCurrentState()->getPositions().isApprox(expectCurrentPositions));
}

TEST_F(Fem2DRepresentationTests, AfterUpdateTest)
{
	// Need to call setInitialState() prior to calling afterUpdate()
	ASSERT_ANY_THROW(m_fem->afterUpdate(dt));

	m_fem->setInitialState(m_initialState);
	ASSERT_TRUE(m_fem->initialize(std::make_shared<SurgSim::Framework::Runtime>()));
	m_fem->beforeUpdate(dt);
	m_fem->update(dt);
	ASSERT_NO_THROW(m_fem->afterUpdate(dt));

	// Final and current state should contain the same information
	// Note that the previous state should be the initial state, but the current state should be different
	EXPECT_TRUE(*m_fem->getPreviousState() == *m_fem->getInitialState());
	EXPECT_TRUE(*m_fem->getCurrentState() != *m_fem->getInitialState());
	EXPECT_TRUE(*m_fem->getCurrentState() == *m_fem->getFinalState());
}

} // namespace Physics

} // namespace SurgSim
