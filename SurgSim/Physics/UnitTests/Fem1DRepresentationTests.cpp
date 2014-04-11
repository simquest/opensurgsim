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

/// \file This file tests the functionalities of the class Fem1DRepresentation.

#include <gtest/gtest.h>

#include "SurgSim/Framework/BasicSceneElement.h"
#include "SurgSim/Framework/Runtime.h"
#include "SurgSim/Math/Matrix.h"
#include "SurgSim/Math/Quaternion.h"
#include "SurgSim/Math/RigidTransform.h"
#include "SurgSim/Math/Vector.h"
#include "SurgSim/Physics/Fem1DRepresentation.h"
#include "SurgSim/Physics/FemElement1DBeam.h"

using SurgSim::Framework::BasicSceneElement;
using SurgSim::Framework::Runtime;
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

class Fem1DBuilder
{
public:
	Fem1DBuilder()
		: massDensity(0.0),
		  youngModulus(0.0),
		  poissonRatio(0.0),
		  radius(0.0),
		  shearingEnabled(true),
		  initializeElements(false)
	{
	}

	std::shared_ptr<Fem1DRepresentation> build(const std::string& name)
	{
		unsigned int& nodes = nodesPerDimension[0];
		SURGSIM_ASSERT(nodes > 0) << "Number of nodes incorrect: " << nodes;

		auto fem = std::make_shared<Fem1DRepresentation>(name);
		auto state = std::make_shared<DeformableRepresentationState>();

		state->setNumDof(fem->getNumDofPerNode(), nodes);

		Vector3d delta = (extremities[1] - extremities[0]) / static_cast<double>(nodes - 1);

		for (unsigned int nodeId = 0; nodeId < nodes; nodeId++)
		{
			state->getPositions().segment<3>(6 * nodeId) = extremities[0] + nodeId * delta;
		}

		for (auto boundaryCondition = std::begin(boundaryConditions); boundaryCondition != std::end(boundaryConditions);
			 ++boundaryCondition)
		{
			state->addBoundaryCondition(*boundaryCondition);
		}

		std::array<unsigned int, 2> nodeEnds;

		for (unsigned int nodeId = 0; nodeId < nodes - 1; nodeId++)
		{
			nodeEnds[0] = nodeId;
			nodeEnds[1] = nodeId + 1;
			auto element = std::make_shared<FemElement1DBeam>(nodeEnds);
			element->setMassDensity(massDensity);
			element->setYoungModulus(youngModulus);
			element->setPoissonRatio(poissonRatio);
			element->setRadius(radius);
			element->setShearingEnabled(shearingEnabled);
			if (initializeElements)
			{
				element->initialize(*state);
			}
			fem->addFemElement(element);
		}

		fem->setInitialState(state);

		return fem;
	}

	void addBoundaryCondition(int node, int dof)
	{
		for (int i = 0; i < dof; i++)
		{
			boundaryConditions.push_back(node * 6 + i);
		}
	}

public:
	std::array<Vector3d, 2> extremities;
	std::vector<unsigned int> boundaryConditions;
	std::array<unsigned int, 1> nodesPerDimension;
	double massDensity;
	double youngModulus;
	double poissonRatio;
	double radius;
	bool shearingEnabled;
	bool initializeElements;
};

class Fem1DRepresentationTests : public ::testing::Test
{
public:
	std::shared_ptr<Fem1DRepresentation> m_fem;
	SurgSim::Math::RigidTransform3d m_initialPose;
	std::shared_ptr<DeformableRepresentationState> m_initialState;
	std::shared_ptr<BasicSceneElement> m_element;

	// Physical properties
	double m_rho;
	double m_nu;
	double m_E;

	// Geometric properties
	double m_radius;
	double m_L;
	double m_Iz;

	Vector m_expectedTransformedPositions;
	Vector m_expectedTransformedVelocities;
	Vector m_expectedTransformedAccelerations;

	Fem1DBuilder m_fem1DBuilder;

protected:
	virtual void SetUp() override
	{
		using SurgSim::Math::getSubVector;

		// Physical properties
		m_rho = 2000.0;
		m_nu = 0.45;
		m_E = 1e6;

		// Geometric properties
		m_radius = 0.05;
		m_L = 0.734511;
		m_Iz = M_PI / 4.0 * (m_radius * m_radius * m_radius * m_radius);

		// Initial Pose
		SurgSim::Math::Quaterniond q(1.9, 4.2, 9.3, 2.1);
		q.normalize();
		Vector3d t(0.1, 0.2, 0.3);
		m_initialPose = SurgSim::Math::makeRigidTransform(q, t);

		// Define fem
		m_fem = std::make_shared<Fem1DRepresentation>("name");

		// Initial state
		m_initialState = std::make_shared<DeformableRepresentationState>();
		m_initialState->setNumDof(m_fem->getNumDofPerNode(), 2);

		Vector& x = m_initialState->getPositions();
		x << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
			 m_L, 0.0, 0.0, 0.0, 0.0, 0.0;
		Vector& v = m_initialState->getVelocities();
		v << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
			 2.0, 0.0, 0.0, 0.0, 0.0, 0.0;
		Vector& a = m_initialState->getAccelerations();
		a << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
			 3.0, 0.0, 0.0, 0.0, 0.0, 0.0;

		// Expected transformed values
		m_expectedTransformedPositions.resize(m_initialState->getNumDof());
		m_expectedTransformedVelocities.resize(m_initialState->getNumDof());
		m_expectedTransformedAccelerations.resize(m_initialState->getNumDof());

		getSubVector(m_expectedTransformedPositions, 0, 3) = m_initialPose * Vector3d(getSubVector(x, 0, 3));
		getSubVector(m_expectedTransformedPositions, 1, 3) = getSubVector(x, 1, 3);
		getSubVector(m_expectedTransformedPositions, 2, 3) = m_initialPose * Vector3d(getSubVector(x, 2, 3));
		getSubVector(m_expectedTransformedPositions, 3, 3) = getSubVector(x, 3, 3);

		getSubVector(m_expectedTransformedVelocities, 0, 3) = m_initialPose.linear() * getSubVector(v, 0, 3);
		getSubVector(m_expectedTransformedVelocities, 1, 3) = getSubVector(v, 1, 3);
		getSubVector(m_expectedTransformedVelocities, 2, 3) = m_initialPose.linear() * getSubVector(v, 2, 3);
		getSubVector(m_expectedTransformedVelocities, 3, 3) = getSubVector(v, 3, 3);

		getSubVector(m_expectedTransformedAccelerations, 0, 3) = m_initialPose.linear() * getSubVector(a, 0, 3);
		getSubVector(m_expectedTransformedAccelerations, 1, 3) = getSubVector(a, 1, 3);
		getSubVector(m_expectedTransformedAccelerations, 2, 3) = m_initialPose.linear() * getSubVector(a, 2, 3);
		getSubVector(m_expectedTransformedAccelerations, 3, 3) = getSubVector(a, 3, 3);

		// Create FemElement1DBeam
		std::array<unsigned int, 2> nodeIds = {0, 1};
		auto element = std::make_shared<FemElement1DBeam>(nodeIds);
		element->setMassDensity(m_rho);
		element->setYoungModulus(m_E);
		element->setPoissonRatio(m_nu);
		element->setRadius(m_radius);

		// Add element to fem
		m_fem->addFemElement(element);

		// Setup m_fem1DBuilder
		m_fem1DBuilder.extremities[0] = Vector3d(0.0, 0.0, 0.0);
		m_fem1DBuilder.extremities[1] = Vector3d(m_L, 0.0, 0.0);
		m_fem1DBuilder.massDensity = m_rho;
		m_fem1DBuilder.youngModulus = m_E;
		m_fem1DBuilder.poissonRatio = m_nu;
		m_fem1DBuilder.radius = m_radius;
		m_fem1DBuilder.shearingEnabled = false;
		m_fem1DBuilder.initializeElements = true;

		m_element = std::make_shared<BasicSceneElement>("element");
		m_element->addComponent(m_fem);
	}
};

TEST_F(Fem1DRepresentationTests, ConstructorTest)
{
	ASSERT_NO_THROW(
		{ Fem1DRepresentation("name"); });
}

TEST_F(Fem1DRepresentationTests, GetTypeTest)
{
	EXPECT_EQ(REPRESENTATION_TYPE_FEM1D, m_fem->getType());
}

TEST_F(Fem1DRepresentationTests, TransformInitialStateTest)
{
	m_fem->setLocalPose(m_initialPose);
	m_fem->setInitialState(m_initialState);

	ASSERT_TRUE(m_fem->initialize(std::make_shared<Runtime>()));
	ASSERT_TRUE(m_fem->wakeUp());

	EXPECT_TRUE(m_fem->getInitialState()->getPositions().isApprox(m_expectedTransformedPositions));
	EXPECT_TRUE(m_fem->getInitialState()->getVelocities().isApprox(m_expectedTransformedVelocities));
	EXPECT_TRUE(m_fem->getInitialState()->getAccelerations().isApprox(m_expectedTransformedAccelerations));
}

TEST_F(Fem1DRepresentationTests, UpdateTest)
{
	// Need to call beforeUpdate() prior to calling update()
	// + Need to call setInitialState() prior to calling beforeUpdate()
	ASSERT_ANY_THROW(m_fem->update(dt));

	m_fem->setInitialState(m_initialState);
	// Need to call Initialize after addFemElement and setInitialState to initialize the mass information
	ASSERT_ANY_THROW(m_fem->update(dt));

	ASSERT_TRUE(m_fem->initialize(std::make_shared<Runtime>()));
	ASSERT_TRUE(m_fem->wakeUp());
	m_fem->beforeUpdate(dt);
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

TEST_F(Fem1DRepresentationTests, AfterUpdateTest)
{
	// Need to call setInitialState() prior to calling afterUpdate()
	ASSERT_ANY_THROW(m_fem->afterUpdate(dt));

	m_fem->setInitialState(m_initialState);
	ASSERT_TRUE(m_fem->initialize(std::make_shared<Runtime>()));
	ASSERT_TRUE(m_fem->wakeUp());
	m_fem->beforeUpdate(dt);
	m_fem->update(dt);
	ASSERT_NO_THROW(m_fem->afterUpdate(dt));

	// Final and current state should contain the same information
	// Note that the previous state should be the initial state, but the current state should be different
	EXPECT_TRUE(*m_fem->getPreviousState() == *m_fem->getInitialState());
	EXPECT_TRUE(*m_fem->getCurrentState() != *m_fem->getInitialState());
	EXPECT_TRUE(*m_fem->getCurrentState() == *m_fem->getFinalState());
}

// Beam tests
TEST_F(Fem1DRepresentationTests, CantileverEndLoadedTest)
{
	// Setup FEM
	unsigned int nodesPerDim = 2;

	m_fem1DBuilder.nodesPerDimension[0] = nodesPerDim;
	m_fem1DBuilder.addBoundaryCondition(0, 6);
	std::shared_ptr<Fem1DRepresentation> fem = m_fem1DBuilder.build("CantileverEndLoadedTest");

	// For last node, apply load to y-direction and calculate deflection
	double load = 0.7;
	unsigned int applyIndex = (nodesPerDim - 1) * 6 + 1;

	Vector applyForce = Vector::Zero(nodesPerDim * 6);
	applyForce[applyIndex] = load;
	Matrix stiffnessInverse = fem->computeK(*fem->getInitialState()).inverse();
	Vector calculatedDeflection = stiffnessInverse * applyForce;

	// Compare theoretical deflection with calculated deflection
	unsigned int lookIndex = applyIndex;

	double deflection = load * (m_L * m_L * m_L) / (3.0 * m_E * m_Iz);

	EXPECT_NEAR(deflection, calculatedDeflection[lookIndex], 1e-8);
}

TEST_F(Fem1DRepresentationTests, CantileverPunctualLoadAnywhereTest)
{
	// Setup FEM
	unsigned int nodesPerDim = 10;

	m_fem1DBuilder.nodesPerDimension[0] = nodesPerDim;
	m_fem1DBuilder.addBoundaryCondition(0, 6);
	std::shared_ptr<Fem1DRepresentation> fem = m_fem1DBuilder.build("CantileverPunctualLoadAnywhereTest");

	for (size_t applyNode = 1; applyNode < nodesPerDim; applyNode++)
	{
		// For each node, apply load to y-direction and calculate deflection
		double load = 0.7;
		unsigned int applyIndex = applyNode * 6 + 1;

		Vector applyForce = Vector::Zero(nodesPerDim * 6);
		applyForce[applyIndex] = load;
		Matrix stiffnessInverse = fem->computeK(*fem->getInitialState()).inverse();
		Vector calculatedDeflection = stiffnessInverse * applyForce;

		for (unsigned int lookNode = 0; lookNode < nodesPerDim; lookNode++)
		{
			// For each node, compare theoretical deflection with calculated deflection
			unsigned int lookIndex = lookNode * 6 + 1;

			double a = m_L * applyNode / (nodesPerDim - 1);
			double x = m_L * lookNode / (nodesPerDim - 1);
			double deflection = (x < a) ? load * x * x * (3 * a - x) / (6 * m_E * m_Iz)
										: load * a * a * (3 * x - a) / (6 * m_E * m_Iz);

			EXPECT_NEAR(deflection, calculatedDeflection[lookIndex], 1e-8);
		}
	}
}

TEST_F(Fem1DRepresentationTests, CantileverEndBentTest)
{
	// Setup FEM
	unsigned int nodesPerDim = 5;

	m_fem1DBuilder.nodesPerDimension[0] = nodesPerDim;
	m_fem1DBuilder.addBoundaryCondition(0, 6);
	std::shared_ptr<Fem1DRepresentation> fem = m_fem1DBuilder.build("CantileverEndBentTest");

	// For last node, apply moment to z-rotational direction and calculate deflection
	double moment = 0.7;
	unsigned int applyIndex = (nodesPerDim - 1) * 6 + 5;

	Vector applyForce = Vector::Zero(nodesPerDim * 6);
	applyForce[applyIndex] = moment;
	Matrix stiffnessInverse = fem->computeK(*fem->getInitialState()).inverse();
	Vector calculatedDeflection = stiffnessInverse * applyForce;

	for (unsigned int lookNode = 0; lookNode < nodesPerDim; lookNode++)
	{
		// For each node, compare theoretical deflection with calculated deflection
		unsigned int lookIndex = lookNode * 6 + 1;

		double x = m_L * lookNode / (nodesPerDim - 1);
		double deflection = moment * x * x / (2 * m_E * m_Iz);

		EXPECT_NEAR(deflection, calculatedDeflection[lookIndex], 1e-8);
	}
}

TEST_F(Fem1DRepresentationTests, EndSupportedBeamCenterLoadedTest)
{
	// Setup FEM
	unsigned int nodesPerDim = 5;

	m_fem1DBuilder.nodesPerDimension[0] = nodesPerDim;
	m_fem1DBuilder.addBoundaryCondition(0, 3);
	m_fem1DBuilder.addBoundaryCondition(nodesPerDim - 1, 3);
	std::shared_ptr<Fem1DRepresentation> fem = m_fem1DBuilder.build("EndSupportedBeamCenterLoadedTest");

	// For middle node, apply load to y-direction and calculate deflection
	double load = 0.7;
	unsigned int applyIndex = (nodesPerDim / 2) * 6 + 1;

	Vector applyForce = Vector::Zero(nodesPerDim * 6);
	applyForce[applyIndex] = load;
	Matrix stiffnessInverse = fem->computeK(*fem->getInitialState()).inverse();
	Vector calculatedDeflection = stiffnessInverse * applyForce;

	// Compare theoretical deflection with calculated deflection
	unsigned int lookIndex = applyIndex;

	double deflection = load * (m_L * m_L * m_L) / (48.0 * m_E * m_Iz);

	EXPECT_NEAR(deflection, calculatedDeflection[lookIndex], 1e-8);
}

TEST_F(Fem1DRepresentationTests, EndSupportedBeamIntermediatelyLoadedTest)
{
	// Setup FEM
	unsigned int nodesPerDim = 5;

	m_fem1DBuilder.nodesPerDimension[0] = nodesPerDim;
	m_fem1DBuilder.addBoundaryCondition(0, 3);
	m_fem1DBuilder.addBoundaryCondition(nodesPerDim - 1, 3);
	std::shared_ptr<Fem1DRepresentation> fem = m_fem1DBuilder.build("EndSupportedBeamIntermediatelyLoadedTest");

	for (unsigned int node = 0; node < nodesPerDim; node++)
	{
		// For each node, apply load to y-direction and calculate deflection
		double load = 0.7;
		unsigned int applyIndex = node * 6 + 1;

		Vector applyForce = Vector::Zero(nodesPerDim * 6);
		applyForce[applyIndex] = load;
		Matrix stiffnessInverse = fem->computeK(*fem->getInitialState()).inverse();
		Vector calculatedDeflection = stiffnessInverse * applyForce;

		// Compare theoretical deflection with calculated deflection
		unsigned int lookIndex = applyIndex;

		double a = static_cast<double>(node) / static_cast<double>(nodesPerDim - 1) * m_L;
		double b = m_L - a;
		double deflection = load * b * a / (6 * m_L * m_E * m_Iz) * (m_L * m_L - a * a - b * b);

		EXPECT_NEAR(deflection, calculatedDeflection[lookIndex], 1e-8);
	}
}

} // namespace Physics

} // namespace SurgSim
