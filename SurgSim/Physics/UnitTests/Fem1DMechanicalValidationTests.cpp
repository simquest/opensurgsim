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

/// \file Fem1DMechanicalValidationTests.cpp
/// This file tests the mechanical behavior of the class Fem1DRepresentation.

#include <gtest/gtest.h>

#include "SurgSim/Framework/Runtime.h"
#include "SurgSim/Math/Matrix.h"
#include "SurgSim/Math/OdeState.h"
#include "SurgSim/Math/Quaternion.h"
#include "SurgSim/Math/RigidTransform.h"
#include "SurgSim/Math/Vector.h"
#include "SurgSim/Physics/Fem1DRepresentation.h"
#include "SurgSim/Physics/Fem1DElementBeam.h"
#include "SurgSim/Physics/UnitTests/MockObjects.h"

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
		size_t& nodes = nodesPerDimension[0];
		SURGSIM_ASSERT(nodes > 0) << "Number of nodes incorrect: " << nodes;

		auto fem = std::make_shared<MockFem1DRepresentation>(name);
		auto state = std::make_shared<SurgSim::Math::OdeState>();

		state->setNumDof(fem->getNumDofPerNode(), nodes);

		Vector3d delta = (extremities[1] - extremities[0]) / static_cast<double>(nodes - 1);

		for (size_t nodeId = 0; nodeId < nodes; nodeId++)
		{
			state->getPositions().segment<3>(6 * nodeId) = extremities[0] + static_cast<double>(nodeId) * delta;
		}

		for (auto boundaryCondition = std::begin(boundaryConditions); boundaryCondition != std::end(boundaryConditions);
			 ++boundaryCondition)
		{
			state->addBoundaryCondition(boundaryCondition->first, boundaryCondition->second);
		}

		std::array<size_t, 2> nodeEnds;

		for (size_t nodeId = 0; nodeId < nodes - 1; nodeId++)
		{
			nodeEnds[0] = nodeId;
			nodeEnds[1] = nodeId + 1;
			auto element = std::make_shared<Fem1DElementBeam>(nodeEnds);
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
		fem->doInitialize();

		return fem;
	}

	void addBoundaryCondition(size_t node, size_t dof)
	{
		for (size_t i = 0; i < dof; i++)
		{
			boundaryConditions.push_back(std::make_pair(node, i));
		}
	}

public:
	std::array<Vector3d, 2> extremities;
	std::vector<std::pair<size_t, size_t>> boundaryConditions; // <nodeId, dofId>
	std::array<size_t, 1> nodesPerDimension;
	double massDensity;
	double youngModulus;
	double poissonRatio;
	double radius;
	bool shearingEnabled;
	bool initializeElements;
};

class Fem1DMechanicalValidationTests : public ::testing::Test
{
public:
	std::shared_ptr<Fem1DRepresentation> m_fem;
	SurgSim::Math::RigidTransform3d m_initialPose;
	std::shared_ptr<SurgSim::Math::OdeState> m_initialState;

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

	Fem1DBuilder m_fem1DBuilder;

protected:
	void SetUp() override
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
		m_initialState = std::make_shared<SurgSim::Math::OdeState>();
		m_initialState->setNumDof(m_fem->getNumDofPerNode(), 2);

		Vector& x = m_initialState->getPositions();
		x << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
		m_L, 0.0, 0.0, 0.0, 0.0, 0.0;
		Vector& v = m_initialState->getVelocities();
		v << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
		2.0, 0.0, 0.0, 0.0, 0.0, 0.0;

		// Expected transformed values
		m_expectedTransformedPositions.resize(m_initialState->getNumDof());
		m_expectedTransformedVelocities.resize(m_initialState->getNumDof());

		getSubVector(m_expectedTransformedPositions, 0, 3) = m_initialPose * Vector3d(getSubVector(x, 0, 3));
		getSubVector(m_expectedTransformedPositions, 1, 3) = getSubVector(x, 1, 3);
		getSubVector(m_expectedTransformedPositions, 2, 3) = m_initialPose * Vector3d(getSubVector(x, 2, 3));
		getSubVector(m_expectedTransformedPositions, 3, 3) = getSubVector(x, 3, 3);

		getSubVector(m_expectedTransformedVelocities, 0, 3) = m_initialPose.linear() * getSubVector(v, 0, 3);
		getSubVector(m_expectedTransformedVelocities, 1, 3) = getSubVector(v, 1, 3);
		getSubVector(m_expectedTransformedVelocities, 2, 3) = m_initialPose.linear() * getSubVector(v, 2, 3);
		getSubVector(m_expectedTransformedVelocities, 3, 3) = getSubVector(v, 3, 3);

		// Create Fem1DElementBeam
		std::array<size_t, 2> nodeIds = {0, 1};
		auto element = std::make_shared<Fem1DElementBeam>(nodeIds);
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
	}
};

// Beam tests
TEST_F(Fem1DMechanicalValidationTests, CantileverEndLoadedTest)
{
	// Setup FEM
	size_t nodesPerDim = 2;

	m_fem1DBuilder.nodesPerDimension[0] = nodesPerDim;
	m_fem1DBuilder.addBoundaryCondition(0, 6);
	std::shared_ptr<Fem1DRepresentation> fem = m_fem1DBuilder.build("CantileverEndLoadedTest");

	// For last node, apply load to y-direction and calculate deflection
	double load = 0.7;
	size_t applyIndex = (nodesPerDim - 1) * 6 + 1;

	Vector applyForce = Vector::Zero(nodesPerDim * 6);
	applyForce[applyIndex] = load;
	fem->update(*(m_fem->getInitialState()), Math::ODEEQUATIONUPDATE_K);
	Matrix stiffness = fem->getK();

	// Apply boundary conditions
	fem->getInitialState()->applyBoundaryConditionsToVector(&applyForce);
	fem->getInitialState()->applyBoundaryConditionsToMatrix(&stiffness);

	Vector calculatedDeflection = stiffness.inverse() * applyForce;

	// Compare theoretical deflection with calculated deflection
	size_t lookIndex = applyIndex;

	double deflection = load * (m_L * m_L * m_L) / (3.0 * m_E * m_Iz);

	EXPECT_NEAR(deflection, calculatedDeflection[lookIndex], 1e-8);
}

TEST_F(Fem1DMechanicalValidationTests, CantileverPunctualLoadAnywhereTest)
{
	// Setup FEM
	size_t nodesPerDim = 10;

	m_fem1DBuilder.nodesPerDimension[0] = nodesPerDim;
	m_fem1DBuilder.addBoundaryCondition(0, 6);
	std::shared_ptr<Fem1DRepresentation> fem = m_fem1DBuilder.build("CantileverPunctualLoadAnywhereTest");

	for (size_t applyNode = 1; applyNode < nodesPerDim; applyNode++)
	{
		// For each node, apply load to y-direction and calculate deflection
		double load = 0.7;
		size_t applyIndex = applyNode * 6 + 1;

		Vector applyForce = Vector::Zero(nodesPerDim * 6);
		applyForce[applyIndex] = load;
		fem->update(*(m_fem->getInitialState()), Math::ODEEQUATIONUPDATE_K);
		Matrix stiffness = fem->getK();

		// Apply boundary conditions
		fem->getInitialState()->applyBoundaryConditionsToVector(&applyForce);
		fem->getInitialState()->applyBoundaryConditionsToMatrix(&stiffness);

		Vector calculatedDeflection = stiffness.inverse() * applyForce;

		for (size_t lookNode = 0; lookNode < nodesPerDim; lookNode++)
		{
			// For each node, compare theoretical deflection with calculated deflection
			size_t lookIndex = lookNode * 6 + 1;

			double a = m_L * applyNode / (nodesPerDim - 1);
			double x = m_L * lookNode / (nodesPerDim - 1);
			double deflection = (x < a) ? load * x * x * (3 * a - x) / (6 * m_E * m_Iz)
								: load * a * a * (3 * x - a) / (6 * m_E * m_Iz);

			EXPECT_NEAR(deflection, calculatedDeflection[lookIndex], 1e-8);
		}
	}
}

TEST_F(Fem1DMechanicalValidationTests, CantileverEndBentTest)
{
	// Setup FEM
	size_t nodesPerDim = 5;

	m_fem1DBuilder.nodesPerDimension[0] = nodesPerDim;
	m_fem1DBuilder.addBoundaryCondition(0, 6);
	std::shared_ptr<Fem1DRepresentation> fem = m_fem1DBuilder.build("CantileverEndBentTest");

	// For last node, apply moment to z-rotational direction and calculate deflection
	double moment = 0.7;
	size_t applyIndex = (nodesPerDim - 1) * 6 + 5;

	Vector applyForce = Vector::Zero(nodesPerDim * 6);
	applyForce[applyIndex] = moment;
	fem->update(*(m_fem->getInitialState()), Math::ODEEQUATIONUPDATE_K);
	Matrix stiffness = fem->getK();

	// Apply boundary conditions
	fem->getInitialState()->applyBoundaryConditionsToVector(&applyForce);
	fem->getInitialState()->applyBoundaryConditionsToMatrix(&stiffness);

	Vector calculatedDeflection = stiffness.inverse() * applyForce;

	for (size_t lookNode = 0; lookNode < nodesPerDim; lookNode++)
	{
		// For each node, compare theoretical deflection with calculated deflection
		size_t lookIndex = lookNode * 6 + 1;

		double x = m_L * lookNode / (nodesPerDim - 1);
		double deflection = moment * x * x / (2 * m_E * m_Iz);

		EXPECT_NEAR(deflection, calculatedDeflection[lookIndex], 1e-8);
	}
}

TEST_F(Fem1DMechanicalValidationTests, EndSupportedBeamCenterLoadedTest)
{
	// Setup FEM
	size_t nodesPerDim = 5;

	m_fem1DBuilder.nodesPerDimension[0] = nodesPerDim;
	m_fem1DBuilder.addBoundaryCondition(0, 3);
	m_fem1DBuilder.addBoundaryCondition(nodesPerDim - 1, 3);
	std::shared_ptr<Fem1DRepresentation> fem = m_fem1DBuilder.build("EndSupportedBeamCenterLoadedTest");

	// For middle node, apply load to y-direction and calculate deflection
	double load = 0.7;
	size_t applyIndex = (nodesPerDim / 2) * 6 + 1;

	Vector applyForce = Vector::Zero(nodesPerDim * 6);
	applyForce[applyIndex] = load;
	fem->update(*(m_fem->getInitialState()), Math::ODEEQUATIONUPDATE_K);
	Matrix stiffness = fem->getK();

	// Apply boundary conditions
	fem->getInitialState()->applyBoundaryConditionsToVector(&applyForce);
	fem->getInitialState()->applyBoundaryConditionsToMatrix(&stiffness);

	Vector calculatedDeflection = stiffness.inverse() * applyForce;

	// Compare theoretical deflection with calculated deflection
	size_t lookIndex = applyIndex;

	double deflection = load * (m_L * m_L * m_L) / (48.0 * m_E * m_Iz);

	EXPECT_NEAR(deflection, calculatedDeflection[lookIndex], 1e-8);
}

TEST_F(Fem1DMechanicalValidationTests, EndSupportedBeamIntermediatelyLoadedTest)
{
	// Setup FEM
	size_t nodesPerDim = 5;

	m_fem1DBuilder.nodesPerDimension[0] = nodesPerDim;
	m_fem1DBuilder.addBoundaryCondition(0, 3);
	m_fem1DBuilder.addBoundaryCondition(nodesPerDim - 1, 3);
	std::shared_ptr<Fem1DRepresentation> fem = m_fem1DBuilder.build("EndSupportedBeamIntermediatelyLoadedTest");

	for (size_t node = 0; node < nodesPerDim; node++)
	{
		// For each node, apply load to y-direction and calculate deflection
		double load = 0.7;
		size_t applyIndex = node * 6 + 1;

		Vector applyForce = Vector::Zero(nodesPerDim * 6);
		applyForce[applyIndex] = load;
		fem->update(*(m_fem->getInitialState()), Math::ODEEQUATIONUPDATE_K);
		Matrix stiffness = fem->getK();

		// Apply boundary conditions
		fem->getInitialState()->applyBoundaryConditionsToVector(&applyForce);
		fem->getInitialState()->applyBoundaryConditionsToMatrix(&stiffness);

		Vector calculatedDeflection = stiffness.inverse() * applyForce;

		// Compare theoretical deflection with calculated deflection
		size_t lookIndex = applyIndex;

		double a = static_cast<double>(node) / static_cast<double>(nodesPerDim - 1) * m_L;
		double b = m_L - a;
		double deflection = load * b * a / (6 * m_L * m_E * m_Iz) * (m_L * m_L - a * a - b * b);

		EXPECT_NEAR(deflection, calculatedDeflection[lookIndex], 1e-8);
	}
}

} // namespace Physics

} // namespace SurgSim
