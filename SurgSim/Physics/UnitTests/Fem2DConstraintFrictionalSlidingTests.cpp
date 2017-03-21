// This file is a part of the OpenSurgSim project.
// Copyright 2016, SimQuest Solutions Inc.
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

#include "SurgSim/Framework/Runtime.h"
#include "SurgSim/Math/LinearSparseSolveAndInverse.h"
#include "SurgSim/Math/OdeState.h"
#include "SurgSim/Math/RigidTransform.h"
#include "SurgSim/Math/SparseMatrix.h"
#include "SurgSim/Math/Vector.h"
#include "SurgSim/Physics/Fem2DElementTriangle.h"
#include "SurgSim/Physics/Fem2DLocalization.h"
#include "SurgSim/Physics/Fem2DRepresentation.h"
#include "SurgSim/Physics/FemConstraintFrictionalSliding.h"
#include "SurgSim/Physics/MlcpPhysicsProblem.h"
#include "SurgSim/Physics/SlidingConstraintData.h"
#include "SurgSim/Physics/UnitTests/EigenGtestAsserts.h"
#include "SurgSim/Physics/UnitTests/MockObjects.h"

using SurgSim::DataStructures::IndexedLocalCoordinate;
using SurgSim::Framework::Runtime;
using SurgSim::Physics::Fem2DRepresentation;
using SurgSim::Physics::FemConstraintFrictionalSliding;
using SurgSim::Physics::Fem2DLocalization;
using SurgSim::Physics::Fem2DElementTriangle;
using SurgSim::Physics::MlcpPhysicsProblem;
using SurgSim::Physics::MockFem2DRepresentation;
using SurgSim::Physics::SlidingConstraintData;
using SurgSim::Math::Vector3d;

namespace
{
const double epsilon = 1e-10;
const double dt = 1e-3;
};

static void addTriangle(Fem2DRepresentation* fem,
						size_t node0, size_t node1, size_t node2,
						double thickness = 0.01,
						double massDensity = 1.0,
						double poissonRatio = 0.1,
						double youngModulus = 1.0)
{
	std::array<size_t, 3> nodes = {node0, node1, node2};
	std::shared_ptr<Fem2DElementTriangle> element(new Fem2DElementTriangle(nodes));
	element->setThickness(thickness);
	element->setMassDensity(massDensity);
	element->setPoissonRatio(poissonRatio);
	element->setYoungModulus(youngModulus);
	fem->addFemElement(element);
}

class Fem2DConstraintFrictionalSlidingTests : public ::testing::Test
{
public:
	void SetUp()
	{
		// Define plane with normal 'n' pointing against gravity.
		m_slidingDirection = Vector3d(0.8539, 0.6289, -0.9978);
		m_slidingDirection.normalize();

		// Create mock FEM
		m_fem = std::make_shared<MockFem2DRepresentation>("Fem2dRepresentation");
		auto state = std::make_shared<SurgSim::Math::OdeState>();
		state->setNumDof(6, 5);

		state->getPositions().segment<3>(0 * 6) = Vector3d(0.30, -0.57,  0.40);
		state->getPositions().segment<3>(1 * 6) = Vector3d(0.06,  0.63, -0.32);
		state->getPositions().segment<3>(2 * 6) = Vector3d(-0.91,  0.72,  0.72);
		state->getPositions().segment<3>(3 * 6) = Vector3d(0.35,  0.52,  0.50);
		state->getPositions().segment<3>(4 * 6) = Vector3d(1.14,  0.66,  0.71);

		addTriangle(m_fem.get(), 0, 1, 2);
		addTriangle(m_fem.get(), 1, 2, 3);
		addTriangle(m_fem.get(), 2, 3, 4);

		m_fem->setInitialState(state);
		m_fem->setIntegrationScheme(SurgSim::Math::IntegrationScheme::INTEGRATIONSCHEME_EULER_EXPLICIT_MODIFIED);
		m_fem->setLocalActive(true);

		m_fem->initialize(std::make_shared<Runtime>());
		m_fem->wakeUp();

		// Update model by one timestep
		m_fem->beforeUpdate(dt);
		m_fem->update(dt);
	}

	void setSlidingConstraintAt(const IndexedLocalCoordinate& coord)
	{
		m_localization = std::make_shared<Fem2DLocalization>(m_fem, coord);
		m_constraintData.setSlidingDirection(m_localization->calculatePosition(0.0), m_slidingDirection);
	}

	std::shared_ptr<MockFem2DRepresentation> m_fem;
	std::shared_ptr<Fem2DLocalization> m_localization;

	Vector3d m_slidingDirection;
	SlidingConstraintData m_constraintData;
};

TEST_F(Fem2DConstraintFrictionalSlidingTests, ConstructorTest)
{
	ASSERT_NO_THROW(
	{
		FemConstraintFrictionalSliding femContact;
	});
}

TEST_F(Fem2DConstraintFrictionalSlidingTests, ConstraintConstantsTest)
{
	auto implementation = std::make_shared<FemConstraintFrictionalSliding>();

	EXPECT_EQ(SurgSim::Physics::FRICTIONAL_SLIDING, implementation->getConstraintType());
	EXPECT_EQ(3u, implementation->getNumDof());
}

TEST_F(Fem2DConstraintFrictionalSlidingTests, BuildMlcpTest)
{
	// This test verifies the build mlcp function works for a simple case.

	auto implementation = std::make_shared<FemConstraintFrictionalSliding>();

	// Initialize MLCP
	MlcpPhysicsProblem mlcpPhysicsProblem = MlcpPhysicsProblem::Zero(m_fem->getNumDof(), 3, 1);

	// Apply constraint purely to the first node of the 0th tetrahedron.
	IndexedLocalCoordinate coord(0, Vector3d(1.0, 0.0, 0.0));
	setSlidingConstraintAt(coord);

	implementation->build(dt, m_constraintData, m_localization,
						  &mlcpPhysicsProblem, 0, 0, SurgSim::Physics::CONSTRAINT_POSITIVE_SIDE);

	Eigen::Matrix<double, 3, 30> H = Eigen::Matrix<double, 3, 30>::Zero();
	H.block<1, 3>(0, 0) = (dt * m_constraintData.getNormals()[0]).eval();
	H.block<1, 3>(1, 0) = (dt * m_constraintData.getNormals()[1]).eval();
	H.block<1, 3>(2, 0) = (dt * m_slidingDirection).eval();

	EXPECT_NEAR_EIGEN(H, mlcpPhysicsProblem.H, epsilon);

	// C = dt * m^{-1}
	m_fem->updateFMDK(*(m_fem->getPreviousState()), SurgSim::Math::ODEEQUATIONUPDATE_M);
	Eigen::Matrix<double, 30, 30> C = dt * m_fem->getM().toDense().inverse();

	EXPECT_NEAR_EIGEN(C * H.transpose(), mlcpPhysicsProblem.CHt, epsilon);

	EXPECT_NEAR_EIGEN(H * C * H.transpose(), mlcpPhysicsProblem.A, epsilon);

	EXPECT_DOUBLE_EQ(0.5, mlcpPhysicsProblem.mu[0]);

	EXPECT_EQ(0u, mlcpPhysicsProblem.constraintTypes.size());
}

TEST_F(Fem2DConstraintFrictionalSlidingTests, BuildMlcpCoordinateTest)
{
	// This test verifies the build mlcp function works for a more complicated case with different nodes.
	auto implementation = std::make_shared<FemConstraintFrictionalSliding>();

	// Initialize MLCP
	MlcpPhysicsProblem mlcpPhysicsProblem = MlcpPhysicsProblem::Zero(m_fem->getNumDof(), 3, 1);

	// Apply constraint to all nodes of an fem.
	const Vector3d barycentric = Vector3d(0.25, 0.33, 0.42);
	IndexedLocalCoordinate coord(0, barycentric);
	setSlidingConstraintAt(coord);

	implementation->build(dt, m_constraintData, m_localization,
						  &mlcpPhysicsProblem, 0, 0, SurgSim::Physics::CONSTRAINT_POSITIVE_SIDE);

	Eigen::Matrix<double, 3, 30> H = Eigen::Matrix<double, 3, 30>::Zero();
	H.block<1, 3>(0, 0) = (barycentric[0] * dt * m_constraintData.getNormals()[0]).eval();
	H.block<1, 3>(1, 0) = (barycentric[0] * dt * m_constraintData.getNormals()[1]).eval();
	H.block<1, 3>(2, 0) = (barycentric[0] * dt * m_slidingDirection).eval();
	H.block<1, 3>(0, 6) = (barycentric[1] * dt * m_constraintData.getNormals()[0]).eval();
	H.block<1, 3>(1, 6) = (barycentric[1] * dt * m_constraintData.getNormals()[1]).eval();
	H.block<1, 3>(2, 6) = (barycentric[1] * dt * m_slidingDirection).eval();
	H.block<1, 3>(0, 12) = (barycentric[2] * dt * m_constraintData.getNormals()[0]).eval();
	H.block<1, 3>(1, 12) = (barycentric[2] * dt * m_constraintData.getNormals()[1]).eval();
	H.block<1, 3>(2, 12) = (barycentric[2] * dt * m_slidingDirection).eval();

	EXPECT_NEAR_EIGEN(H, mlcpPhysicsProblem.H, epsilon);

	// C = dt * m^{-1}
	m_fem->updateFMDK(*(m_fem->getPreviousState()), SurgSim::Math::ODEEQUATIONUPDATE_M);
	Eigen::Matrix<double, 30, 30> C = dt * m_fem->getM().toDense().inverse();

	EXPECT_NEAR_EIGEN(C * H.transpose(), mlcpPhysicsProblem.CHt, epsilon);

	EXPECT_NEAR_EIGEN(H * C * H.transpose(), mlcpPhysicsProblem.A, epsilon);

	EXPECT_DOUBLE_EQ(0.5, mlcpPhysicsProblem.mu[0]);

	EXPECT_EQ(0u, mlcpPhysicsProblem.constraintTypes.size());
}

TEST_F(Fem2DConstraintFrictionalSlidingTests, BuildMlcpIndiciesTest)
{
	// This test verifies the build mlcp function works given a hypothetical, preexisting mlcp problem.
	auto implementation = std::make_shared<FemConstraintFrictionalSliding>();

	// Initialize MLCP
	MlcpPhysicsProblem mlcpPhysicsProblem = MlcpPhysicsProblem::Zero(m_fem->getNumDof() + 5, 4, 2);

	// Suppose 5 dof and 1 constraint are defined elsewhere.  Then H, CHt, HCHt, and b are prebuilt.
	Eigen::SparseVector<double, Eigen::RowMajor, ptrdiff_t> localH;
	localH.resize(5);
	localH.reserve(5);
	localH.insert(0) = 0.9478;
	localH.insert(1) = -0.3807;
	localH.insert(2) = 0.5536;
	localH.insert(3) = -0.6944;
	localH.insert(4) = 0.1815;

	mlcpPhysicsProblem.H.coeffRef(0, 0) += localH.coeff(0);
	mlcpPhysicsProblem.H.coeffRef(0, 1) += localH.coeff(1);
	mlcpPhysicsProblem.H.coeffRef(0, 2) += localH.coeff(2);
	mlcpPhysicsProblem.H.coeffRef(0, 3) += localH.coeff(3);
	mlcpPhysicsProblem.H.coeffRef(0, 4) += localH.coeff(4);

	Eigen::Matrix<double, 5, 5> localC;
	localC <<
		   -0.2294,  0.5160,  0.2520,  0.5941, -0.4854,
		   0.1233, -0.4433,  0.3679,  0.9307,  0.2600,
		   0.1988,  0.6637, -0.7591,  0.1475,  0.8517,
		   -0.5495, -0.4305,  0.3162, -0.7862,  0.7627,
		   -0.5754,  0.4108,  0.8445, -0.5565,  0.7150;
	localC = localC * localC.transpose(); // force to be symmetric

	Eigen::Matrix<double, 5, 1> localCHt = localC * localH.transpose();
	mlcpPhysicsProblem.CHt.block<5, 1>(0, 0) = localCHt;

	mlcpPhysicsProblem.A.block<1, 1>(0, 0) = localH * localCHt;

	mlcpPhysicsProblem.b.block<1, 1>(0, 0)[0] = 0.6991;

	// Place the Fem at 5th dof (1 or multiple representations exist before, covering a total of 5 dof)
	// and the constraint at the index 1 (1 atomic constraint exists before this one)
	size_t indexOfRepresentation = 5;
	size_t indexOfConstraint = 1;

	// Apply constraint to all nodes of an fem.
	Vector3d barycentric = Vector3d(0.25, 0.33, 0.42);
	IndexedLocalCoordinate coord(0, barycentric);
	setSlidingConstraintAt(coord);

	implementation->build(dt, m_constraintData, m_localization,
						  &mlcpPhysicsProblem, indexOfRepresentation, indexOfConstraint,
						  SurgSim::Physics::CONSTRAINT_POSITIVE_SIDE);

	Eigen::Matrix<double, 3, 30> H = Eigen::Matrix<double, 3, 30>::Zero();
	H.block<1, 3>(0, 0) = (barycentric[0] * dt * m_constraintData.getNormals()[0]).eval();
	H.block<1, 3>(1, 0) = (barycentric[0] * dt * m_constraintData.getNormals()[1]).eval();
	H.block<1, 3>(2, 0) = (barycentric[0] * dt * m_slidingDirection).eval();
	H.block<1, 3>(0, 6) = (barycentric[1] * dt * m_constraintData.getNormals()[0]).eval();
	H.block<1, 3>(1, 6) = (barycentric[1] * dt * m_constraintData.getNormals()[1]).eval();
	H.block<1, 3>(2, 6) = (barycentric[1] * dt * m_slidingDirection).eval();
	H.block<1, 3>(0, 12) = (barycentric[2] * dt * m_constraintData.getNormals()[0]).eval();
	H.block<1, 3>(1, 12) = (barycentric[2] * dt * m_constraintData.getNormals()[1]).eval();
	H.block<1, 3>(2, 12) = (barycentric[2] * dt * m_slidingDirection).eval();

	EXPECT_NEAR_EIGEN(H, mlcpPhysicsProblem.H.block(indexOfConstraint, indexOfRepresentation, 3, 30), epsilon);

	m_fem->updateFMDK(*(m_fem->getPreviousState()), SurgSim::Math::ODEEQUATIONUPDATE_M);
	Eigen::Matrix<double, 30, 30> C = dt * m_fem->getM().toDense().inverse();

	EXPECT_NEAR_EIGEN(
		C * H.transpose(), mlcpPhysicsProblem.CHt.block(indexOfRepresentation, indexOfConstraint, 30, 3), epsilon);

	EXPECT_NEAR_EIGEN(
		H * C * H.transpose(), mlcpPhysicsProblem.A.block(indexOfConstraint, indexOfConstraint, 3, 3), epsilon);

	EXPECT_DOUBLE_EQ(0.5, mlcpPhysicsProblem.mu[1]);

	EXPECT_EQ(0u, mlcpPhysicsProblem.constraintTypes.size());
}
