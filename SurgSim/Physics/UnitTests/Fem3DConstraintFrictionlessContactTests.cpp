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

#include "SurgSim/Framework/Runtime.h"
#include "SurgSim/Math/LinearSparseSolveAndInverse.h"
#include "SurgSim/Math/OdeState.h"
#include "SurgSim/Math/SparseMatrix.h"
#include "SurgSim/Math/Vector.h"
#include "SurgSim/Physics/ContactConstraintData.h"
#include "SurgSim/Physics/Fem3DElementTetrahedron.h"
#include "SurgSim/Physics/Fem3DLocalization.h"
#include "SurgSim/Physics/Fem3DRepresentation.h"
#include "SurgSim/Physics/FemConstraintFrictionlessContact.h"
#include "SurgSim/Physics/MlcpPhysicsProblem.h"
#include "SurgSim/Physics/UnitTests/EigenGtestAsserts.h"

using SurgSim::DataStructures::IndexedLocalCoordinate;
using SurgSim::Framework::Runtime;
using SurgSim::Physics::ContactConstraintData;
using SurgSim::Physics::Fem3DRepresentation;
using SurgSim::Physics::Fem3DConstraintFrictionlessContact;
using SurgSim::Physics::Fem3DLocalization;
using SurgSim::Physics::Fem3DElementTetrahedron;
using SurgSim::Physics::MlcpPhysicsProblem;
using SurgSim::Math::Vector3d;
using SurgSim::Math::Vector4d;

namespace
{
const double epsilon = 1e-10;
const double dt = 1e-3;
};

static void addTetraheadron(Fem3DRepresentation* fem,
							size_t node0,
							size_t node1,
							size_t node2,
							size_t node3,
							double massDensity = 1.0,
							double poissonRatio = 0.1,
							double youngModulus = 1.0)
{
	std::array<size_t, 4> nodes = {node0, node1, node2, node3};
	auto element = std::make_shared<Fem3DElementTetrahedron>(nodes);
	element->setMassDensity(massDensity);
	element->setPoissonRatio(poissonRatio);
	element->setYoungModulus(youngModulus);
	fem->addFemElement(element);
}

class Fem3DConstraintFrictionlessContactTests : public ::testing::Test
{
public:
	void SetUp()
	{
		// Define plane with normal 'n' pointing against gravity.
		m_n = Vector3d(0.8539, 0.6289, -0.9978);
		m_n.normalize();

		// Create mock FEM
		m_fem = std::make_shared<Fem3DRepresentation>("Fem3dRepresentation");
		auto state = std::make_shared<SurgSim::Math::OdeState>();
		state->setNumDof(3, 6);

		// Place coordinates at
		// ( 0.00, 0.00,  0.00) + (0.24, -0.43, 0.55) + ( 0.06, -0.14, -0.15) = ( 0.30, -0.57,  0.40)
		// ( 0.00, 1.00, -1.00) + (0.24, -0.43, 0.55) + (-0.18,  0.06,  0.13) = ( 0.06,  0.63, -0.32)
		// (-1.00, 1.00,  0.00) + (0.24, -0.43, 0.55) + (-0.15,  0.15,  0.17) = (-0.91,  0.72,  0.72)
		// ( 0.00, 1.00,  0.00) + (0.24, -0.43, 0.55) + ( 0.11, -0.05, -0.05) = ( 0.35,  0.52,  0.50)
		// ( 1.00, 1.00,  0.00) + (0.24, -0.43, 0.55) + (-0.10,  0.09,  0.16) = ( 1.14,  0.66,  0.71)
		// ( 1.00, 0.00, -1.00) + (0.24, -0.43, 0.55) + (-0.22,  0.12, -0.09) = ( 1.02, -0.31, -0.54)

		state->getPositions().segment<3>(0 * 3) = Vector3d(0.30, -0.57,  0.40);
		state->getPositions().segment<3>(1 * 3) = Vector3d(0.06,  0.63, -0.32);
		state->getPositions().segment<3>(2 * 3) = Vector3d(-0.91,  0.72,  0.72);
		state->getPositions().segment<3>(3 * 3) = Vector3d(0.35,  0.52,  0.50);
		state->getPositions().segment<3>(4 * 3) = Vector3d(1.14,  0.66,  0.71);
		state->getPositions().segment<3>(5 * 3) = Vector3d(1.02, -0.31, -0.54);

		addTetraheadron(m_fem.get(), 0, 1, 2, 3);
		addTetraheadron(m_fem.get(), 0, 1, 3, 4);
		addTetraheadron(m_fem.get(), 0, 1, 4, 5);

		m_fem->setInitialState(state);
		m_fem->setIntegrationScheme(SurgSim::Math::IntegrationScheme::INTEGRATIONSCHEME_EULER_EXPLICIT_MODIFIED);
		m_fem->setLocalActive(true);

		m_fem->initialize(std::make_shared<Runtime>());
		m_fem->wakeUp();

		// Update model by one timestep
		m_fem->beforeUpdate(dt);
		m_fem->update(dt);
	}

	void setContactAt(const IndexedLocalCoordinate& coord)
	{
		m_coord = coord;
		m_localization = std::make_shared<Fem3DLocalization>(m_fem, m_coord);

		// Calculate position at state before "m_fem->update(dt)" was called.
		double distance = -m_localization->calculatePosition(0.0).dot(m_n);
		m_constraintData.setPlaneEquation(m_n, distance);
	}

	std::shared_ptr<Fem3DRepresentation> m_fem;
	std::shared_ptr<Fem3DLocalization> m_localization;

	IndexedLocalCoordinate m_coord;
	Vector3d m_n;
	ContactConstraintData m_constraintData;
};

TEST_F(Fem3DConstraintFrictionlessContactTests, ConstructorTest)
{
	ASSERT_NO_THROW(
	{
		Fem3DConstraintFrictionlessContact femContact;
	});
}

TEST_F(Fem3DConstraintFrictionlessContactTests, ConstraintConstantsTest)
{
	auto implementation = std::make_shared<Fem3DConstraintFrictionlessContact>();

	EXPECT_EQ(SurgSim::Physics::FRICTIONLESS_3DCONTACT, implementation->getConstraintType());
	EXPECT_EQ(1u, implementation->getNumDof());
}

TEST_F(Fem3DConstraintFrictionlessContactTests, BuildMlcpTest)
{
	// This test verifies the build mlcp function works for a simple case.

	auto implementation = std::make_shared<Fem3DConstraintFrictionlessContact>();

	// Initialize MLCP
	MlcpPhysicsProblem mlcpPhysicsProblem = MlcpPhysicsProblem::Zero(m_fem->getNumDof(), 1, 1);

	// Apply constraint purely to the first node of the 0th tetrahedron.
	IndexedLocalCoordinate coord(0, Vector4d(1.0, 0.0, 0.0, 0.0));
	setContactAt(coord);

	implementation->build(dt, m_constraintData, m_localization,
						  &mlcpPhysicsProblem, 0, 0, SurgSim::Physics::CONSTRAINT_POSITIVE_SIDE);

	const Vector3d newPosition = Vector3d(0.30, -0.57,  0.40) - Vector3d::UnitY() * 9.81 * dt * dt;
	EXPECT_NEAR(newPosition.dot(m_n), mlcpPhysicsProblem.b[0], epsilon);

	Eigen::Matrix<double, 1, 18> H = Eigen::Matrix<double, 1, 18>::Zero();
	SurgSim::Math::setSubVector(dt * m_n, 0, 3, &H);

	EXPECT_NEAR_EIGEN(H, mlcpPhysicsProblem.H, epsilon);

	// C = dt * m^{-1}
	m_fem->updateFMDK(*(m_fem->getPreviousState()), SurgSim::Math::ODEEQUATIONUPDATE_M);
	Eigen::Matrix<double, 18, 18> denseMat = m_fem->getM();
	Eigen::Matrix<double, 18, 18> C = dt * denseMat.inverse();

	EXPECT_NEAR_EIGEN(C * H.transpose(), mlcpPhysicsProblem.CHt, epsilon);

	EXPECT_NEAR_EIGEN(H * C * H.transpose(), mlcpPhysicsProblem.A, epsilon);

	EXPECT_EQ(0u, mlcpPhysicsProblem.constraintTypes.size());
}

TEST_F(Fem3DConstraintFrictionlessContactTests, BuildMlcpCoordinateTest)
{
	// This test verifies the build mlcp function works for a more complicated case with different nodes.

	auto implementation = std::make_shared<Fem3DConstraintFrictionlessContact>();

	// Initialize MLCP
	MlcpPhysicsProblem mlcpPhysicsProblem = MlcpPhysicsProblem::Zero(m_fem->getNumDof(), 1, 1);

	// Apply constraint to all nodes of an fem.
	const Vector4d barycentric = Vector4d(0.25, 0.33, 0.28, 0.14);
	IndexedLocalCoordinate coord(1, barycentric);
	setContactAt(coord);

	implementation->build(dt, m_constraintData, m_localization,
						  &mlcpPhysicsProblem, 0, 0, SurgSim::Physics::CONSTRAINT_POSITIVE_SIDE);

	const Vector3d newPosition = (Vector3d(0.30, -0.57,  0.40) * barycentric[0] +
								  Vector3d(0.06,  0.63, -0.32) * barycentric[1] +
								  Vector3d(0.35,  0.52,  0.50) * barycentric[2] +
								  Vector3d(1.14,  0.66,  0.71) * barycentric[3]) -
								 Vector3d::UnitY() * 9.81 * dt * dt;
	EXPECT_NEAR(newPosition.dot(m_n), mlcpPhysicsProblem.b[0], epsilon);

	Eigen::Matrix<double, 1, 18> H = Eigen::Matrix<double, 1, 18>::Zero();
	SurgSim::Math::setSubVector(0.25 * dt * m_n, 0, 3, &H);
	SurgSim::Math::setSubVector(0.33 * dt * m_n, 1, 3, &H);
	SurgSim::Math::setSubVector(0.28 * dt * m_n, 3, 3, &H);
	SurgSim::Math::setSubVector(0.14 * dt * m_n, 4, 3, &H);

	EXPECT_NEAR_EIGEN(H, mlcpPhysicsProblem.H, epsilon);

	// C = dt * m^{-1}
	SurgSim::Math::Matrix C;
	SurgSim::Math::SparseMatrix M(18, 18);
	m_fem->updateFMDK(*(m_fem->getPreviousState()), SurgSim::Math::ODEEQUATIONUPDATE_M);
	M = m_fem->getM();
	SurgSim::Math::LinearSparseSolveAndInverseLU solver;
	SurgSim::Math::Vector b = SurgSim::Math::Vector::Zero(18);
	solver.setMatrix(M);
	C = solver.getInverse();
	C *= dt;

	EXPECT_NEAR_EIGEN(C * H.transpose(), mlcpPhysicsProblem.CHt, epsilon);

	EXPECT_NEAR_EIGEN(H * C * H.transpose(), mlcpPhysicsProblem.A, epsilon);

	EXPECT_EQ(0u, mlcpPhysicsProblem.constraintTypes.size());
}

TEST_F(Fem3DConstraintFrictionlessContactTests, BuildMlcpIndiciesTest)
{
	// This test verifies the build mlcp function works given a hypothetical, preexisting mlcp problem.

	auto implementation = std::make_shared<Fem3DConstraintFrictionlessContact>();

	// Initialize MLCP
	MlcpPhysicsProblem mlcpPhysicsProblem = MlcpPhysicsProblem::Zero(m_fem->getNumDof() + 5, 2, 1);

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
	const Vector4d barycentric = Vector4d(0.25, 0.33, 0.28, 0.14);
	IndexedLocalCoordinate coord(1, barycentric);
	setContactAt(coord);

	implementation->build(dt, m_constraintData, m_localization,
						  &mlcpPhysicsProblem, indexOfRepresentation, indexOfConstraint,
						  SurgSim::Physics::CONSTRAINT_POSITIVE_SIDE);

	const Vector3d newPosition = (Vector3d(0.30, -0.57,  0.40) * barycentric[0] +
								  Vector3d(0.06,  0.63, -0.32) * barycentric[1] +
								  Vector3d(0.35,  0.52,  0.50) * barycentric[2] +
								  Vector3d(1.14,  0.66,  0.71) * barycentric[3]) -
								 Vector3d::UnitY() * 9.81 * dt * dt;
	EXPECT_NEAR(newPosition.dot(m_n), mlcpPhysicsProblem.b[indexOfConstraint], epsilon);

	Eigen::Matrix<double, 1, 18> H = Eigen::Matrix<double, 1, 18>::Zero();
	SurgSim::Math::setSubVector(0.25 * dt * m_n, 0, 3, &H);
	SurgSim::Math::setSubVector(0.33 * dt * m_n, 1, 3, &H);
	SurgSim::Math::setSubVector(0.28 * dt * m_n, 3, 3, &H);
	SurgSim::Math::setSubVector(0.14 * dt * m_n, 4, 3, &H);

	EXPECT_NEAR_EIGEN(H, mlcpPhysicsProblem.H.block(indexOfConstraint, indexOfRepresentation, 1, 18), epsilon);

	SurgSim::Math::Matrix C;
	SurgSim::Math::SparseMatrix M(18, 18);
	m_fem->updateFMDK(*m_fem->getPreviousState(), SurgSim::Math::ODEEQUATIONUPDATE_M);
	M = m_fem->getM();
	SurgSim::Math::LinearSparseSolveAndInverseLU solver;
	SurgSim::Math::Vector b = SurgSim::Math::Vector::Zero(18);
	solver.setMatrix(M);
	C = solver.getInverse();
	C *= dt;

	EXPECT_NEAR_EIGEN(
		C * H.transpose(), mlcpPhysicsProblem.CHt.block(indexOfRepresentation, indexOfConstraint, 18, 1), epsilon);

	EXPECT_NEAR_EIGEN(
		H * C * H.transpose(), mlcpPhysicsProblem.A.block(indexOfConstraint, indexOfConstraint, 1, 1), epsilon);

	EXPECT_EQ(0u, mlcpPhysicsProblem.constraintTypes.size());
}
