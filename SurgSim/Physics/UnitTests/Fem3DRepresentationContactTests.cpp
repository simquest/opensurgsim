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
#include <memory>

#include "SurgSim/Framework/Runtime.h"
#include "SurgSim/Math/Vector.h"
#include "SurgSim/Physics/ContactConstraintData.h"
#include "SurgSim/Physics/Fem3DRepresentation.h"
#include "SurgSim/Physics/Fem3DRepresentationContact.h"
#include "SurgSim/Physics/Fem3DRepresentationLocalization.h"
#include "SurgSim/Physics/FemElement3DTetrahedron.h"
#include "SurgSim/Physics/MlcpPhysicsProblem.h"

using SurgSim::Framework::Runtime;
using SurgSim::Physics::ContactConstraintData;
using SurgSim::Physics::DeformableRepresentationState;
using SurgSim::Physics::Fem3DRepresentation;
using SurgSim::Physics::Fem3DRepresentationContact;
using SurgSim::Physics::Fem3DRepresentationLocalization;
using SurgSim::Physics::FemElement3DTetrahedron;
using SurgSim::Physics::FemRepresentationCoordinate;
using SurgSim::Physics::MlcpPhysicsProblem;
using SurgSim::Math::Vector3d;
using SurgSim::Math::Vector4d;

namespace
{
	const double epsilon = 1e-10;
	const double dt = 1e-3;
};

static void addTetraheadron(Fem3DRepresentation *fem,
							unsigned int node0,
							unsigned int node1,
							unsigned int node2,
							unsigned int node3,
							const DeformableRepresentationState &state,
							double massDensity = 1.0,
							double poissonRatio = 0.1,
							double youngModulus = 1.0)
{
	std::array<unsigned int, 4> nodes = {node0, node1, node2, node3};
	auto element = std::make_shared<FemElement3DTetrahedron>(nodes, state);
	element->setMassDensity(massDensity);
	element->setPoissonRatio(poissonRatio);
	element->setYoungModulus(youngModulus);
	element->initialize(state);
	fem->addFemElement(element);
}

class Fem3DRepresentationContactTests : public ::testing::Test
{
public:
	void SetUp()
	{
		// Define plane with normal 'n' pointing against gravity.
		m_n = Vector3d(0.8539, 0.6289, -0.9978);
		m_n.normalize();

		// Create mock FEM
		m_fem = std::make_shared<Fem3DRepresentation>("Fem3dRepresentation");
		auto state = std::make_shared<DeformableRepresentationState>();
		state->setNumDof(3, 6);

		// Place coordinates at
		// ( 0.00, 0.00,  0.00) + (0.24, -0.43, 0.55) + ( 0.06, -0.14, -0.15) = ( 0.30, -0.57,  0.40)
		// ( 0.00, 1.00, -1.00) + (0.24, -0.43, 0.55) + (-0.18,  0.06,  0.13) = ( 0.06,  0.63, -0.32)
		// (-1.00, 1.00,  0.00) + (0.24, -0.43, 0.55) + (-0.15,  0.15,  0.17) = (-0.91,  0.72,  0.72)
		// ( 0.00, 1.00,  0.00) + (0.24, -0.43, 0.55) + ( 0.11, -0.05, -0.05) = ( 0.35,  0.52,  0.50)
		// ( 1.00, 1.00,  0.00) + (0.24, -0.43, 0.55) + (-0.10,  0.09,  0.16) = ( 1.14,  0.66,  0.71)
		// ( 1.00, 0.00, -1.00) + (0.24, -0.43, 0.55) + (-0.22,  0.12, -0.09) = ( 1.02, -0.31, -0.54)

		state->getPositions().segment<3>(0 * 3) = Vector3d( 0.30, -0.57,  0.40);
		state->getPositions().segment<3>(1 * 3) = Vector3d( 0.06,  0.63, -0.32);
		state->getPositions().segment<3>(2 * 3) = Vector3d(-0.91,  0.72,  0.72);
		state->getPositions().segment<3>(3 * 3) = Vector3d( 0.35,  0.52,  0.50);
		state->getPositions().segment<3>(4 * 3) = Vector3d( 1.14,  0.66,  0.71);
		state->getPositions().segment<3>(5 * 3) = Vector3d( 1.02, -0.31, -0.54);

		addTetraheadron(m_fem.get(), 0, 1, 2, 3, *state);
		addTetraheadron(m_fem.get(), 0, 1, 3, 4, *state);
		addTetraheadron(m_fem.get(), 0, 1, 4, 5, *state);

		m_fem->setInitialState(state);
		m_fem->initialize(std::make_shared<Runtime>());

		m_fem->setIntegrationScheme(SurgSim::Math::IntegrationScheme::INTEGRATIONSCHEME_MODIFIED_EXPLICIT_EULER);
		m_fem->setIsActive(true);

		// Update model by one timestep
		m_fem->beforeUpdate(dt);
		m_fem->update(dt);

		m_localization = std::make_shared<Fem3DRepresentationLocalization>(m_fem);
	}

	void setContactAt(const FemRepresentationCoordinate &coord)
	{
		m_coord = coord;
		m_localization->setLocalPosition(coord);

		// Calculate position at state before "m_fem->update(dt)" was called.
		double distance = -m_localization->calculatePosition(0.0).dot(m_n);
		m_constraintData.setPlaneEquation(m_n, distance);
	}

	std::shared_ptr<Fem3DRepresentation> m_fem;
	std::shared_ptr<Fem3DRepresentationLocalization> m_localization;

	FemRepresentationCoordinate m_coord;
	Vector3d m_n;
	ContactConstraintData m_constraintData;
};

TEST_F(Fem3DRepresentationContactTests, ConstructorTest)
{
	ASSERT_NO_THROW({
		Fem3DRepresentationContact femContact;
	});
}

TEST_F(Fem3DRepresentationContactTests, ConstraintConstantsTest)
{
	auto implementation = std::make_shared<Fem3DRepresentationContact>();

	EXPECT_EQ(SurgSim::Math::MLCP_UNILATERAL_3D_FRICTIONLESS_CONSTRAINT, implementation->getMlcpConstraintType());
	EXPECT_EQ(SurgSim::Physics::REPRESENTATION_TYPE_FEM3D, implementation->getRepresentationType());
	EXPECT_EQ(1u, implementation->getNumDof());
}

static void initializeMlcp(MlcpPhysicsProblem *mlcpPhysicsProblem, size_t numDof, size_t numConstraints)
{
	// Resize and zero all Eigen types
	mlcpPhysicsProblem->A = SurgSim::Math::Matrix::Zero(numConstraints, numConstraints);
	mlcpPhysicsProblem->b = SurgSim::Math::Vector::Zero(numConstraints);
	mlcpPhysicsProblem->mu = SurgSim::Math::Vector::Zero(numConstraints);
	mlcpPhysicsProblem->CHt = SurgSim::Math::Matrix::Zero(numDof, numConstraints);
	mlcpPhysicsProblem->H = SurgSim::Math::Matrix::Zero(numConstraints, numDof);
	// Empty all std::vector types
	mlcpPhysicsProblem->constraintTypes.clear();
}

TEST_F(Fem3DRepresentationContactTests, BuildMlcpTest)
{
	// This test verifies the build mlcp function works for a simple case.

	auto implementation = std::make_shared<Fem3DRepresentationContact>();

	// Initialize MLCP
	MlcpPhysicsProblem mlcpPhysicsProblem;
	initializeMlcp(&mlcpPhysicsProblem, m_fem->getNumDof(), 1);

	// Apply constraint purely to the first node of the 0th tetrahedron.
	FemRepresentationCoordinate coord(0, Vector4d(1.0, 0.0, 0.0, 0.0));
	setContactAt(coord);

	implementation->build(dt, m_constraintData, m_localization,
		&mlcpPhysicsProblem, 0, 0, SurgSim::Physics::CONSTRAINT_POSITIVE_SIDE);

	EXPECT_NEAR(-9.81 * dt * dt * m_n[1], mlcpPhysicsProblem.b[0], epsilon);

	EXPECT_NEAR(m_n[0], mlcpPhysicsProblem.H(0,  0), epsilon);
	EXPECT_NEAR(m_n[1], mlcpPhysicsProblem.H(0,  1), epsilon);
	EXPECT_NEAR(m_n[2], mlcpPhysicsProblem.H(0,  2), epsilon);
	EXPECT_NEAR(0.0,    mlcpPhysicsProblem.H(0,  3), epsilon);
	EXPECT_NEAR(0.0,    mlcpPhysicsProblem.H(0,  4), epsilon);
	EXPECT_NEAR(0.0,    mlcpPhysicsProblem.H(0,  5), epsilon);
	EXPECT_NEAR(0.0,    mlcpPhysicsProblem.H(0,  6), epsilon);
	EXPECT_NEAR(0.0,    mlcpPhysicsProblem.H(0,  7), epsilon);
	EXPECT_NEAR(0.0,    mlcpPhysicsProblem.H(0,  8), epsilon);
	EXPECT_NEAR(0.0,    mlcpPhysicsProblem.H(0,  9), epsilon);
	EXPECT_NEAR(0.0,    mlcpPhysicsProblem.H(0, 10), epsilon);
	EXPECT_NEAR(0.0,    mlcpPhysicsProblem.H(0, 11), epsilon);
	EXPECT_NEAR(0.0,    mlcpPhysicsProblem.H(0, 12), epsilon);
	EXPECT_NEAR(0.0,    mlcpPhysicsProblem.H(0, 13), epsilon);
	EXPECT_NEAR(0.0,    mlcpPhysicsProblem.H(0, 14), epsilon);
	EXPECT_NEAR(0.0,    mlcpPhysicsProblem.H(0, 15), epsilon);
	EXPECT_NEAR(0.0,    mlcpPhysicsProblem.H(0, 16), epsilon);
	EXPECT_NEAR(0.0,    mlcpPhysicsProblem.H(0, 17), epsilon);

	// C = dt * m^{-1}
	//
	// => CHt = [C[0,0:3]*n] = [(C[0,0]*nx + C[0,1]*ny + C[0,2]*nz])]
	//			[C[1,0:3]*n]   [(C[1,0]*nx + C[1,1]*ny + C[1,2]*nz])]
	//			[...	   ]   [ ...								]
	//
	// Note computeM is verified in BuildMlcpMassTest.
	Eigen::Matrix<double, 18, 18> C = dt * m_fem->computeM(*m_fem->getPreviousState()).inverse();

	EXPECT_NEAR(C( 0, 0) * m_n[0] + C( 0, 1) * m_n[1] + C( 0, 2) * m_n[2], mlcpPhysicsProblem.CHt( 0, 0), epsilon);
	EXPECT_NEAR(C( 1, 0) * m_n[0] + C( 1, 1) * m_n[1] + C( 1, 2) * m_n[2], mlcpPhysicsProblem.CHt( 1, 0), epsilon);
	EXPECT_NEAR(C( 2, 0) * m_n[0] + C( 2, 1) * m_n[1] + C( 2, 2) * m_n[2], mlcpPhysicsProblem.CHt( 2, 0), epsilon);
	EXPECT_NEAR(C( 3, 0) * m_n[0] + C( 3, 1) * m_n[1] + C( 3, 2) * m_n[2], mlcpPhysicsProblem.CHt( 3, 0), epsilon);
	EXPECT_NEAR(C( 4, 0) * m_n[0] + C( 4, 1) * m_n[1] + C( 4, 2) * m_n[2], mlcpPhysicsProblem.CHt( 4, 0), epsilon);
	EXPECT_NEAR(C( 5, 0) * m_n[0] + C( 5, 1) * m_n[1] + C( 5, 2) * m_n[2], mlcpPhysicsProblem.CHt( 5, 0), epsilon);
	EXPECT_NEAR(C( 6, 0) * m_n[0] + C( 6, 1) * m_n[1] + C( 6, 2) * m_n[2], mlcpPhysicsProblem.CHt( 6, 0), epsilon);
	EXPECT_NEAR(C( 7, 0) * m_n[0] + C( 7, 1) * m_n[1] + C( 7, 2) * m_n[2], mlcpPhysicsProblem.CHt( 7, 0), epsilon);
	EXPECT_NEAR(C( 8, 0) * m_n[0] + C( 8, 1) * m_n[1] + C( 8, 2) * m_n[2], mlcpPhysicsProblem.CHt( 8, 0), epsilon);
	EXPECT_NEAR(C( 9, 0) * m_n[0] + C( 9, 1) * m_n[1] + C( 9, 2) * m_n[2], mlcpPhysicsProblem.CHt( 9, 0), epsilon);
	EXPECT_NEAR(C(10, 0) * m_n[0] + C(10, 1) * m_n[1] + C(10, 2) * m_n[2], mlcpPhysicsProblem.CHt(10, 0), epsilon);
	EXPECT_NEAR(C(11, 0) * m_n[0] + C(11, 1) * m_n[1] + C(11, 2) * m_n[2], mlcpPhysicsProblem.CHt(11, 0), epsilon);
	EXPECT_NEAR(C(12, 0) * m_n[0] + C(12, 1) * m_n[1] + C(12, 2) * m_n[2], mlcpPhysicsProblem.CHt(12, 0), epsilon);
	EXPECT_NEAR(C(13, 0) * m_n[0] + C(13, 1) * m_n[1] + C(13, 2) * m_n[2], mlcpPhysicsProblem.CHt(13, 0), epsilon);
	EXPECT_NEAR(C(14, 0) * m_n[0] + C(14, 1) * m_n[1] + C(14, 2) * m_n[2], mlcpPhysicsProblem.CHt(14, 0), epsilon);
	EXPECT_NEAR(C(15, 0) * m_n[0] + C(15, 1) * m_n[1] + C(15, 2) * m_n[2], mlcpPhysicsProblem.CHt(15, 0), epsilon);
	EXPECT_NEAR(C(16, 0) * m_n[0] + C(16, 1) * m_n[1] + C(16, 2) * m_n[2], mlcpPhysicsProblem.CHt(16, 0), epsilon);
	EXPECT_NEAR(C(17, 0) * m_n[0] + C(17, 1) * m_n[1] + C(17, 2) * m_n[2], mlcpPhysicsProblem.CHt(17, 0), epsilon);

	EXPECT_NEAR(
		  m_n[0] * (C(0, 0) * m_n[0] + C(0, 1) * m_n[1] + C(0, 2) * m_n[2])
		+ m_n[1] * (C(1, 0) * m_n[0] + C(1, 1) * m_n[1] + C(1, 2) * m_n[2])
		+ m_n[2] * (C(2, 0) * m_n[0] + C(2, 1) * m_n[1] + C(2, 2) * m_n[2]), mlcpPhysicsProblem.A(0, 0), epsilon);

	EXPECT_EQ(0u, mlcpPhysicsProblem.constraintTypes.size());
}

TEST_F(Fem3DRepresentationContactTests, BuildMlcpCoordinateTest)
{
	// This test verifies the build mlcp function works for a more complicated case with different nodes.

	auto implementation = std::make_shared<Fem3DRepresentationContact>();

	// Initialize MLCP
	MlcpPhysicsProblem mlcpPhysicsProblem;
	initializeMlcp(&mlcpPhysicsProblem, m_fem->getNumDof(), 1);

	// Apply constraint to all nodes of an fem.
	FemRepresentationCoordinate coord(1, Vector4d(0.25, 0.33, 0.28, 0.14));
	setContactAt(coord);

	implementation->build(dt, m_constraintData, m_localization,
		&mlcpPhysicsProblem, 0, 0, SurgSim::Physics::CONSTRAINT_POSITIVE_SIDE);

	EXPECT_NEAR(-9.81 * dt * dt * m_n[1], mlcpPhysicsProblem.b[0], epsilon);

	Eigen::Matrix<double, 18, 1> H;
	H << 0.25 * m_n[0],
		 0.25 * m_n[1],
		 0.25 * m_n[2],
		 0.33 * m_n[0],
		 0.33 * m_n[1],
		 0.33 * m_n[2],
		 0.0,
		 0.0,
		 0.0,
		 0.28 * m_n[0],
		 0.28 * m_n[1],
		 0.28 * m_n[2],
		 0.14 * m_n[0],
		 0.14 * m_n[1],
		 0.14 * m_n[2],
		 0.0,
		 0.0,
		 0.0;

	EXPECT_NEAR(H( 0), mlcpPhysicsProblem.H(0,  0), epsilon);
	EXPECT_NEAR(H( 1), mlcpPhysicsProblem.H(0,  1), epsilon);
	EXPECT_NEAR(H( 2), mlcpPhysicsProblem.H(0,  2), epsilon);
	EXPECT_NEAR(H( 3), mlcpPhysicsProblem.H(0,  3), epsilon);
	EXPECT_NEAR(H( 4), mlcpPhysicsProblem.H(0,  4), epsilon);
	EXPECT_NEAR(H( 5), mlcpPhysicsProblem.H(0,  5), epsilon);
	EXPECT_NEAR(H( 6), mlcpPhysicsProblem.H(0,  6), epsilon);
	EXPECT_NEAR(H( 7), mlcpPhysicsProblem.H(0,  7), epsilon);
	EXPECT_NEAR(H( 8), mlcpPhysicsProblem.H(0,  8), epsilon);
	EXPECT_NEAR(H( 9), mlcpPhysicsProblem.H(0,  9), epsilon);
	EXPECT_NEAR(H(10), mlcpPhysicsProblem.H(0, 10), epsilon);
	EXPECT_NEAR(H(11), mlcpPhysicsProblem.H(0, 11), epsilon);
	EXPECT_NEAR(H(12), mlcpPhysicsProblem.H(0, 12), epsilon);
	EXPECT_NEAR(H(13), mlcpPhysicsProblem.H(0, 13), epsilon);
	EXPECT_NEAR(H(14), mlcpPhysicsProblem.H(0, 14), epsilon);
	EXPECT_NEAR(H(15), mlcpPhysicsProblem.H(0, 15), epsilon);
	EXPECT_NEAR(H(16), mlcpPhysicsProblem.H(0, 16), epsilon);
	EXPECT_NEAR(H(17), mlcpPhysicsProblem.H(0, 17), epsilon);

	// CHt
	Eigen::Matrix<double, 18, 18> C = dt * m_fem->computeM(*m_fem->getPreviousState()).inverse();

	EXPECT_NEAR(C.row( 0) * H, mlcpPhysicsProblem.CHt( 0, 0), epsilon);
	EXPECT_NEAR(C.row( 1) * H, mlcpPhysicsProblem.CHt( 1, 0), epsilon);
	EXPECT_NEAR(C.row( 2) * H, mlcpPhysicsProblem.CHt( 2, 0), epsilon);
	EXPECT_NEAR(C.row( 3) * H, mlcpPhysicsProblem.CHt( 3, 0), epsilon);
	EXPECT_NEAR(C.row( 4) * H, mlcpPhysicsProblem.CHt( 4, 0), epsilon);
	EXPECT_NEAR(C.row( 5) * H, mlcpPhysicsProblem.CHt( 5, 0), epsilon);
	EXPECT_NEAR(C.row( 6) * H, mlcpPhysicsProblem.CHt( 6, 0), epsilon);
	EXPECT_NEAR(C.row( 7) * H, mlcpPhysicsProblem.CHt( 7, 0), epsilon);
	EXPECT_NEAR(C.row( 8) * H, mlcpPhysicsProblem.CHt( 8, 0), epsilon);
	EXPECT_NEAR(C.row( 9) * H, mlcpPhysicsProblem.CHt( 9, 0), epsilon);
	EXPECT_NEAR(C.row(10) * H, mlcpPhysicsProblem.CHt(10, 0), epsilon);
	EXPECT_NEAR(C.row(11) * H, mlcpPhysicsProblem.CHt(11, 0), epsilon);
	EXPECT_NEAR(C.row(12) * H, mlcpPhysicsProblem.CHt(12, 0), epsilon);
	EXPECT_NEAR(C.row(13) * H, mlcpPhysicsProblem.CHt(13, 0), epsilon);
	EXPECT_NEAR(C.row(14) * H, mlcpPhysicsProblem.CHt(14, 0), epsilon);
	EXPECT_NEAR(C.row(15) * H, mlcpPhysicsProblem.CHt(15, 0), epsilon);
	EXPECT_NEAR(C.row(16) * H, mlcpPhysicsProblem.CHt(16, 0), epsilon);
	EXPECT_NEAR(C.row(17) * H, mlcpPhysicsProblem.CHt(17, 0), epsilon);

	// HCHt
	EXPECT_NEAR((H.transpose() * C * H)(0, 0), mlcpPhysicsProblem.A(0, 0), epsilon);

	EXPECT_EQ(0u, mlcpPhysicsProblem.constraintTypes.size());
}

TEST_F(Fem3DRepresentationContactTests, BuildMlcpIndiciesTest)
{
	// This test verifies the build mlcp function works given a hypothetical, preexisting mlcp problem.

	auto implementation = std::make_shared<Fem3DRepresentationContact>();

	// Initialize MLCP
	MlcpPhysicsProblem mlcpPhysicsProblem;
	initializeMlcp(&mlcpPhysicsProblem, m_fem->getNumDof() + 5, 2);

	// Suppose 5 dof and 1 constraint are defined elsewhere.  Then H, CHt, HCHt, and b are prebuilt.
	Eigen::Matrix<double, 1, 5> localH;
	localH <<
		0.9478,  -0.3807,  0.5536, -0.6944,  0.1815;
	mlcpPhysicsProblem.H.block<1, 5>(0, 0) = localH;

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

	// Place mass-spring at 5th dof and 1th constraint.
	unsigned int indexOfRepresentation = 5;
	unsigned int indexOfConstraint = 1;

	// Apply constraint to all nodes of an fem.
	FemRepresentationCoordinate coord(1, Vector4d(0.25, 0.33, 0.28, 0.14));
	setContactAt(coord);

	implementation->build(dt, m_constraintData, m_localization,
		&mlcpPhysicsProblem, indexOfRepresentation, indexOfConstraint, SurgSim::Physics::CONSTRAINT_POSITIVE_SIDE);

	EXPECT_NEAR(-9.81 * dt * dt * m_n[1], mlcpPhysicsProblem.b[indexOfConstraint], epsilon);

	Eigen::Matrix<double, 18, 1> H;
	H << 0.25 * m_n[0],
		 0.25 * m_n[1],
		 0.25 * m_n[2],
		 0.33 * m_n[0],
		 0.33 * m_n[1],
		 0.33 * m_n[2],
		 0.0,
		 0.0,
		 0.0,
		 0.28 * m_n[0],
		 0.28 * m_n[1],
		 0.28 * m_n[2],
		 0.14 * m_n[0],
		 0.14 * m_n[1],
		 0.14 * m_n[2],
		 0.0,
		 0.0,
		 0.0;

	EXPECT_NEAR(H( 0), mlcpPhysicsProblem.H(indexOfConstraint, indexOfRepresentation +  0), epsilon);
	EXPECT_NEAR(H( 1), mlcpPhysicsProblem.H(indexOfConstraint, indexOfRepresentation +  1), epsilon);
	EXPECT_NEAR(H( 2), mlcpPhysicsProblem.H(indexOfConstraint, indexOfRepresentation +  2), epsilon);
	EXPECT_NEAR(H( 3), mlcpPhysicsProblem.H(indexOfConstraint, indexOfRepresentation +  3), epsilon);
	EXPECT_NEAR(H( 4), mlcpPhysicsProblem.H(indexOfConstraint, indexOfRepresentation +  4), epsilon);
	EXPECT_NEAR(H( 5), mlcpPhysicsProblem.H(indexOfConstraint, indexOfRepresentation +  5), epsilon);
	EXPECT_NEAR(H( 6), mlcpPhysicsProblem.H(indexOfConstraint, indexOfRepresentation +  6), epsilon);
	EXPECT_NEAR(H( 7), mlcpPhysicsProblem.H(indexOfConstraint, indexOfRepresentation +  7), epsilon);
	EXPECT_NEAR(H( 8), mlcpPhysicsProblem.H(indexOfConstraint, indexOfRepresentation +  8), epsilon);
	EXPECT_NEAR(H( 9), mlcpPhysicsProblem.H(indexOfConstraint, indexOfRepresentation +  9), epsilon);
	EXPECT_NEAR(H(10), mlcpPhysicsProblem.H(indexOfConstraint, indexOfRepresentation + 10), epsilon);
	EXPECT_NEAR(H(11), mlcpPhysicsProblem.H(indexOfConstraint, indexOfRepresentation + 11), epsilon);
	EXPECT_NEAR(H(12), mlcpPhysicsProblem.H(indexOfConstraint, indexOfRepresentation + 12), epsilon);
	EXPECT_NEAR(H(13), mlcpPhysicsProblem.H(indexOfConstraint, indexOfRepresentation + 13), epsilon);
	EXPECT_NEAR(H(14), mlcpPhysicsProblem.H(indexOfConstraint, indexOfRepresentation + 14), epsilon);
	EXPECT_NEAR(H(15), mlcpPhysicsProblem.H(indexOfConstraint, indexOfRepresentation + 15), epsilon);
	EXPECT_NEAR(H(16), mlcpPhysicsProblem.H(indexOfConstraint, indexOfRepresentation + 16), epsilon);
	EXPECT_NEAR(H(17), mlcpPhysicsProblem.H(indexOfConstraint, indexOfRepresentation + 17), epsilon);

	// CHt
	Eigen::Matrix<double, 18, 18> C = dt * m_fem->computeM(*m_fem->getPreviousState()).inverse();

	EXPECT_NEAR(C.row( 0) * H, mlcpPhysicsProblem.CHt(indexOfRepresentation +  0, indexOfConstraint), epsilon);
	EXPECT_NEAR(C.row( 1) * H, mlcpPhysicsProblem.CHt(indexOfRepresentation +  1, indexOfConstraint), epsilon);
	EXPECT_NEAR(C.row( 2) * H, mlcpPhysicsProblem.CHt(indexOfRepresentation +  2, indexOfConstraint), epsilon);
	EXPECT_NEAR(C.row( 3) * H, mlcpPhysicsProblem.CHt(indexOfRepresentation +  3, indexOfConstraint), epsilon);
	EXPECT_NEAR(C.row( 4) * H, mlcpPhysicsProblem.CHt(indexOfRepresentation +  4, indexOfConstraint), epsilon);
	EXPECT_NEAR(C.row( 5) * H, mlcpPhysicsProblem.CHt(indexOfRepresentation +  5, indexOfConstraint), epsilon);
	EXPECT_NEAR(C.row( 6) * H, mlcpPhysicsProblem.CHt(indexOfRepresentation +  6, indexOfConstraint), epsilon);
	EXPECT_NEAR(C.row( 7) * H, mlcpPhysicsProblem.CHt(indexOfRepresentation +  7, indexOfConstraint), epsilon);
	EXPECT_NEAR(C.row( 8) * H, mlcpPhysicsProblem.CHt(indexOfRepresentation +  8, indexOfConstraint), epsilon);
	EXPECT_NEAR(C.row( 9) * H, mlcpPhysicsProblem.CHt(indexOfRepresentation +  9, indexOfConstraint), epsilon);
	EXPECT_NEAR(C.row(10) * H, mlcpPhysicsProblem.CHt(indexOfRepresentation + 10, indexOfConstraint), epsilon);
	EXPECT_NEAR(C.row(11) * H, mlcpPhysicsProblem.CHt(indexOfRepresentation + 11, indexOfConstraint), epsilon);
	EXPECT_NEAR(C.row(12) * H, mlcpPhysicsProblem.CHt(indexOfRepresentation + 12, indexOfConstraint), epsilon);
	EXPECT_NEAR(C.row(13) * H, mlcpPhysicsProblem.CHt(indexOfRepresentation + 13, indexOfConstraint), epsilon);
	EXPECT_NEAR(C.row(14) * H, mlcpPhysicsProblem.CHt(indexOfRepresentation + 14, indexOfConstraint), epsilon);
	EXPECT_NEAR(C.row(15) * H, mlcpPhysicsProblem.CHt(indexOfRepresentation + 15, indexOfConstraint), epsilon);
	EXPECT_NEAR(C.row(16) * H, mlcpPhysicsProblem.CHt(indexOfRepresentation + 16, indexOfConstraint), epsilon);
	EXPECT_NEAR(C.row(17) * H, mlcpPhysicsProblem.CHt(indexOfRepresentation + 17, indexOfConstraint), epsilon);

	// HCHt
	EXPECT_NEAR((H.transpose() * C * H)(0, 0), mlcpPhysicsProblem.A(indexOfConstraint, indexOfConstraint), epsilon);

	EXPECT_EQ(0u, mlcpPhysicsProblem.constraintTypes.size());
}
