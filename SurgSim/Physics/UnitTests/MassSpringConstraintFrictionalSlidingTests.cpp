// This file is a part of the OpenSurgSim project.
// Copyright 2020, SimQuest Solutions Inc.
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
#include "SurgSim/Physics/LinearSpring.h"
#include "SurgSim/Physics/Mass.h"
#include "SurgSim/Physics/MassSpringConstraintFrictionalSliding.h"
#include "SurgSim/Physics/MassSpringLocalization.h"
#include "SurgSim/Physics/MassSpringRepresentation.h"
#include "SurgSim/Physics/MlcpPhysicsProblem.h"
#include "SurgSim/Physics/SlidingConstraintData.h"
#include "SurgSim/Physics/UnitTests/EigenGtestAsserts.h"
#include "SurgSim/Physics/UnitTests/MockObjects.h"

using SurgSim::DataStructures::IndexedLocalCoordinate;
using SurgSim::Framework::Runtime;
using SurgSim::Physics::Localization;
using SurgSim::Physics::MassSpringConstraintFrictionalSliding;
using SurgSim::Physics::MassSpringRepresentation;
using SurgSim::Physics::MlcpPhysicsProblem;
using SurgSim::Physics::MockMassSpring;
using SurgSim::Physics::SlidingConstraintData;
using SurgSim::Math::Vector3d;
using SurgSim::Math::Vector2d;

namespace
{
	const double epsilon = 1e-10;
	const double dt = 1e-3;
};

class MassSpringConstraintFrictionalSlidingTests : public ::testing::Test
{
public:
	void SetUp()
	{
		// Define plane with normal 'n' pointing against gravity.
		m_slidingDirection = Vector3d(0.8539, 0.6289, -0.9978);
		m_slidingDirection.normalize();

		// Create mock FEM
		m_massSpring = std::make_shared<MockMassSpring>();
		auto state = std::make_shared<SurgSim::Math::OdeState>();
		state->setNumDof(3, 5);

		state->getPositions().segment<3>(0 * 3) = Vector3d(0.30, -0.57,  0.40);
		state->getPositions().segment<3>(1 * 3) = Vector3d(0.06,  0.63, -0.32);
		state->getPositions().segment<3>(2 * 3) = Vector3d(-0.91,  0.72,  0.72);
		state->getPositions().segment<3>(3 * 3) = Vector3d(0.35,  0.52,  0.50);
		state->getPositions().segment<3>(4 * 3) = Vector3d(1.14,  0.66,  0.71);
		
		double mass = 0.1;
		for (int i = 0; i < state->getNumNodes(); ++i)
		{
			m_massSpring->addMass(std::make_shared<SurgSim::Physics::Mass>(mass));
		}

		double stiffnessStretching = 100.0;
		double dampingStretching = 1.5;
		for (int i = 0; i < state->getNumNodes() - 1; ++i)
		{
			m_massSpring->addSpring(std::make_shared<SurgSim::Physics::LinearSpring>(state, i, i + 1,
				stiffnessStretching, dampingStretching));
		}

		double stiffnessBending = 8.0;
		double dampingBending = 1.3;
		for (int i = 0; i < state->getNumNodes() - 2; ++i)
		{
			m_massSpring->addSpring(std::make_shared<SurgSim::Physics::LinearSpring>(state, i, i + 2,
				stiffnessBending, dampingBending));
		}

		m_massSpring->setInitialState(state);
		m_massSpring->setIntegrationScheme(SurgSim::Math::IntegrationScheme::INTEGRATIONSCHEME_EULER_EXPLICIT_MODIFIED);
		m_massSpring->setLocalActive(true);

		m_massSpring->initialize(std::make_shared<Runtime>());
		m_massSpring->wakeUp();

		// Update model by one timestep
		m_massSpring->beforeUpdate(dt);
		m_massSpring->update(dt);
	}

	void setSlidingConstraintAt(size_t node)
	{
		SurgSim::DataStructures::Location location;
		location.index = node;
		m_localization = m_massSpring->createLocalization(location);
		m_constraintData.setSlidingDirection(m_localization->calculatePosition(0.0), m_slidingDirection);
	}

	std::shared_ptr<MockMassSpring> m_massSpring;
	std::shared_ptr<Localization> m_localization;

	Vector3d m_slidingDirection;
	SlidingConstraintData m_constraintData;
};

TEST_F(MassSpringConstraintFrictionalSlidingTests, ConstructorTest)
{
	ASSERT_NO_THROW(
	{
		MassSpringConstraintFrictionalSliding massSpringContact;
	});
}

TEST_F(MassSpringConstraintFrictionalSlidingTests, ConstraintConstantsTest)
{
	auto implementation = std::make_shared<MassSpringConstraintFrictionalSliding>();

	EXPECT_EQ(SurgSim::Physics::FRICTIONAL_SLIDING, implementation->getConstraintType());
	EXPECT_EQ(5u, implementation->getNumDof());
}

TEST_F(MassSpringConstraintFrictionalSlidingTests, BuildMlcpTest)
{
	// This test verifies the build mlcp function works for a simple case.
	auto implementation = std::make_shared<MassSpringConstraintFrictionalSliding>();

	// Initialize MLCP
	auto mlcpPhysicsProblem = MlcpPhysicsProblem::Zero(m_massSpring->getNumDof(), implementation->getNumDof(), 1);

	// Apply constraint purely to the first node.
	setSlidingConstraintAt(0);

	implementation->build(dt, m_constraintData, m_localization,
		&mlcpPhysicsProblem, 0, 0, SurgSim::Physics::CONSTRAINT_POSITIVE_SIDE);

	Eigen::Matrix<double, 5, 15> H = Eigen::Matrix<double, 5, 15>::Zero();
	H.block<1, 3>(0, 0) = (dt * m_constraintData.getNormals()[0]).eval();
	H.block<1, 3>(1, 0) = (dt * m_constraintData.getNormals()[1]).eval();
	H.block<1, 3>(2, 0) = (dt * Vector3d::UnitX()).eval();
	H.block<1, 3>(3, 0) = (dt * Vector3d::UnitY()).eval();
	H.block<1, 3>(4, 0) = (dt * Vector3d::UnitZ()).eval();

	EXPECT_NEAR_EIGEN(H, mlcpPhysicsProblem.H, epsilon);

	// C = dt * m^{-1}
	m_massSpring->updateFMDK(*(m_massSpring->getPreviousState()), SurgSim::Math::ODEEQUATIONUPDATE_M);
	Eigen::Matrix<double, 15, 15> C = dt * m_massSpring->getM().toDense().inverse();

	EXPECT_NEAR_EIGEN(C * H.transpose(), mlcpPhysicsProblem.CHt, epsilon);

	EXPECT_NEAR_EIGEN(H * C * H.transpose(), mlcpPhysicsProblem.A, epsilon);

	EXPECT_DOUBLE_EQ(0.5, mlcpPhysicsProblem.mu[0]);

	EXPECT_EQ(0u, mlcpPhysicsProblem.constraintTypes.size());
}

TEST_F(MassSpringConstraintFrictionalSlidingTests, BuildMlcpCoordinateTest)
{
	// This test verifies the build mlcp function works for a more complicated case with different nodes.
	auto implementation = std::make_shared<MassSpringConstraintFrictionalSliding>();

	// Initialize MLCP
	auto mlcpPhysicsProblem = MlcpPhysicsProblem::Zero(m_massSpring->getNumDof(), implementation->getNumDof(), 1);

	// Apply constraint to the second node.
	setSlidingConstraintAt(1);

	implementation->build(dt, m_constraintData, m_localization,
		&mlcpPhysicsProblem, 0, 0, SurgSim::Physics::CONSTRAINT_POSITIVE_SIDE);

	Eigen::Matrix<double, 5, 15> H = Eigen::Matrix<double, 5, 15>::Zero();
	H.block<1, 3>(0, 3) = (dt * m_constraintData.getNormals()[0]).eval();
	H.block<1, 3>(1, 3) = (dt * m_constraintData.getNormals()[1]).eval();
	H.block<1, 3>(2, 3) = (dt * Vector3d::UnitX()).eval();
	H.block<1, 3>(3, 3) = (dt * Vector3d::UnitY()).eval();
	H.block<1, 3>(4, 3) = (dt * Vector3d::UnitZ()).eval();

	EXPECT_NEAR_EIGEN(H, mlcpPhysicsProblem.H, epsilon);

	// C = dt * m^{-1}
	m_massSpring->updateFMDK(*(m_massSpring->getPreviousState()), SurgSim::Math::ODEEQUATIONUPDATE_M);
	Eigen::Matrix<double, 15, 15> C = dt * m_massSpring->getM().toDense().inverse();

	EXPECT_NEAR_EIGEN(C * H.transpose(), mlcpPhysicsProblem.CHt, epsilon);

	EXPECT_NEAR_EIGEN(H * C * H.transpose(), mlcpPhysicsProblem.A, epsilon);

	EXPECT_DOUBLE_EQ(0.5, mlcpPhysicsProblem.mu[0]);

	EXPECT_EQ(0u, mlcpPhysicsProblem.constraintTypes.size());
}

TEST_F(MassSpringConstraintFrictionalSlidingTests, BuildMlcpIndiciesTest)
{
	// This test verifies the build mlcp function works given a hypothetical, preexisting mlcp problem.

	auto implementation = std::make_shared<MassSpringConstraintFrictionalSliding>();

	// Initialize MLCP
	auto mlcpPhysicsProblem =
		MlcpPhysicsProblem::Zero(m_massSpring->getNumDof() + 5, implementation->getNumDof() + 1, 2);

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

	// Place the MassSpring at 5th dof (1 or multiple representations exist before, covering a total of 5 dof)
	// and the constraint at the index 1 (1 atomic constraint exists before this one)
	size_t indexOfRepresentation = 5;
	size_t indexOfConstraint = 1;

	// Apply constraint to the second node.
	setSlidingConstraintAt(1);

	implementation->build(dt, m_constraintData, m_localization,
		&mlcpPhysicsProblem, indexOfRepresentation, indexOfConstraint,
		SurgSim::Physics::CONSTRAINT_POSITIVE_SIDE);

	Eigen::Matrix<double, 5, 15> H = Eigen::Matrix<double, 5, 15>::Zero();
	H.block<1, 3>(0, 3) = (dt * m_constraintData.getNormals()[0]).eval();
	H.block<1, 3>(1, 3) = (dt * m_constraintData.getNormals()[1]).eval();
	H.block<1, 3>(2, 3) = (dt * Vector3d::UnitX()).eval();
	H.block<1, 3>(3, 3) = (dt * Vector3d::UnitY()).eval();
	H.block<1, 3>(4, 3) = (dt * Vector3d::UnitZ()).eval();

	EXPECT_NEAR_EIGEN(H, mlcpPhysicsProblem.H.block(indexOfConstraint, indexOfRepresentation, 5, 15), epsilon);

	m_massSpring->updateFMDK(*(m_massSpring->getPreviousState()), SurgSim::Math::ODEEQUATIONUPDATE_M);
	Eigen::Matrix<double, 15, 15> C = dt * m_massSpring->getM().toDense().inverse();

	EXPECT_NEAR_EIGEN(
		C * H.transpose(), mlcpPhysicsProblem.CHt.block(indexOfRepresentation, indexOfConstraint, 15, 5), epsilon);

	EXPECT_NEAR_EIGEN(
		H * C * H.transpose(), mlcpPhysicsProblem.A.block(indexOfConstraint, indexOfConstraint, 5, 5), epsilon);

	EXPECT_DOUBLE_EQ(0.5, mlcpPhysicsProblem.mu[1]);

	EXPECT_EQ(0u, mlcpPhysicsProblem.constraintTypes.size());
}
