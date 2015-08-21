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

#include <memory>

#include <gtest/gtest.h>

#include "SurgSim/Blocks/MassSpring1DRepresentation.h"
#include "SurgSim/Framework/Runtime.h"
#include "SurgSim/Math/RigidTransform.h"
#include "SurgSim/Math/SparseMatrix.h"
#include "SurgSim/Math/Vector.h"
#include "SurgSim/Physics/MassSpringConstraintFixedPoint.h"
#include "SurgSim/Physics/MassSpringLocalization.h"
#include "SurgSim/Physics/MlcpPhysicsProblem.h"

namespace
{
const double epsilon = 1e-10;
const double dt = 1e-3;
};

namespace SurgSim
{
namespace Physics
{
using SurgSim::Math::Vector3d;

class MassSpringConstraintFixedPointTest : public ::testing::Test
{
public:
	void SetUp()
	{
		// Place spring at random location.
		m_extremities.push_back(Vector3d(0.8799, -0.0871, 0.7468));
		m_extremities.push_back(Vector3d(0.9040, -0.7074, 0.6783));

		// Define physics representation of mass-spring using 1d helper function.
		m_massSpring = std::make_shared<SurgSim::Blocks::MassSpring1DRepresentation>("MassSpring");
		size_t numNodesPerDim[1] = {2};
		m_massPerNode = 0.137;
		std::vector<size_t> boundaryConditions;
		m_massSpring->init1D(
			m_extremities,
			boundaryConditions,
			numNodesPerDim[0] * m_massPerNode, // total mass (in Kg)
			100.0, // Stiffness stretching
			0.0, // Damping stretching
			10.0, // Stiffness bending
			0.0); // Damping bending

		// Update position in only 1 timestep
		// Forward Euler for velocity, backward Euler for position
		m_massSpring->setIntegrationScheme(SurgSim::Math::IntegrationScheme::INTEGRATIONSCHEME_EULER_EXPLICIT_MODIFIED);

		m_massSpring->initialize(std::make_shared<SurgSim::Framework::Runtime>());
		m_massSpring->wakeUp();

		// Update model by one timestep
		m_massSpring->beforeUpdate(dt);
		m_massSpring->update(dt);

		// Create localization helper class
		m_localization = std::make_shared<MassSpringLocalization>(m_massSpring);
	}

	void setConstraintAtNode(size_t nodeId)
	{
		m_nodeId = nodeId;
		m_localization->setLocalNode(nodeId);
	}

	ConstraintData m_constraintData;
	size_t m_nodeId;

	std::shared_ptr<SurgSim::Blocks::MassSpring1DRepresentation> m_massSpring;
	double m_massPerNode;
	std::vector<Vector3d> m_extremities;

	std::shared_ptr<MassSpringLocalization> m_localization;
};

TEST_F(MassSpringConstraintFixedPointTest, ConstructorTest)
{
	ASSERT_NO_THROW({ MassSpringConstraintFixedPoint constraint; });

	ASSERT_NE(nullptr, std::make_shared<MassSpringConstraintFixedPoint>());
}

TEST_F(MassSpringConstraintFixedPointTest, ConstraintConstantsTest)
{
	auto implementation = std::make_shared<MassSpringConstraintFixedPoint>();

	EXPECT_EQ(SurgSim::Physics::FIXED_3DPOINT, implementation->getConstraintType());
	EXPECT_EQ(3u, implementation->getNumDof());
}

TEST_F(MassSpringConstraintFixedPointTest, BuildMlcpTest)
{
	// Define constraint (frictionless non-penetration)
	auto implementation = std::make_shared<MassSpringConstraintFixedPoint>();

	// Initialize MLCP
	MlcpPhysicsProblem mlcpPhysicsProblem = MlcpPhysicsProblem::Zero(m_massSpring->getNumDof(), 3, 1);

	// Build MLCP for 0th node
	setConstraintAtNode(0);

	implementation->build(dt, m_constraintData, m_localization,
						  &mlcpPhysicsProblem, 0, 0, SurgSim::Physics::CONSTRAINT_POSITIVE_SIDE);

	// Expected results.
	// At the initial time-step, the only force is gravity.
	//
	// F(0) = m * a(0)
	// => a(0) = 1/m * F(0)
	//         = (0, -g, 0)
	//
	// Modified Explicit Euler:
	// v(1) = v(0) + dv/dt|(t=0)*dt
	//      = v(0) + a(0)*dt
	//      = (0, -g*dt, 0)
	//
	// p(1) = p(0) + dp/dt|(t=1)*dt
	//      = p(0) + v(1)*dt
	//      = p(0) + (0, -g*dt^2, 0)
	//
	// The constraint violation is the new position. The constraint is
	//      U(t) = p(t) = 0
	// U(1) = p(1)
	Vector3d expectedViolation = m_extremities[0] - Vector3d::UnitY() * 9.81 * dt * dt;
	EXPECT_TRUE(mlcpPhysicsProblem.b.isApprox(expectedViolation)) <<
		"b = " << mlcpPhysicsProblem.b.transpose() <<
		" expected = " << (-Vector3d::UnitY() * 9.81 * dt * dt).transpose();

	// By definition, H = dU/dv.
	// U(t+dt) = p(t+dt) = p(t) + dt.v(t) [using Backward Euler integration scheme for approximation]
	//
	// dU/dv = dt.Id(3x3)
	// => H = dt.Id(3x3) for the constrained node and Zero(3x3) for all other nodes
	SurgSim::Math::Matrix H = mlcpPhysicsProblem.H;
	SurgSim::Math::Matrix expectedH = SurgSim::Math::Matrix::Zero(3, 6);
	expectedH.block<3, 3>(0, 0) = SurgSim::Math::Matrix33d::Identity() * dt;
	EXPECT_TRUE((H.isApprox(expectedH)));

	// We define C as the matrix which transforms F -> v (which differs from treatments which define it as F -> p)
	// v(1) = v(0) + a(0)*dt
	//      = v(0) + 1/m*F(0)*dt
	// => (v(1) - v(0)) = [dt/m].Id(3x3) * F(0)
	//
	// Therefore C = dt/m.Id(6x6) for the entire system
	// and
	// CH^T = dt/m.(Id(3x3)  0     )* (dt.Id(3x3)) = dt^2/m (Id(3x3))
	//             ( 0      Id(3x3))  (   0      )          (   0   )
	SurgSim::Math::Matrix expectedCHt = SurgSim::Math::Matrix::Zero(6, 3);
	expectedCHt.block<3, 3>(0, 0) = SurgSim::Math::Matrix33d::Identity() * dt * dt / m_massPerNode;
	EXPECT_TRUE(mlcpPhysicsProblem.CHt.isApprox(expectedCHt));

	// And finally,
	// HCHt = H * (dt/m).Id(6x6) * H^T = (dt.Id(3x3) 0) (dt^2/m.Id(3x3)) = dt^3/m.Id(3x3)
	//                                                  (      0       )
	EXPECT_TRUE(mlcpPhysicsProblem.A.isApprox(dt * dt * dt / m_massPerNode * SurgSim::Math::Matrix33d::Identity()));

	// ConstraintTypes should contain 0 entry as it is setup by the constraint and not the ConstraintImplementation
	// This way, the constraint can verify that both ConstraintImplementation are the same type
	EXPECT_EQ(0u, mlcpPhysicsProblem.constraintTypes.size());
}

TEST_F(MassSpringConstraintFixedPointTest, BuildMlcpIndiciesTest)
{
	auto implementation = std::make_shared<MassSpringConstraintFixedPoint>();

	MlcpPhysicsProblem mlcpPhysicsProblem = MlcpPhysicsProblem::Zero(11, 6, 3);

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

	// Place mass-spring at 5th dof and 1th constraint (0-based).
	size_t indexOfRepresentation = 5;
	size_t indexOfConstraint = 1;

	setConstraintAtNode(1);

	implementation->build(dt, m_constraintData, m_localization,
		&mlcpPhysicsProblem, indexOfRepresentation, indexOfConstraint, SurgSim::Physics::CONSTRAINT_POSITIVE_SIDE);

	// b -> E -> [#constraints, 1]
	const Vector3d newPosition = m_extremities[1] - Vector3d::UnitY() * 9.81 * dt * dt;
	EXPECT_TRUE(mlcpPhysicsProblem.b.segment<3>(indexOfConstraint).isApprox(newPosition));

	// H -> [#constraints, #dof]
	SurgSim::Math::Matrix H = mlcpPhysicsProblem.H;
	SurgSim::Math::Matrix expectedH = SurgSim::Math::Matrix::Zero(3, 6);
	expectedH.block<3, 3>(0, 3) = SurgSim::Math::Matrix33d::Identity() * dt;
	EXPECT_TRUE((H.block<3, 6>(indexOfConstraint, indexOfRepresentation).isApprox(expectedH))) << H;

	// C -> [#dof, #dof]
	// CHt -> [#dof, #constraints]
	SurgSim::Math::Matrix CHt = mlcpPhysicsProblem.CHt;
	SurgSim::Math::Matrix expectedCHt = SurgSim::Math::Matrix::Zero(6, 3);
	expectedCHt.block<3, 3>(3, 0) = SurgSim::Math::Matrix33d::Identity() * dt * dt / m_massPerNode;
	EXPECT_TRUE((CHt.block(indexOfRepresentation, indexOfConstraint, 6, 3).isApprox(expectedCHt))) << CHt;

	// A -> HCHt -> [#constraints, #constraints]
	SurgSim::Math::Matrix expectedA = dt * dt * dt / m_massPerNode * SurgSim::Math::Matrix33d::Identity();
	EXPECT_TRUE(mlcpPhysicsProblem.A.block(1, 1, 3, 3).isApprox(expectedA));
}

};  //  namespace Physics
};  //  namespace SurgSim
