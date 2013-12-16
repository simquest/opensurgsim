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
#include "SurgSim/Physics/ContactConstraintData.h"
#include "SurgSim/Blocks/MassSpring1DRepresentation.h"
#include "SurgSim/Physics/MassSpringRepresentationContact.h"
#include "SurgSim/Physics/MassSpringRepresentationLocalization.h"
#include "SurgSim/Physics/MlcpPhysicsProblem.h"

#include "SurgSim/Math/RigidTransform.h"
#include "SurgSim/Math/Vector.h"

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

class MassSpringRepresentationContactTest : public ::testing::Test
{
public:
	void SetUp() {
		// Define plane with normal 'n' pointing against gravity.
		m_n = Vector3d(0.8539, 0.6289, -0.9978);
		m_n.normalize();

		// Place spring at random location.
		m_extremities[0] = Vector3d(0.8799, -0.0871, 0.7468);
		m_extremities[1] = Vector3d(0.9040, -0.7074, 0.6783);

		// Define physics representation of mass-spring using 1d helper function.
		m_massSpring = std::make_shared<SurgSim::Blocks::MassSpring1DRepresentation>("MassSpring");
		unsigned int numNodesPerDim[1] = {2};
		m_massPerNode = 0.137;
		std::vector<unsigned int> boundaryConditions;
		m_massSpring->init1D(
			m_extremities,
			numNodesPerDim,
			boundaryConditions,
			numNodesPerDim[0] * m_massPerNode, // total mass (in Kg)
			100.0, // Stiffness stretching
			0.0, // Damping stretching
			10.0, // Stiffness bending
			0.0); // Damping bending

		// Update position in only 1 timestep
		// Forward euler for velocity, backward euler for position
		m_massSpring->setIntegrationScheme(SurgSim::Math::IntegrationScheme::INTEGRATIONSCHEME_MODIFIED_EXPLICIT_EULER);

		m_massSpring->setIsActive(true);

		// Update model by one timestep
		m_massSpring->beforeUpdate(dt);
		m_massSpring->update(dt);

		// Create localization helper class
		m_localization = std::make_shared<MassSpringRepresentationLocalization>(m_massSpring);
	}

	void setContactAtNode(size_t nodeId) 
	{
		m_nodeId = nodeId;
		m_localization->setLocalNode(nodeId);

		// Place plane at nodeId
		double distance = -m_extremities[nodeId].dot(m_n);
		m_constraintData.setPlaneEquation(m_n, distance);
	}

	Vector3d m_n;
	ContactConstraintData m_constraintData;
	size_t m_nodeId;

	std::shared_ptr<SurgSim::Blocks::MassSpring1DRepresentation> m_massSpring;
	double m_massPerNode;
	std::array<Vector3d, 2> m_extremities;

	std::shared_ptr<MassSpringRepresentationLocalization> m_localization;
};

TEST_F(MassSpringRepresentationContactTest, ConstructorTest)
{
	ASSERT_NO_THROW({ MassSpringRepresentationContact massSpring; });

	ASSERT_NE(nullptr, std::make_shared<MassSpringRepresentationContact>());
}

TEST_F(MassSpringRepresentationContactTest, ConstraintConstantsTest)
{
	auto implementation = std::make_shared<MassSpringRepresentationContact>();

	EXPECT_EQ(SurgSim::Math::MLCP_UNILATERAL_3D_FRICTIONLESS_CONSTRAINT, implementation->getMlcpConstraintType());
	EXPECT_EQ(SurgSim::Physics::REPRESENTATION_TYPE_MASSSPRING, implementation->getRepresentationType());
	EXPECT_EQ(1u, implementation->getNumDof());
}

void initializeMlcp(MlcpPhysicsProblem *mlcpPhysicsProblem, size_t numDof, size_t numConstraints)
{
	// Resize and zero all Eigen types
	mlcpPhysicsProblem->A.resize(numConstraints, numConstraints);
	mlcpPhysicsProblem->A.setZero();
	mlcpPhysicsProblem->b.resize(numConstraints);
	mlcpPhysicsProblem->b.setZero();
	mlcpPhysicsProblem->mu.resize(numConstraints);
	mlcpPhysicsProblem->mu.setZero();
	mlcpPhysicsProblem->CHt.resize(numDof, numConstraints);
	mlcpPhysicsProblem->CHt.setZero();
	mlcpPhysicsProblem->H.resize(numConstraints, numDof);
	mlcpPhysicsProblem->H.setZero();
	// Empty all std::vector types
	mlcpPhysicsProblem->constraintTypes.clear();
}

TEST_F(MassSpringRepresentationContactTest, BuildMlcpTest)
{
	// Define constraint (frictionless non-penetration)
	auto implementation = std::make_shared<MassSpringRepresentationContact>();

	// Initialize MLCP
	MlcpPhysicsProblem mlcpPhysicsProblem;
	initializeMlcp(&mlcpPhysicsProblem, m_massSpring->getNumDof(), 1);

	// Build MLCP for 0th node
	setContactAtNode(0);

	implementation->build(dt, m_constraintData, m_localization,
		&mlcpPhysicsProblem, 0, 0, SurgSim::Physics::CONSTRAINT_POSITIVE_SIDE);

	// Expected results.
	// F = ma
	// => mg = ma
	// => dv/dt = -g
	//
	// Modified Explicit Euler:
	// v(1) = v(0) + dv/dt|(t=0)*dt
	//      = -g*dt
	//
	// x(1) = x(0) + dx/dt|(t=1)*dt
	//      = v(1)*dt
	//      = -g*dt^2

	EXPECT_NEAR(d - (9.81 * dt * dt), mlcpPhysicsProblem.b[0], epsilon);

	// Constraint U = nt.p + d >= 0, so
	// H = dU/dp
	//   = dt.[nx  ny  nz]
	EXPECT_NEAR(dt * n[0]   , mlcpPhysicsProblem.H(0, 0), epsilon);
	EXPECT_NEAR(dt * n[1]   , mlcpPhysicsProblem.H(0, 1), epsilon);
	EXPECT_NEAR(dt * n[2]   , mlcpPhysicsProblem.H(0, 2), epsilon);

	// Test CHt
	const Eigen::Matrix<double, 6, 6> &C = massSpring->getComplianceMatrix();
	EXPECT_NEAR((C.block<1,3>(0, 0) * n * dt)[0], mlcpPhysicsProblem.CHt(0, 0), epsilon);
	EXPECT_NEAR((C.block<1,3>(1, 0) * n * dt)[0], mlcpPhysicsProblem.CHt(1, 0), epsilon);
	EXPECT_NEAR((C.block<1,3>(2, 0) * n * dt)[0], mlcpPhysicsProblem.CHt(2, 0), epsilon);
	EXPECT_NEAR((C.block<1,3>(3, 0) * n * dt)[0], mlcpPhysicsProblem.CHt(3, 0), epsilon);
	EXPECT_NEAR((C.block<1,3>(4, 0) * n * dt)[0], mlcpPhysicsProblem.CHt(4, 0), epsilon);
	EXPECT_NEAR((C.block<1,3>(5, 0) * n * dt)[0], mlcpPhysicsProblem.CHt(5, 0), epsilon);

	// Test HCHt
	Eigen::Matrix<double, 1, 6> H;
	H.block<1, 3>(0, 0) = dt * n;
	H.block<1, 3>(0, 3) = Eigen::Matrix<double, 1, 3>::Zero();

	EXPECT_TRUE((H * C * H.transpose()).isApprox(mlcpPhysicsProblem.A, epsilon));

	// ConstraintTypes should contain 0 entry as it is setup by the constraint and not the ConstraintImplementation
	// This way, the constraint can verify that both ConstraintImplementation are the same type
	EXPECT_EQ(0u, mlcpPhysicsProblem.constraintTypes.size());
}

};  //  namespace Physics
};  //  namespace SurgSim
