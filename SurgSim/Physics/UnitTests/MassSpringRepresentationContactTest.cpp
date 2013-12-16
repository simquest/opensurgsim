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
	// The constraint violation is the position from the plane projected onto the constraint.  Note, we have defined the
	// plane to intersect with p(0).  The constraint is
	//      U(t) = n^t.(p(t) - p(0)) >= 0
	//
	// U(1) = n^t.(p(1) - p(0))
	//      = (nx, ny, nz)^t.(0, -g*dt^2, 0)
	//      = -g*dt^2*ny
	EXPECT_NEAR(-9.81 * dt * dt * m_n[1], mlcpPhysicsProblem.b[0], epsilon);

	// By definition, H = dU/dp.
	//
	// dU/dt = (dU/dp).(dp/dt)
	//       = H.(dp/dt)
	//
	// dp = p(1) - p(0)
	// => dp/dt = (p(1) - p(0)) / dt
	//
	// dU/dt = (U(1) - U(0)) / dt      [Note U(0) = 0]
	//       = n^t.(p(1) - p(0)) / dt
	//       = n^t.(dp/dt)
	// => H = n^t
	EXPECT_NEAR(m_n[0], mlcpPhysicsProblem.H(0, 0), epsilon);
	EXPECT_NEAR(m_n[1], mlcpPhysicsProblem.H(0, 1), epsilon);
	EXPECT_NEAR(m_n[2], mlcpPhysicsProblem.H(0, 2), epsilon);

	// We define C as the matrix which transforms F -> v (which differs from treatments which define it as F -> p)
	// v(1) = v(0) + a(0)*dt
	//      = v(0) + 1/m*F(0)*dt
	// => (v(1) - v(0)) = [dt/m] * F(0)
	//
	// Therefore C = dt/m
	//
	// We can directly calculate
	// CHt = C * H^t
	//     = (dt/m) * (nx, ny, nz)
	EXPECT_NEAR(dt / m_massPerNode * m_n[0], mlcpPhysicsProblem.CHt(0, 0), epsilon);
	EXPECT_NEAR(dt / m_massPerNode * m_n[1], mlcpPhysicsProblem.CHt(1, 0), epsilon);
	EXPECT_NEAR(dt / m_massPerNode * m_n[2], mlcpPhysicsProblem.CHt(2, 0), epsilon);

	// And finally,
	// HCHt = [nx ny nz] * (dt/m) * (nx, ny, nz)
	//      = (dt/m) * (nx*nx + ny*ny + nz*nz)
	double calculatedA = mlcpPhysicsProblem.A.block<1, 1>(0, 0)[0]; // VS intellisense error workaround
	EXPECT_NEAR(dt / m_massPerNode * (m_n[0] * m_n[0] + m_n[1] * m_n[1] + m_n[2] * m_n[2]), calculatedA, epsilon);

	// ConstraintTypes should contain 0 entry as it is setup by the constraint and not the ConstraintImplementation
	// This way, the constraint can verify that both ConstraintImplementation are the same type
	EXPECT_EQ(0u, mlcpPhysicsProblem.constraintTypes.size());
}

};  //  namespace Physics
};  //  namespace SurgSim
