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
#include "SurgSim/Physics/ContactConstraintData.h"
#include "SurgSim/Physics/MassSpringContact.h"
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

class MassSpringContactTest : public ::testing::Test
{
public:
	void SetUp() {
		// Define plane normal
		m_n = Vector3d(0.8539, 0.6289, -0.9978);
		m_n.normalize();

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
		m_massSpring->setIntegrationScheme(SurgSim::Math::IntegrationScheme::INTEGRATIONSCHEME_MODIFIED_EXPLICIT_EULER);

		m_massSpring->initialize(std::make_shared<SurgSim::Framework::Runtime>());
		m_massSpring->wakeUp();

		// Update model by one timestep
		m_massSpring->beforeUpdate(dt);
		m_massSpring->update(dt);

		// Create localization helper class
		m_localization = std::make_shared<MassSpringLocalization>(m_massSpring);
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
	std::vector<Vector3d> m_extremities;

	std::shared_ptr<MassSpringLocalization> m_localization;
};

TEST_F(MassSpringContactTest, ConstructorTest)
{
	ASSERT_NO_THROW({ MassSpringContact massSpring; });

	ASSERT_NE(nullptr, std::make_shared<MassSpringContact>());
}

TEST_F(MassSpringContactTest, ConstraintConstantsTest)
{
	auto implementation = std::make_shared<MassSpringContact>();

	EXPECT_EQ(SurgSim::Math::MLCP_UNILATERAL_3D_FRICTIONLESS_CONSTRAINT, implementation->getMlcpConstraintType());
	EXPECT_EQ(1u, implementation->getNumDof());
}

TEST_F(MassSpringContactTest, BuildMlcpTest)
{
	// Define constraint (frictionless non-penetration)
	auto implementation = std::make_shared<MassSpringContact>();

	// Initialize MLCP
	MlcpPhysicsProblem mlcpPhysicsProblem = MlcpPhysicsProblem::Zero(m_massSpring->getNumDof(), 1, 1);

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
	// The constraint violation is the projection of the new position onto the constraint normal.  Note, we have
	// defined the plane to intersect with p(0).  The constraint is
	//      U(t) = n^t.p(t) >= 0
	// U(1) = n^t.p(1)
	const Vector3d newPosition = m_extremities[0] - Vector3d::UnitY() * 9.81 * dt * dt;
	EXPECT_NEAR(newPosition.dot(m_n), mlcpPhysicsProblem.b[0], epsilon);

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
	SurgSim::Math::Matrix h = mlcpPhysicsProblem.H;
	EXPECT_NEAR(m_n[0], h(0, 0), epsilon);
	EXPECT_NEAR(m_n[1], h(0, 1), epsilon);
	EXPECT_NEAR(m_n[2], h(0, 2), epsilon);

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

TEST_F(MassSpringContactTest, BuildMlcpIndiciesTest)
{
	auto implementation = std::make_shared<MassSpringContact>();

	MlcpPhysicsProblem mlcpPhysicsProblem = MlcpPhysicsProblem::Zero(11, 2, 2);

	// Suppose 5 dof and 1 constraint are defined elsewhere.  Then H, CHt, HCHt, and b are prebuilt.
	Eigen::SparseVector<double, Eigen::RowMajor, ptrdiff_t> localH;
	localH.resize(5);
	localH.reserve(5);
	localH.insert(0) = 0.9478;
	localH.insert(1) = -0.3807;
	localH.insert(2) = 0.5536;
	localH.insert(3) = -0.6944;
	localH.insert(4) = 0.1815;
	typedef Math::Dynamic::Operation<double, Eigen::RowMajor, ptrdiff_t,
		Eigen::SparseVector<double, Eigen::RowMajor, ptrdiff_t>> Operation;
	Math::blockOperation<Eigen::SparseVector<double, Eigen::RowMajor, ptrdiff_t>, double, Eigen::RowMajor, ptrdiff_t>(
		localH, 0, 0, &mlcpPhysicsProblem.H, &Operation::add);

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
	size_t indexOfRepresentation = 5;
	size_t indexOfConstraint = 1;

	setContactAtNode(1);

	implementation->build(dt, m_constraintData, m_localization,
		&mlcpPhysicsProblem, indexOfRepresentation, indexOfConstraint, SurgSim::Physics::CONSTRAINT_POSITIVE_SIDE);

	// b -> E -> [#constraints, 1]
	const Vector3d newPosition = m_extremities[1] - Vector3d::UnitY() * 9.81 * dt * dt;
	EXPECT_NEAR(newPosition.dot(m_n), mlcpPhysicsProblem.b[indexOfConstraint], epsilon);

	// H -> [#constraints, #dof]
	SurgSim::Math::Matrix h = mlcpPhysicsProblem.H;
	EXPECT_NEAR(m_n[0], h(indexOfConstraint, indexOfRepresentation + 3 * m_nodeId + 0), epsilon);
	EXPECT_NEAR(m_n[1], h(indexOfConstraint, indexOfRepresentation + 3 * m_nodeId + 1), epsilon);
	EXPECT_NEAR(m_n[2], h(indexOfConstraint, indexOfRepresentation + 3 * m_nodeId + 2), epsilon);

	// C -> [#dof, #dof]
	// CHt -> [#dof, #constraints]
	EXPECT_NEAR(dt / m_massPerNode * m_n[0],
		mlcpPhysicsProblem.CHt(indexOfRepresentation + 3 * m_nodeId + 0, indexOfConstraint), epsilon);
	EXPECT_NEAR(dt / m_massPerNode * m_n[1],
		mlcpPhysicsProblem.CHt(indexOfRepresentation + 3 * m_nodeId + 1, indexOfConstraint), epsilon);
	EXPECT_NEAR(dt / m_massPerNode * m_n[2],
		mlcpPhysicsProblem.CHt(indexOfRepresentation + 3 * m_nodeId + 2, indexOfConstraint), epsilon);

	// A -> HCHt -> [#constraints, #constraints]
	double calculatedA = mlcpPhysicsProblem.A.block<1, 1>(indexOfConstraint, indexOfConstraint)[0];
	EXPECT_NEAR(dt / m_massPerNode * (m_n[0] * m_n[0] + m_n[1] * m_n[1] + m_n[2] * m_n[2]), calculatedA, epsilon);
}

};  //  namespace Physics
};  //  namespace SurgSim
