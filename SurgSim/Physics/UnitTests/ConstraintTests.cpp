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

#include "SurgSim/DataStructures/Location.h"
#include "SurgSim/Math/Quaternion.h"
#include "SurgSim/Math/RigidTransform.h"
#include "SurgSim/Math/SphereShape.h"
#include "SurgSim/Math/Vector.h"
#include "SurgSim/Physics/Constraint.h"
#include "SurgSim/Physics/ConstraintData.h"
#include "SurgSim/Physics/ContactConstraintData.h"
#include "SurgSim/Physics/FixedConstraintFrictionlessContact.h"
#include "SurgSim/Physics/FixedRepresentation.h"
#include "SurgSim/Physics/MlcpPhysicsProblem.h"
#include "SurgSim/Physics/RigidConstraintFrictionlessContact.h"
#include "SurgSim/Physics/RigidRepresentation.h"

using SurgSim::DataStructures::Location;
using SurgSim::Math::SphereShape;
using SurgSim::Math::Vector3d;

namespace
{
	const double epsilon = 1e-10;
};

namespace SurgSim
{
namespace Physics
{

class ConstraintTests : public ::testing::Test
{
protected:
	/// Fixed plane pose
	SurgSim::Math::RigidTransform3d m_poseFixed;
	/// Rigid sphere pose
	SurgSim::Math::RigidTransform3d m_poseRigid;
	/// Contact normal direction
	Vector3d m_n;
	/// Distance to origin of the contact plane equation
	double m_d;
	/// Sphere radius
	double m_radius;
	/// Simulation time step
	double m_dt;
	/// Mlcp index of the sphere representation
	size_t m_indexSphereRepresentation;
	/// Mlcp index of the plane representation
	size_t m_indexPlaneRepresentation;
	/// Mlcp index of the constraint (frictionless contact)
	size_t m_indexConstraint;
	/// Contact location on the plane (point on the plane with the most penetration)
	Vector3d m_contactPositionPlane;
	/// Contact location on the sphere (point on the sphere with the most penetration)
	Vector3d m_contactPositionSphere;

	/// Rigid sphere
	std::shared_ptr<RigidRepresentation> m_rigid;
	/// Fixed plane
	std::shared_ptr<FixedRepresentation> m_fixed;

	/// Mlcp
	MlcpPhysicsProblem m_mlcpPhysicsProblem;
	/// Constraint
	std::shared_ptr<Constraint> m_constraint;
	/// Constraint data: frictionless contact
	std::shared_ptr<ContactConstraintData> m_constraintData;
	/// Location on the fixed plane
	Location m_locFixedPlane;
	/// Location on the rigid sphere
	Location m_locRigidSphere;

	/// Total number of degrees of freedom in the system (plane + sphere)
	size_t m_numDof;
	/// Total number of atomic constraint in the system (1 for a frictionless contact)
	size_t m_numConstraint;

	/// Setup the test case by creating all object
	void SetUp()
	{
		m_d = 0.0;
		m_radius = 0.01;
		m_dt = 1e-3;
		m_numDof = 0;
		m_indexSphereRepresentation = 0;
		m_indexConstraint = 0;
		m_numConstraint = 1;
		m_contactPositionPlane.setZero();
		m_contactPositionSphere.setZero();
		m_contactPositionSphere[1] = -m_radius;

		m_poseFixed.setIdentity();
		m_poseRigid.setIdentity();

		m_rigid = std::make_shared<RigidRepresentation>("Rigid");
		m_rigid->setLocalActive(true);
		m_rigid->setIsGravityEnabled(false);
		m_rigid->setLocalPose(m_poseRigid);
		{
			m_rigid->setDensity(1000.0);
			std::shared_ptr<SphereShape> shape = std::make_shared<SphereShape>(m_radius);
			m_rigid->setShape(shape);
		}
		m_numDof += m_rigid->getNumDof();

		m_indexPlaneRepresentation = m_indexSphereRepresentation + m_rigid->getNumDof();
		m_fixed = std::make_shared<FixedRepresentation>("Fixed");
		m_fixed->setLocalActive(true);
		m_fixed->setIsGravityEnabled(false);
		m_fixed->setLocalPose(m_poseFixed);
		m_numDof += m_fixed->getNumDof();

		m_locFixedPlane.rigidLocalPosition = m_contactPositionPlane;
		m_locRigidSphere.rigidLocalPosition = m_contactPositionSphere;

		m_constraintData = std::make_shared<ContactConstraintData>();

		clearMlcpPhysicsProblem(m_numDof, m_numConstraint);
	}

	/// Allocate and clear the Mlcp
	/// \param numDof The number of degrees of freedom in the system
	/// \param numConstraint The number of atomic constraints in the system
	void clearMlcpPhysicsProblem(size_t numDof, size_t numConstraint)
	{
		// Resize and zero all Eigen types
		m_mlcpPhysicsProblem.A.setZero(numConstraint, numConstraint);
		m_mlcpPhysicsProblem.b.setZero(numConstraint);
		m_mlcpPhysicsProblem.mu.setZero(numConstraint);
		m_mlcpPhysicsProblem.CHt.setZero(numDof, numConstraint);
		m_mlcpPhysicsProblem.H.resize(numConstraint, numDof);

		// Empty all std::vector types
		m_mlcpPhysicsProblem.constraintTypes.clear();
	}
};

TEST_F (ConstraintTests, TestConstructor)
{
	auto fixedRep = std::make_shared<FixedRepresentation>("fixed");
	auto rigidRep = std::make_shared<RigidRepresentation>("rigid");

	Location fixedLoc(m_contactPositionPlane);
	Location rigidLoc(m_contactPositionSphere);

	auto type = SurgSim::Math::MLCP_UNILATERAL_3D_FRICTIONLESS_CONSTRAINT;

	ASSERT_NO_THROW({Constraint c(type, m_constraintData, fixedRep, fixedLoc, rigidRep, rigidLoc);});

	EXPECT_THROW(
	{ Constraint c(type, nullptr, nullptr, fixedLoc, nullptr, fixedLoc); },
	SurgSim::Framework::AssertionFailure);
	EXPECT_THROW(
	{ Constraint c(type, m_constraintData, nullptr, fixedLoc, nullptr, fixedLoc); },
	SurgSim::Framework::AssertionFailure);
	EXPECT_THROW(
	{ Constraint c(type, m_constraintData, fixedRep, fixedLoc, nullptr, fixedLoc); },
	SurgSim::Framework::AssertionFailure);

	Constraint c(type, m_constraintData, fixedRep, fixedLoc, rigidRep, rigidLoc);

	EXPECT_EQ(m_constraintData, c.getData());
	EXPECT_EQ(type, c.getImplementations().first->getMlcpConstraintType());
	EXPECT_EQ(type, c.getImplementations().second->getMlcpConstraintType());
	EXPECT_EQ(fixedRep, c.getLocalizations().first->getRepresentation());
	EXPECT_EQ(rigidRep, c.getLocalizations().second->getRepresentation());
}

TEST_F (ConstraintTests, TestGetNumDof)
{
	auto fixedRep = std::make_shared<FixedRepresentation>("fixed");
	auto rigidRep = std::make_shared<RigidRepresentation>("rigid");

	Location fixedLoc(m_contactPositionPlane);
	Location rigidLoc(m_contactPositionSphere);

	auto type = SurgSim::Math::MLCP_UNILATERAL_3D_FRICTIONLESS_CONSTRAINT;

	{
		SCOPED_TRACE("1DOF for a frictionless contact");
		Constraint c(type, m_constraintData,
			m_fixed, m_locFixedPlane,
			m_rigid, m_locRigidSphere);
		EXPECT_EQ(1u, c.getNumDof());
	}

	{
		SCOPED_TRACE("1DOF for a frictionless contact between 2 fixed representations");
		Constraint c(type, m_constraintData, fixedRep, fixedLoc, fixedRep, fixedLoc);
		EXPECT_EQ(1u, c.getNumDof());
	}

	{
		SCOPED_TRACE("1DOF for a frictionless contact between 1 fixed representation and 1 rigid representation");
		Constraint c(type, m_constraintData, fixedRep, fixedLoc, rigidRep, rigidLoc);
		EXPECT_EQ(1u, c.getNumDof());
	}
}

// Test case: Rigid sphere at (0 0 0) with radius 0.01 colliding with Fixed plane Y=0
// Contact location on the rigid sphere is (0 -0.01 0)
// Contact location on the fixed plane is (0 0 0)
// Constraint: (Sphere - Plane).n >= 0 with n=(0 1 0) The normal should be the contact normal on the 2nd object
TEST_F (ConstraintTests, TestBuildMlcpSpherePlane)
{
	auto type = SurgSim::Math::MLCP_UNILATERAL_3D_FRICTIONLESS_CONSTRAINT;
	m_n.setZero();
	m_n[1] = 1.0;
	m_constraintData->setPlaneEquation(m_n, m_d);
	m_constraint = std::make_shared<Constraint>(type, m_constraintData,
												m_rigid, m_locRigidSphere,
												m_fixed, m_locFixedPlane);

	// Simulate 1 time step...to make sure all representation have a valid compliance matrix...
	{
		m_fixed->beforeUpdate(m_dt);
		m_rigid->beforeUpdate(m_dt);

		m_fixed->update(m_dt);
		m_rigid->update(m_dt);

		m_fixed->afterUpdate(m_dt);
		m_rigid->afterUpdate(m_dt);
	}

	// Fill up the Mlcp
	m_constraint->build(m_dt, &m_mlcpPhysicsProblem, m_indexSphereRepresentation, m_indexPlaneRepresentation,
		m_indexConstraint);

	// Violation b should be exactly -radius (the sphere center is on the plane)
	// This should not depend on the ordering of the object...the violation remains the same no matter what
	EXPECT_NEAR(-m_radius, m_mlcpPhysicsProblem.b[0], epsilon);

	// Constraint H should be
	// H = dt.[nx  ny  nz  nz.GPy-ny.GPz  nx.GPz-nz.GPx  ny.GPx-nx.GPy]
	// The rigid sphere being the 1st representation in the pair, it has the positive sign in the constraint !
	double sign = 1.0;
	Vector3d n_GP = m_n.cross(Vector3d(0.0, -m_radius, 0.0));

	SurgSim::Math::Matrix h = m_mlcpPhysicsProblem.H;
	EXPECT_NEAR(sign * m_dt * m_n[0] , h(0, 0), epsilon);
	EXPECT_NEAR(sign * m_dt * m_n[1] , h(0, 1), epsilon);
	EXPECT_NEAR(sign * m_dt * m_n[2] , h(0, 2), epsilon);
	EXPECT_NEAR(sign * m_dt * n_GP[0], h(0, 3), epsilon);
	EXPECT_NEAR(sign * m_dt * n_GP[1], h(0, 4), epsilon);
	EXPECT_NEAR(sign * m_dt * n_GP[2], h(0, 5), epsilon);

	// ConstraintTypes should contain 1 entry SurgSim::Math::MLCP_UNILATERAL_3D_FRICTIONLESS_CONSTRAINT
	ASSERT_EQ(1u, m_mlcpPhysicsProblem.constraintTypes.size());
	EXPECT_EQ(type, m_mlcpPhysicsProblem.constraintTypes[0]);
}

// Test case: Rigid sphere at (0 0 0) with radius 0.01 colliding with Fixed plane Y=0
// Contact location on the rigid sphere is (0 -0.01 0)
// Contact location on the fixed plane is (0 0 0)
// Constraint: (Plane - Sphere).n >= 0 with n=(0 -1 0) The normal should be the contact normal on the 2nd object
TEST_F (ConstraintTests, TestBuildMlcpPlaneSphere)
{
	auto type = SurgSim::Math::MLCP_UNILATERAL_3D_FRICTIONLESS_CONSTRAINT;
	m_n.setZero();
	m_n[1] = -1.0;
	m_constraintData->setPlaneEquation(m_n, m_d);
	m_constraint = std::make_shared<Constraint>(type, m_constraintData,
		m_fixed, m_locFixedPlane,
		m_rigid, m_locRigidSphere);

	// Simulate 1 time step...to make sure all representation have a valid compliance matrix...
	{
		m_fixed->beforeUpdate(m_dt);
		m_rigid->beforeUpdate(m_dt);

		m_fixed->update(m_dt);
		m_rigid->update(m_dt);

		m_fixed->afterUpdate(m_dt);
		m_rigid->afterUpdate(m_dt);
	}

	// Fill up the Mlcp
	m_constraint->build(m_dt, &m_mlcpPhysicsProblem, m_indexPlaneRepresentation, m_indexSphereRepresentation,
		m_indexConstraint);

	// Violation b should be exactly -radius (the sphere center is on the plane)
	// This should not depend on the ordering of the object...the violation remains the same no matter what
	EXPECT_NEAR(-m_radius, m_mlcpPhysicsProblem.b[0], epsilon);

	// Constraint H should be
	// H = dt.[nx  ny  nz  nz.GPy-ny.GPz  nx.GPz-nz.GPx  ny.GPx-nx.GPy]
	// The rigid sphere being the 2nd representation in the pair, it has the negative sign in the constraint !
	double sign = -1.0;
	Vector3d n_GP = m_n.cross(Vector3d(0.0, -m_radius, 0.0));

	SurgSim::Math::Matrix h = m_mlcpPhysicsProblem.H;
	EXPECT_NEAR(sign * m_dt * m_n[0] , h(0, 0), epsilon);
	EXPECT_NEAR(sign * m_dt * m_n[1] , h(0, 1), epsilon);
	EXPECT_NEAR(sign * m_dt * m_n[2] , h(0, 2), epsilon);
	EXPECT_NEAR(sign * m_dt * n_GP[0], h(0, 3), epsilon);
	EXPECT_NEAR(sign * m_dt * n_GP[1], h(0, 4), epsilon);
	EXPECT_NEAR(sign * m_dt * n_GP[2], h(0, 5), epsilon);

	// ConstraintTypes should contain 1 entry SurgSim::Math::MLCP_UNILATERAL_3D_FRICTIONLESS_CONSTRAINT
	ASSERT_EQ(1u, m_mlcpPhysicsProblem.constraintTypes.size());
	EXPECT_EQ(type, m_mlcpPhysicsProblem.constraintTypes[0]);
}

};  //  namespace Physics
};  //  namespace SurgSim
