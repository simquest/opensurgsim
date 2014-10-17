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
#include "SurgSim/Physics/Constraint.h"
#include "SurgSim/Physics/ConstraintData.h"
#include "SurgSim/Physics/ContactConstraintData.h"
#include "SurgSim/Physics/MlcpPhysicsProblem.h"
#include "SurgSim/Physics/RigidRepresentation.h"
#include "SurgSim/Physics/RigidRepresentationContact.h"

#include "SurgSim/Math/Quaternion.h"
#include "SurgSim/Math/RigidTransform.h"
#include "SurgSim/Math/SphereShape.h"
#include "SurgSim/Math/Vector.h"

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

TEST (RigidRepresentationContactTests, SetGet_BuildMlcp_Test)
{
	Vector3d n(0.0, 1.0, 0.0);
	double d = 0.0;
	double radius = 0.01;
	double violation = -radius;

	Vector3d contactPosition = -n * (d - violation);
	SurgSim::Math::RigidTransform3d poseRigid;
	poseRigid.setIdentity();

	std::shared_ptr<RigidRepresentation> rigid = std::make_shared<RigidRepresentation>("Rigid");
	rigid->setLocalActive(true);
	rigid->setIsGravityEnabled(false);
	rigid->setLocalPose(poseRigid);
	rigid->setDensity(1000.0);
	rigid->setShape(std::make_shared<SphereShape>(radius));

	auto loc = std::make_shared<RigidRepresentationLocalization>(rigid);
	loc->setLocalPosition(contactPosition);
	std::shared_ptr<RigidRepresentationContact> implementation = std::make_shared<RigidRepresentationContact>();

	EXPECT_EQ(SurgSim::Math::MLCP_UNILATERAL_3D_FRICTIONLESS_CONSTRAINT, implementation->getMlcpConstraintType());
	EXPECT_EQ(1u, implementation->getNumDof());

	ContactConstraintData constraintData;
	constraintData.setPlaneEquation(n, d);

	MlcpPhysicsProblem mlcpPhysicsProblem = MlcpPhysicsProblem::Zero(rigid->getNumDof(), 1, 1);

	// Fill up the Mlcp
	double dt = 1e-3;
	implementation->build(dt, constraintData, loc,
		&mlcpPhysicsProblem, 0, 0, SurgSim::Physics::CONSTRAINT_POSITIVE_SIDE);

	// Violation b should be exactly violation = -radius (the sphere center is on the plane)
	EXPECT_NEAR(violation, mlcpPhysicsProblem.b[0], epsilon);

	// Constraint H should be
	// H = dt.[nx  ny  nz  nz.GPy-ny.GPz  nx.GPz-nz.GPx  ny.GPx-nx.GPy]
	Vector3d n_GP = n.cross(Vector3d(0.0, 0.0, 0.0));
	EXPECT_NEAR(dt * n[0]   , mlcpPhysicsProblem.H(0, 0), epsilon);
	EXPECT_NEAR(dt * n[1]   , mlcpPhysicsProblem.H(0, 1), epsilon);
	EXPECT_NEAR(dt * n[2]   , mlcpPhysicsProblem.H(0, 2), epsilon);
	EXPECT_NEAR(dt * n_GP[0], mlcpPhysicsProblem.H(0, 3), epsilon);
	EXPECT_NEAR(dt * n_GP[1], mlcpPhysicsProblem.H(0, 4), epsilon);
	EXPECT_NEAR(dt * n_GP[2], mlcpPhysicsProblem.H(0, 5), epsilon);

	// ConstraintTypes should contain 0 entry as it is setup by the constraint and not the ConstraintImplementation
	// This way, the constraint can verify that both ConstraintImplementation are the same type
	ASSERT_EQ(0u, mlcpPhysicsProblem.constraintTypes.size());
}

};  //  namespace Physics
};  //  namespace SurgSim
