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
#include <memory>
#include <string>

#include "SurgSim/DataStructures/IndexedLocalCoordinate.h"
#include "SurgSim/Framework/Runtime.h"
#include "SurgSim/Math/MlcpConstraintType.h"
#include "SurgSim/Math/OdeState.h"
#include "SurgSim/Math/Vector.h"
#include "SurgSim/Physics/Fem1DElementBeam.h"
#include "SurgSim/Physics/Fem1DLocalization.h"
#include "SurgSim/Physics/Fem1DRepresentation.h"
#include "SurgSim/Physics/FemConstraintFixedRotationVector.h"
#include "SurgSim/Physics/FixedRepresentation.h"
#include "SurgSim/Physics/UnitTests/EigenGtestAsserts.h"
#include "SurgSim/Physics/Representation.h"
#include "SurgSim/Physics/RigidRepresentation.h"
#include "SurgSim/Physics/RotationVectorConstraintData.h"

using SurgSim::Math::Vector2d;
using SurgSim::Math::Vector3d;
using SurgSim::Math::Vector4d;
using SurgSim::Math::Vector6d;

namespace
{
const double epsilon = 1e-10;
const double dt = 1e-3;
};

namespace SurgSim
{
namespace Physics
{

static std::shared_ptr<Fem1DElementBeam> getBeam(size_t node0, size_t node1,
												 double radius,
												 double massDensity,
												 double poissonRatio,
												 double youngModulus)
{
	std::array<size_t, 2> nodeIds = {node0, node1};
	auto element = std::make_shared<Fem1DElementBeam>(nodeIds);
	element->setMassDensity(massDensity);
	element->setPoissonRatio(poissonRatio);
	element->setYoungModulus(youngModulus);
	element->setRadius(radius);
	return element;
}

static std::shared_ptr<Fem1DRepresentation> getFem1d(const std::string &name,
													 double radius = 0.01,
													 double massDensity = 1.0,
													 double poissonRatio = 0.1,
													 double youngModulus = 1.0)
{
	auto fem = std::make_shared<Fem1DRepresentation>(name);
	auto state = std::make_shared<SurgSim::Math::OdeState>();
	state->setNumDof(6, 4);

	std::array<double, 6> p0 = {{0.11, 0.22, 0.33, 0.44, 0.55, 0.66}};
	std::array<double, 6> p1 = {{0.12, -0.23, 0.34, -0.45, 0.56, -0.67}};
	std::array<double, 6> p2 = {{-0.10, 0.21, -0.32, 0.43, -0.54, 0.65}};
	std::array<double, 6> p3 = {{0.2, 0.2, 0.2, 0.2, 0.2, 0.2}};

	state->getPositions().segment<6>(0 * 6) = Vector6d(p0.data());
	state->getPositions().segment<6>(1 * 6) = Vector6d(p1.data());
	state->getPositions().segment<6>(2 * 6) = Vector6d(p2.data());
	state->getPositions().segment<6>(3 * 6) = Vector6d(p3.data());

	fem->addFemElement(getBeam(0, 1, radius, massDensity, poissonRatio, youngModulus));
	fem->addFemElement(getBeam(1, 2, radius, massDensity, poissonRatio, youngModulus));
	fem->addFemElement(getBeam(2, 3, radius, massDensity, poissonRatio, youngModulus));

	fem->setInitialState(state);
	fem->initialize(std::make_shared<SurgSim::Framework::Runtime>());
	fem->wakeUp();

	fem->setIsGravityEnabled(false);

	fem->beforeUpdate(dt);
	fem->update(dt);

	return fem;
}

TEST(Fem1DConstraintFixedRotationVectorTests, Constructor)
{
	ASSERT_NO_THROW({ FemConstraintFixedRotationVector constraint; });
}

TEST(Fem1DConstraintFixedRotationVectorTests, Constants)
{
	FemConstraintFixedRotationVector constraint;

	EXPECT_EQ(SurgSim::Physics::FIXED_3DROTATION_VECTOR, constraint.getConstraintType());
	EXPECT_EQ(3u, constraint.getNumDof());
}

TEST(Fem1DConstraintFixedRotationVectorTests, TestConnectionWithFixedRepresentation)
{
	auto fem1d = getFem1d("representation");
	auto fixed = std::make_shared<SurgSim::Physics::FixedRepresentation>("fixed");

	auto localization = std::make_shared<Fem1DLocalization>(fem1d,
		SurgSim::DataStructures::IndexedLocalCoordinate(2u, Vector2d(1.0, 0.0)));

	RotationVectorRigidFem1DConstraintData emptyConstraint;
	emptyConstraint.setFem1DRotation(fem1d, localization->getLocalPosition().index);
	emptyConstraint.setRigidOrFixedRotation(rigid, rigid->getPose().linear());

}

TEST(Fem1DConstraintFixedRotationVectorTests, TestConnectionWithRigidRepresentation)
{
}

TEST(Fem1DConstraintFixedRotationVectorTests, BuildMlcpBasic)
{
	// Whitebox test which validates ConstraintImplementation::build's output parameter, MlcpPhysicsProblem. It assumes
	// CHt and HCHt can be correctly built given H, so it does not necessarily construct the physical parameters
	// necessary to supply a realistic C.  It only checks H and b.
	FemConstraintFixedRotationVector constraint;

	auto fem1d = getFem1d("representation");
	auto rigid = std::make_shared<SurgSim::Physics::RigidRepresentation>("rigid");

	auto localization = std::make_shared<Fem1DLocalization>(fem1d,
		SurgSim::DataStructures::IndexedLocalCoordinate(2u, Vector2d(1.0, 0.0)));

	Vector3d actual = Vector3d::Zero();
	auto nodeIds = fem1d->getFemElement(2u)->getNodeIds();
	Vector3d rotationVector0 = fem1d->getInitialState()->getPositions().segment<3>(6 * nodeIds[0] + 3);
	Vector3d rotationVector1 = fem1d->getInitialState()->getPositions().segment<3>(6 * nodeIds[1] + 3);
	actual = SurgSim::Math::interpolate(rotationVector0, rotationVector1, 0.0);

	MlcpPhysicsProblem mlcpPhysicsProblem = MlcpPhysicsProblem::Zero(24, 3, 1);

	RotationVectorRigidFem1DConstraintData emptyConstraint;
	emptyConstraint.setFem1DRotation(fem1d, localization->getLocalPosition().index);
	emptyConstraint.setRigidOrFixedRotation(rigid, rigid->getPose().linear());

	ASSERT_NO_THROW(constraint.build(
		dt, emptyConstraint, localization, &mlcpPhysicsProblem, 0, 0, SurgSim::Physics::CONSTRAINT_POSITIVE_SIDE));

	// Compare results
	Eigen::Matrix<double, 3, 1> violation = actual;
	EXPECT_NEAR_EIGEN(violation, mlcpPhysicsProblem.b, epsilon);

	Eigen::Matrix<double, 3, 24> H = Eigen::Matrix<double, 3, 24>::Zero();
	Eigen::Matrix<double, 3, 3> identity = Eigen::Matrix<double, 3, 3>::Identity();
	SurgSim::Math::setSubMatrix(1.0 * dt * identity, 0, 5, 3, 3, &H); // This weight is on node 1 (beam 1, nodeId 0)
	EXPECT_NEAR_EIGEN(H, mlcpPhysicsProblem.H, epsilon);

	EXPECT_EQ(0u, mlcpPhysicsProblem.constraintTypes.size());
}

TEST(Fem1DConstraintFixedRotationVectorTests, BuildMlcp)
{
	// Whitebox test which validates ConstraintImplementation::build's output parameter, MlcpPhysicsProblem.  It assumes
	// CHt and HCHt can be correctly built given H, so it does not necessarily construct the physical parameters
	// necessary to supply a realistic C.  It only checks H and b.
	FemConstraintFixedRotationVector constraint;

	auto fem1d = getFem1d("representation");
	auto rigid = std::make_shared<SurgSim::Physics::RigidRepresentation>("rigid");

	// Setup parameters for FemConstraintFixedPoint::build
	auto localization = std::make_shared<Fem1DLocalization>(fem1d,
		SurgSim::DataStructures::IndexedLocalCoordinate(2u, Vector2d(0.3, 0.7)));

	Vector3d actual = Vector3d::Zero();
	auto nodeIds = fem1d->getFemElement(2u)->getNodeIds();
	Vector3d rotationVector0 = fem1d->getInitialState()->getPositions().segment<3>(6 * nodeIds[0] + 3);
	Vector3d rotationVector1 = fem1d->getInitialState()->getPositions().segment<3>(6 * nodeIds[1] + 3);
	actual = SurgSim::Math::interpolate(rotationVector0, rotationVector1, 0.7);

	MlcpPhysicsProblem mlcpPhysicsProblem = MlcpPhysicsProblem::Zero(24, 3, 1);

	RotationVectorRigidFem1DConstraintData emptyConstraint;
	emptyConstraint.setFem1DRotation(fem1d, localization->getLocalPosition().index);
	emptyConstraint.setRigidOrFixedRotation(rigid, rigid->getPose().linear());

	ASSERT_NO_THROW(constraint.build(
		dt, emptyConstraint, localization, &mlcpPhysicsProblem, 0, 0, SurgSim::Physics::CONSTRAINT_POSITIVE_SIDE));

	// Compare results
	Eigen::Matrix<double, 3, 1> violation = actual;
	EXPECT_NEAR_EIGEN(violation, mlcpPhysicsProblem.b, epsilon);

	Eigen::Matrix<double, 3, 24> H = Eigen::Matrix<double, 3, 24>::Zero();
	Eigen::Matrix<double, 3, 3> identity = Eigen::Matrix<double, 3, 3>::Identity();
	SurgSim::Math::setSubMatrix(0.3 * dt * identity, 0, 5, 3, 3, &H); // This weight is on node 2 (beam 2, nodeId 0)
	SurgSim::Math::setSubMatrix(0.7 * dt * identity, 0, 7, 3, 3, &H); // This weight is on node 3 (beam 2, nodeId 1)
	EXPECT_NEAR_EIGEN(H, mlcpPhysicsProblem.H, epsilon);

	EXPECT_EQ(0u, mlcpPhysicsProblem.constraintTypes.size());
}

TEST(Fem1DConstraintFixedRotationVectorTests, BuildMlcpTwoStep)
{
	// Whitebox test which validates ConstraintImplementation::build's output parameter, MlcpPhysicsProblem. It assumes
	// CHt and HCHt can be correctly built given H, so it does not necessarily construct the physical parameters
	// necessary to supply a realistic C.  It only checks H and b.
	FemConstraintFixedRotationVector constraint;

	auto fem1d = getFem1d("representation");
	auto rigid = std::make_shared<SurgSim::Physics::RigidRepresentation>("rigid");

	// Setup parameters for FemConstraintFixedPoint::build
	MlcpPhysicsProblem mlcpPhysicsProblem = MlcpPhysicsProblem::Zero(24, 3, 1);

	auto localization = std::make_shared<Fem1DLocalization>(fem1d,
		SurgSim::DataStructures::IndexedLocalCoordinate(2u, Vector2d(0.11, 0.89)));

	Vector3d actual = Vector3d::Zero();
	auto nodeIds = fem1d->getFemElement(2u)->getNodeIds();
	Vector3d rotationVector0 = fem1d->getInitialState()->getPositions().segment<3>(6 * nodeIds[0] + 3);
	Vector3d rotationVector1 = fem1d->getInitialState()->getPositions().segment<3>(6 * nodeIds[1] + 3);
	actual = SurgSim::Math::interpolate(rotationVector0, rotationVector1, 0.89);

	RotationVectorRigidFem1DConstraintData emptyConstraint;
	emptyConstraint.setFem1DRotation(fem1d, localization->getLocalPosition().index);
	emptyConstraint.setRigidOrFixedRotation(rigid, rigid->getPose().linear());

	ASSERT_NO_THROW(constraint.build(
		dt, emptyConstraint, localization, &mlcpPhysicsProblem, 0, 0, SurgSim::Physics::CONSTRAINT_POSITIVE_SIDE));

	localization->setLocalPosition(
		SurgSim::DataStructures::IndexedLocalCoordinate(1u, Vector2d(0.32, 0.68)));
	Vector3d desired = Vector3d::Zero();
	{
		auto nodeIds = fem1d->getFemElement(1u)->getNodeIds();
		Vector3d rotationVector0 = fem1d->getInitialState()->getPositions().segment<3>(6 * nodeIds[0] + 3);
		Vector3d rotationVector1 = fem1d->getInitialState()->getPositions().segment<3>(6 * nodeIds[1] + 3);
		desired = SurgSim::Math::interpolate(rotationVector0, rotationVector1, 0.68);
	}
	ASSERT_NO_THROW(constraint.build(
		dt, emptyConstraint, localization, &mlcpPhysicsProblem, 0, 0, SurgSim::Physics::CONSTRAINT_NEGATIVE_SIDE));

	// Compare results
	Eigen::Matrix<double, 3, 1> violation = actual - desired;
	EXPECT_NEAR_EIGEN(violation, mlcpPhysicsProblem.b, epsilon);

	Eigen::Matrix<double, 3, 24> H = Eigen::Matrix<double, 3, 24>::Zero();
	Eigen::Matrix<double, 3, 3> identity = Eigen::Matrix<double, 3, 3>::Identity();
	SurgSim::Math::addSubMatrix( 0.11 * dt * identity, 0, 5, 3, 3, &H); // This weight is on node 2 (beam 2, nodeId 0)
	SurgSim::Math::addSubMatrix( 0.89 * dt * identity, 0, 7, 3, 3, &H); // This weight is on node 3 (beam 2, nodeId 1)
	SurgSim::Math::addSubMatrix(-0.32 * dt * identity, 0, 3, 3, 3, &H); // This weight is on node 1 (beam 1, nodeId 0)
	SurgSim::Math::addSubMatrix(-0.68 * dt * identity, 0, 5, 3, 3, &H); // This weight is on node 2 (beam 1, nodeId 0)
	EXPECT_NEAR_EIGEN(H, mlcpPhysicsProblem.H, epsilon);
}

};  //  namespace Physics
};  //  namespace SurgSim
