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
#include <string>

#include "SurgSim/DataStructures/IndexedLocalCoordinate.h"
#include "SurgSim/Framework/Runtime.h"
#include "SurgSim/Math/MlcpConstraintType.h"
#include "SurgSim/Math/OdeState.h"
#include "SurgSim/Math/Vector.h"
#include "SurgSim/Physics/ConstraintData.h"
#include "SurgSim/Physics/Representation.h"
#include "SurgSim/Physics/Fem1DConstraintFixedPoint.h"
#include "SurgSim/Physics/Fem1DElementBeam.h"
#include "SurgSim/Physics/Fem1DLocalization.h"
#include "SurgSim/Physics/Fem1DRepresentation.h"
#include "SurgSim/Physics/UnitTests/EigenGtestAsserts.h"

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

TEST(Fem1DConstraintFixedPointTests, Constructor)
{
	ASSERT_NO_THROW({ Fem1DConstraintFixedPoint constraint; });
}

TEST(Fem1DConstraintFixedPointTests, Constants)
{
	Fem1DConstraintFixedPoint constraint;

	EXPECT_EQ(SurgSim::Physics::FIXED_3DPOINT, constraint.getConstraintType());
	EXPECT_EQ(3u, constraint.getNumDof());
}

TEST(Fem1DConstraintFixedPointTests, BuildMlcpBasic)
{
	// Whitebox test which validates ConstraintImplementation::build's output parameter, MlcpPhysicsProblem. It assumes
	// CHt and HCHt can be correctly built given H, so it does not necessarily construct the physical parameters
	// necessary to supply a realistic C.  It only checks H and b.
	Fem1DConstraintFixedPoint constraint;

	Vector3d actual;

	// Setup parameters for Fem1DConstraintFixedPoint::build
	auto localization = std::make_shared<Fem1DLocalization>(getFem1d("representation"),
		SurgSim::DataStructures::IndexedLocalCoordinate(2u, Vector2d(1.0, 0.0)));

	actual = localization->calculatePosition();

	MlcpPhysicsProblem mlcpPhysicsProblem = MlcpPhysicsProblem::Zero(24, 3, 1);

	ConstraintData emptyConstraint;

	ASSERT_NO_THROW(constraint.build(
		dt, emptyConstraint, localization, &mlcpPhysicsProblem, 0, 0, SurgSim::Physics::CONSTRAINT_POSITIVE_SIDE));

	// Compare results
	Eigen::Matrix<double, 3, 1> violation = actual;
	EXPECT_NEAR_EIGEN(violation, mlcpPhysicsProblem.b, epsilon);

	Eigen::Matrix<double, 3, 24> H = Eigen::Matrix<double, 3, 24>::Zero();
	Eigen::Matrix<double, 3, 3> identity = Eigen::Matrix<double, 3, 3>::Identity();
	SurgSim::Math::setSubMatrix(1.0 * dt * identity, 0, 4, 3, 3, &H); // This weight is on node 2 (beam 2, nodeId 0)
	EXPECT_NEAR_EIGEN(H, mlcpPhysicsProblem.H, epsilon);

	EXPECT_EQ(0u, mlcpPhysicsProblem.constraintTypes.size());
}

TEST(Fem1DConstraintFixedPointTests, BuildMlcp)
{
	// Whitebox test which validates ConstraintImplementation::build's output parameter, MlcpPhysicsProblem.  It assumes
	// CHt and HCHt can be correctly built given H, so it does not necessarily construct the physical parameters
	// necessary to supply a realistic C.  It only checks H and b.
	Fem1DConstraintFixedPoint constraint;

	Vector3d actual;

	// Setup parameters for Fem1DConstraintFixedPoint::build
	auto localization = std::make_shared<Fem1DLocalization>(getFem1d("representation"),
		SurgSim::DataStructures::IndexedLocalCoordinate(2u, Vector2d(0.3, 0.7)));

	actual = localization->calculatePosition();

	MlcpPhysicsProblem mlcpPhysicsProblem = MlcpPhysicsProblem::Zero(24, 3, 1);

	ConstraintData emptyConstraint;

	ASSERT_NO_THROW(constraint.build(
		dt, emptyConstraint, localization, &mlcpPhysicsProblem, 0, 0, SurgSim::Physics::CONSTRAINT_POSITIVE_SIDE));

	// Compare results
	Eigen::Matrix<double, 3, 1> violation = actual;
	EXPECT_NEAR_EIGEN(violation, mlcpPhysicsProblem.b, epsilon);

	Eigen::Matrix<double, 3, 24> H = Eigen::Matrix<double, 3, 24>::Zero();
	Eigen::Matrix<double, 3, 3> identity = Eigen::Matrix<double, 3, 3>::Identity();
	SurgSim::Math::setSubMatrix(0.3 * dt * identity, 0, 4, 3, 3, &H); // This weight is on node 2 (beam 2, nodeId 0)
	SurgSim::Math::setSubMatrix(0.7 * dt * identity, 0, 6, 3, 3, &H); // This weight is on node 3 (beam 2, nodeId 1)
	EXPECT_NEAR_EIGEN(H, mlcpPhysicsProblem.H, epsilon);

	EXPECT_EQ(0u, mlcpPhysicsProblem.constraintTypes.size());
}

TEST(Fem1DConstraintFixedPointTests, BuildMlcpTwoStep)
{
	// Whitebox test which validates ConstraintImplementation::build's output parameter, MlcpPhysicsProblem. It assumes
	// CHt and HCHt can be correctly built given H, so it does not necessarily construct the physical parameters
	// necessary to supply a realistic C.  It only checks H and b.
	Fem1DConstraintFixedPoint constraint;

	Vector3d actual;
	Vector3d desired;

	// Setup parameters for Fem3DConstraintFixedPoint::build
	MlcpPhysicsProblem mlcpPhysicsProblem = MlcpPhysicsProblem::Zero(24, 3, 1);

	ConstraintData emptyConstraint;

	auto localization = std::make_shared<Fem1DLocalization>(getFem1d("representation"),
		SurgSim::DataStructures::IndexedLocalCoordinate(2u, Vector2d(0.11, 0.89)));
	actual = localization->calculatePosition();
	ASSERT_NO_THROW(constraint.build(
		dt, emptyConstraint, localization, &mlcpPhysicsProblem, 0, 0, SurgSim::Physics::CONSTRAINT_POSITIVE_SIDE));

	localization->setLocalPosition(
		SurgSim::DataStructures::IndexedLocalCoordinate(1u, Vector2d(0.32, 0.68)));
	desired = localization->calculatePosition();
	ASSERT_NO_THROW(constraint.build(
		dt, emptyConstraint, localization, &mlcpPhysicsProblem, 0, 0, SurgSim::Physics::CONSTRAINT_NEGATIVE_SIDE));

	// Compare results
	Eigen::Matrix<double, 3, 1> violation = actual - desired;
	EXPECT_NEAR_EIGEN(violation, mlcpPhysicsProblem.b, epsilon);

	Eigen::Matrix<double, 3, 24> H = Eigen::Matrix<double, 3, 24>::Zero();
	Eigen::Matrix<double, 3, 3> identity = Eigen::Matrix<double, 3, 3>::Identity();
	SurgSim::Math::addSubMatrix( 0.11 * dt * identity, 0, 4, 3, 3, &H); // This weight is on node 2 (beam 2, nodeId 0)
	SurgSim::Math::addSubMatrix( 0.89 * dt * identity, 0, 6, 3, 3, &H); // This weight is on node 3 (beam 2, nodeId 1)
	SurgSim::Math::addSubMatrix(-0.32 * dt * identity, 0, 2, 3, 3, &H); // This weight is on node 1 (beam 1, nodeId 0)
	SurgSim::Math::addSubMatrix(-0.68 * dt * identity, 0, 4, 3, 3, &H); // This weight is on node 2 (beam 1, nodeId 0)
	EXPECT_NEAR_EIGEN(H, mlcpPhysicsProblem.H, epsilon);
}

};  //  namespace Physics
};  //  namespace SurgSim
