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
#include "SurgSim/Physics/Fem3DConstraintFixedPoint.h"
#include "SurgSim/Physics/Fem3DElementTetrahedron.h"
#include "SurgSim/Physics/Fem3DLocalization.h"
#include "SurgSim/Physics/Fem3DRepresentation.h"
#include "SurgSim/Physics/UnitTests/EigenGtestAsserts.h"

using SurgSim::Math::Vector3d;
using SurgSim::Math::Vector4d;

namespace
{
const double epsilon = 1e-10;
const double dt = 1e-3;
};

namespace SurgSim
{
namespace Physics
{

static std::shared_ptr<Fem3DElementTetrahedron> makeTetrahedron(size_t node0,
																size_t node1,
																size_t node2,
																size_t node3,
																double massDensity,
																double poissonRatio,
																double youngModulus)
{
	std::array<size_t, 4> nodes = {node0, node1, node2, node3};
	auto element = std::make_shared<Fem3DElementTetrahedron>(nodes);
	element->setMassDensity(massDensity);
	element->setPoissonRatio(poissonRatio);
	element->setYoungModulus(youngModulus);
	return element;
}

static std::shared_ptr<Fem3DRepresentation> getTestingFem3d(const std::string &name,
															double massDensity = 1.0,
															double poissonRatio = 0.1,
															double youngModulus = 1.0)
{
	auto fem = std::make_shared<Fem3DRepresentation>(name);
	auto state = std::make_shared<SurgSim::Math::OdeState>();
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

	fem->addFemElement(makeTetrahedron(0, 1, 2, 3, massDensity, poissonRatio, youngModulus));
	fem->addFemElement(makeTetrahedron(0, 1, 3, 4, massDensity, poissonRatio, youngModulus));
	fem->addFemElement(makeTetrahedron(0, 1, 4, 5, massDensity, poissonRatio, youngModulus));

	fem->setInitialState(state);
	fem->initialize(std::make_shared<SurgSim::Framework::Runtime>());
	fem->wakeUp();

	fem->setIsGravityEnabled(false);

	fem->beforeUpdate(dt);
	fem->update(dt);

	return fem;
}

TEST(Fem3DConstraintFixedPointTests, Constructor)
{
	ASSERT_NO_THROW(
		{ Fem3DConstraintFixedPoint constraint; });
}

TEST(Fem3DConstraintFixedPointTests, Constants)
{
	Fem3DConstraintFixedPoint constraint;

	EXPECT_EQ(SurgSim::Physics::FIXED_3DPOINT, constraint.getConstraintType());
	EXPECT_EQ(3u, constraint.getNumDof());
}

TEST(Fem3DConstraintFixedPointTests, BuildMlcpBasic)
{
	// Whitebox test which validates ConstraintImplementation::build's output parameter, MlcpPhysicsProblem.  It assumes
	// CHt and HCHt can be correctly built given H, so it does not necessarily construct the physical parameters
	// necessary to supply a realistic C.  It only checks H and b.
	Fem3DConstraintFixedPoint constraint;

	Vector3d actual;

	// Setup parameters for Fem3DConstraintFixedPoint::build
	auto localization = std::make_shared<Fem3DLocalization>(
		getTestingFem3d("representation"),
		SurgSim::DataStructures::IndexedLocalCoordinate(2u, Vector4d(0.0, 0.0, 1.0, 0.0)));

	actual = localization->calculatePosition();

	MlcpPhysicsProblem mlcpPhysicsProblem = MlcpPhysicsProblem::Zero(18, 3, 1);

	ConstraintData emptyConstraint;

	ASSERT_NO_THROW(constraint.build(
		dt, emptyConstraint, localization, &mlcpPhysicsProblem, 0, 0, SurgSim::Physics::CONSTRAINT_POSITIVE_SIDE));

	// Compare results
	Eigen::Matrix<double, 3, 1> violation = actual;
	EXPECT_NEAR_EIGEN(violation, mlcpPhysicsProblem.b, epsilon);

	Eigen::Matrix<double, 3, 18> H = Eigen::Matrix<double, 3, 18>::Zero();
	Eigen::Matrix<double, 3, 3> identity = Eigen::Matrix<double, 3, 3>::Identity();
	SurgSim::Math::setSubMatrix(1.0 * dt * identity, 0, 4, 3, 3, &H);
	EXPECT_NEAR_EIGEN(H, mlcpPhysicsProblem.H, epsilon);

	EXPECT_EQ(0u, mlcpPhysicsProblem.constraintTypes.size());
}

TEST(Fem3DConstraintFixedPointTests, BuildMlcp)
{
	// Whitebox test which validates ConstraintImplementation::build's output parameter, MlcpPhysicsProblem.  It assumes
	// CHt and HCHt can be correctly built given H, so it does not necessarily construct the physical parameters
	// necessary to supply a realistic C.  It only checks H and b.
	Fem3DConstraintFixedPoint constraint;

	Vector3d actual;

	// Setup parameters for Fem3DConstraintFixedPoint::build
	auto localization = std::make_shared<Fem3DLocalization>(
		getTestingFem3d("representation"),
		SurgSim::DataStructures::IndexedLocalCoordinate(2u, Vector4d(0.11, 0.02, 0.33, 0.54)));

	actual = localization->calculatePosition();

	MlcpPhysicsProblem mlcpPhysicsProblem = MlcpPhysicsProblem::Zero(18, 3, 1);

	ConstraintData emptyConstraint;

	ASSERT_NO_THROW(constraint.build(
		dt, emptyConstraint, localization, &mlcpPhysicsProblem, 0, 0, SurgSim::Physics::CONSTRAINT_POSITIVE_SIDE));

	// Compare results
	Eigen::Matrix<double, 3, 1> violation = actual;
	EXPECT_NEAR_EIGEN(violation, mlcpPhysicsProblem.b, epsilon);

	Eigen::Matrix<double, 3, 18> H = Eigen::Matrix<double, 3, 18>::Zero();
	Eigen::Matrix<double, 3, 3> identity = Eigen::Matrix<double, 3, 3>::Identity();
	SurgSim::Math::setSubMatrix(0.11 * dt * identity, 0, 0, 3, 3, &H);
	SurgSim::Math::setSubMatrix(0.02 * dt * identity, 0, 1, 3, 3, &H);
	SurgSim::Math::setSubMatrix(0.33 * dt * identity, 0, 4, 3, 3, &H);
	SurgSim::Math::setSubMatrix(0.54 * dt * identity, 0, 5, 3, 3, &H);
	EXPECT_NEAR_EIGEN(H, mlcpPhysicsProblem.H, epsilon);

	EXPECT_EQ(0u, mlcpPhysicsProblem.constraintTypes.size());
}

TEST(Fem3DConstraintFixedPointTests, BuildMlcpTwoStep)
{
	// Whitebox test which validates ConstraintImplementation::build's output parameter, MlcpPhysicsProblem.  It assumes
	// CHt and HCHt can be correctly built given H, so it does not neccessarily construct the physical parameters
	// neccessary to supply a realistic C.  It only checks H and b.
	Fem3DConstraintFixedPoint constraint;

	Vector3d actual;
	Vector3d desired;

	// Setup parameters for Fem3DConstraintFixedPoint::build
	MlcpPhysicsProblem mlcpPhysicsProblem = MlcpPhysicsProblem::Zero(18, 3, 1);

	ConstraintData emptyConstraint;

	auto localization = std::make_shared<Fem3DLocalization>(
		getTestingFem3d("representation"),
		SurgSim::DataStructures::IndexedLocalCoordinate(2u, Vector4d(0.11, 0.02, 0.33, 0.54)));
	actual = localization->calculatePosition();
	ASSERT_NO_THROW(constraint.build(
		dt, emptyConstraint, localization, &mlcpPhysicsProblem, 0, 0, SurgSim::Physics::CONSTRAINT_POSITIVE_SIDE));

	localization->setLocalPosition(
		SurgSim::DataStructures::IndexedLocalCoordinate(1u, Vector4d(0.32, 0.05, 0.14, 0.49)));
	desired = localization->calculatePosition();
	ASSERT_NO_THROW(constraint.build(
		dt, emptyConstraint, localization, &mlcpPhysicsProblem, 0, 0, SurgSim::Physics::CONSTRAINT_NEGATIVE_SIDE));

	// Compare results
	Eigen::Matrix<double, 3, 1> violation = actual - desired;
	EXPECT_NEAR_EIGEN(violation, mlcpPhysicsProblem.b, epsilon);

	Eigen::Matrix<double, 3, 18> H = Eigen::Matrix<double, 3, 18>::Zero();
	Eigen::Matrix<double, 3, 3> identity = Eigen::Matrix<double, 3, 3>::Identity();
	SurgSim::Math::addSubMatrix( 0.11 * dt * identity, 0, 0, 3, 3, &H);
	SurgSim::Math::addSubMatrix( 0.02 * dt * identity, 0, 1, 3, 3, &H);
	SurgSim::Math::addSubMatrix( 0.33 * dt * identity, 0, 4, 3, 3, &H);
	SurgSim::Math::addSubMatrix( 0.54 * dt * identity, 0, 5, 3, 3, &H);
	SurgSim::Math::addSubMatrix(-0.32 * dt * identity, 0, 0, 3, 3, &H);
	SurgSim::Math::addSubMatrix(-0.05 * dt * identity, 0, 1, 3, 3, &H);
	SurgSim::Math::addSubMatrix(-0.14 * dt * identity, 0, 3, 3, 3, &H);
	SurgSim::Math::addSubMatrix(-0.49 * dt * identity, 0, 4, 3, 3, &H);
	EXPECT_NEAR_EIGEN(H, mlcpPhysicsProblem.H, epsilon);
}

};  //  namespace Physics
};  //  namespace SurgSim
