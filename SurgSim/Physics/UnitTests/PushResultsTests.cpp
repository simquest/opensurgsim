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

/// \file PushResultsTests.cpp
/// Simple Test for PushResults calculation

#include <gtest/gtest.h>

#include <string>
#include <memory>

#include "SurgSim/Physics/UnitTests/CommonTests.h"
#include "SurgSim/Physics/BuildMlcp.h"
#include "SurgSim/Physics/PushResults.h"

namespace SurgSim
{
namespace Physics
{

class PushResultsTests : public CommonTests
{
public:
	void SetUp()
	{
		CommonTests::SetUp();

		// Create the BuildMlcp computation
		m_pushResultsComputation = std::make_shared<PushResults>();
	}

protected:

	/// The Push Results computation
	std::shared_ptr<PushResults> m_pushResultsComputation;
};

void updateRepresentationsMapping(std::shared_ptr<PhysicsManagerState> state)
{
	// The BuildMlcp computation build the representations mapping. So it is called.
	auto buildMlcpComputation = std::make_shared<BuildMlcp>();
	buildMlcpComputation->update(0.0, state);
}

TEST_F(PushResultsTests, NoRepresentationNoConstraint)
{
	// Run the BuildMlcp computation...
	ASSERT_NO_THROW(m_pushResultsComputation->update(dt, m_physicsManagerState));
}

TEST_F(PushResultsTests, OneRepresentationNoConstraintTest)
{
	// Prep the list of representations: use only 1 representation
	m_usedRepresentations.push_back(m_allRepresentations[0]);
	// Set the representation list in the Physics Manager State
	m_physicsManagerState->setRepresentations(m_usedRepresentations);

	// Run the BuildMlcp computation...
	ASSERT_NO_THROW(m_pushResultsComputation->update(dt, m_physicsManagerState));
}

TEST_F(PushResultsTests, TwoRepresentationsNoConstraintTest)
{
	// Prep the list of representations: use 2 representations
	m_usedRepresentations.push_back(m_allRepresentations[0]);
	m_usedRepresentations.push_back(m_allRepresentations[1]);
	// Set the representation list in the Physics Manager State
	m_physicsManagerState->setRepresentations(m_usedRepresentations);

	// Run the BuildMlcp computation...
	ASSERT_NO_THROW(m_pushResultsComputation->update(dt, m_physicsManagerState));
}

TEST_F(PushResultsTests, OneRepresentationOneConstraintTest)
{
	// Prep the list of representations: use only 1 rigid representation + 1 fixed
	m_usedRepresentations.push_back(m_allRepresentations[0]);
	m_usedRepresentations.push_back(m_fixedWorldRepresentation);
	// Set the representation list in the Physics Manager State
	m_physicsManagerState->setRepresentations(m_usedRepresentations);
	// The type of constraint.
	auto constraintType = SurgSim::Physics::FRICTIONLESS_3DCONTACT;

	// Prep the list of constraints: use only 1 constraint
	{
		// Define the constraint specific data
		std::shared_ptr<ContactConstraintData> data = std::make_shared<ContactConstraintData>();
		data->setPlaneEquation(SurgSim::Math::Vector3d(0.0, 1.0, 0.0), 0.0);

		// Set up the constraint
		std::shared_ptr<Constraint> constraint = std::make_shared<Constraint>(constraintType,
			data, m_usedRepresentations[0],
			SurgSim::DataStructures::Location(SurgSim::Math::Vector3d::Zero()),
			m_fixedWorldRepresentation,
			SurgSim::DataStructures::Location(SurgSim::Math::Vector3d::Zero()));

		// Register the constraint in the list of used constraints for this test
		m_usedConstraints.push_back(constraint);
	}

	// Set the constraint list in the Physics Manager State
	m_physicsManagerState->setConstraintGroup(CONSTRAINT_GROUP_TYPE_CONTACT, m_usedConstraints);

	// Update the Representations mapping.
	updateRepresentationsMapping(m_physicsManagerState);

	// Fill up the Mlcp problem and clear up the Mlcp solution
	resetMlcpProblem(6, 1);
	MlcpPhysicsProblem& mlcpProblem = m_physicsManagerState->getMlcpProblem();
	MlcpPhysicsSolution& mlcpSolution = m_physicsManagerState->getMlcpSolution();
	{
		mlcpProblem.CHt(0, 0) = 0.0;
		mlcpProblem.CHt(1, 0) = 1.0;
		mlcpProblem.CHt(2, 0) = 2.0;
		mlcpProblem.CHt(3, 0) = 3.0;
		mlcpProblem.CHt(4, 0) = 4.0;
		mlcpProblem.CHt(5, 0) = 5.0;

		mlcpSolution.x(0) = 1.3;
	}

	// Run the BuildMlcp computation...
	ASSERT_NO_THROW(m_pushResultsComputation->update(dt, m_physicsManagerState));

	// Test that the Mlcp is as expected
	EXPECT_EQ(1, mlcpSolution.x.rows());
	EXPECT_NEAR(1.3, mlcpSolution.x(0), epsilon);
	EXPECT_EQ(6, mlcpSolution.dofCorrection.rows());
	EXPECT_NEAR(1.3 * 0.0, mlcpSolution.dofCorrection(0), epsilon);
	EXPECT_NEAR(1.3 * 1.0, mlcpSolution.dofCorrection(1), epsilon);
	EXPECT_NEAR(1.3 * 2.0, mlcpSolution.dofCorrection(2), epsilon);
	EXPECT_NEAR(1.3 * 3.0, mlcpSolution.dofCorrection(3), epsilon);
	EXPECT_NEAR(1.3 * 4.0, mlcpSolution.dofCorrection(4), epsilon);
	EXPECT_NEAR(1.3 * 5.0, mlcpSolution.dofCorrection(5), epsilon);

	std::shared_ptr<RigidRepresentation> rigid;
	rigid = std::static_pointer_cast<RigidRepresentation>(m_usedRepresentations[0]);
	const SurgSim::Math::Vector3d& linVel = rigid->getCurrentState().getLinearVelocity();
	const SurgSim::Math::Vector3d& angVel = rigid->getCurrentState().getAngularVelocity();
	EXPECT_NEAR(1.3 * 0.0, linVel[0], epsilon);
	EXPECT_NEAR(1.3 * 1.0, linVel[1], epsilon);
	EXPECT_NEAR(1.3 * 2.0, linVel[2], epsilon);
	EXPECT_NEAR(1.3 * 3.0, angVel[0], epsilon);
	EXPECT_NEAR(1.3 * 4.0, angVel[1], epsilon);
	EXPECT_NEAR(1.3 * 5.0, angVel[2], epsilon);

	const SurgSim::Math::RigidTransform3d& pose = rigid->getCurrentState().getPose();
	EXPECT_NEAR(1.3 * 0.0 * dt, pose.translation()[0], epsilon);
	EXPECT_NEAR(1.3 * 1.0 * dt, pose.translation()[1], epsilon);
	EXPECT_NEAR(1.3 * 2.0 * dt, pose.translation()[2], epsilon);
}


TEST_F(PushResultsTests, DiscardBadResultsTest)
{
	// Prep the list of representations: use only 1 rigid representation + 1 fixed
	m_usedRepresentations.push_back(m_allRepresentations[0]);
	m_usedRepresentations.push_back(m_fixedWorldRepresentation);
	// Set the representation list in the Physics Manager State
	m_physicsManagerState->setRepresentations(m_usedRepresentations);
	// The type of constraint.
	auto constraintType = SurgSim::Physics::FRICTIONLESS_3DCONTACT;

	// Prep the list of constraints: use only 1 constraint
	{
		// Define the constraint specific data
		std::shared_ptr<ContactConstraintData> data = std::make_shared<ContactConstraintData>();
		data->setPlaneEquation(SurgSim::Math::Vector3d(0.0, 1.0, 0.0), 0.0);

		// Set up the constraint
		std::shared_ptr<Constraint> constraint = std::make_shared<Constraint>(constraintType,
			data, m_usedRepresentations[0],
			SurgSim::DataStructures::Location(SurgSim::Math::Vector3d::Zero()),
			m_fixedWorldRepresentation,
			SurgSim::DataStructures::Location(SurgSim::Math::Vector3d::Zero()));

		// Register the constraint in the list of used constraints for this test
		m_usedConstraints.push_back(constraint);
	}

	// Set the constraint list in the Physics Manager State
	m_physicsManagerState->setConstraintGroup(CONSTRAINT_GROUP_TYPE_CONTACT, m_usedConstraints);

	// Update the Representations mapping.
	updateRepresentationsMapping(m_physicsManagerState);

	// Fill up the Mlcp problem and clear up the Mlcp solution
	resetMlcpProblem(6, 1);
	MlcpPhysicsProblem& mlcpProblem = m_physicsManagerState->getMlcpProblem();
	MlcpPhysicsSolution& mlcpSolution = m_physicsManagerState->getMlcpSolution();
	double violation = 0.3;
	{
		mlcpProblem.CHt(0, 0) = 0.0;
		mlcpProblem.CHt(1, 0) = 1.0;
		mlcpProblem.CHt(2, 0) = 2.0;
		mlcpProblem.CHt(3, 0) = 3.0;
		mlcpProblem.CHt(4, 0) = 4.0;
		mlcpProblem.CHt(5, 0) = 5.0;
		mlcpProblem.A(0, 0) = 0.174;
		mlcpSolution.x(0) = 1.3;
		mlcpProblem.b(0) = violation - mlcpProblem.A(0, 0) * mlcpSolution.x(0);
		mlcpSolution.epsilonConvergence = 1e-4;
	}

	ASSERT_FALSE(m_pushResultsComputation->isDiscardBadResults());
	m_pushResultsComputation->setDiscardBadResults(true);
	ASSERT_TRUE(m_pushResultsComputation->isDiscardBadResults());
	{
		double contactTolerance = violation / 100.0;
		m_pushResultsComputation->setContactTolerance(contactTolerance);
		ASSERT_NEAR(contactTolerance, m_pushResultsComputation->getContactTolerance(), epsilon);
	}

	ASSERT_NO_THROW(m_pushResultsComputation->update(dt, m_physicsManagerState));

	// Test that the results were discarded because a violation is greater than the contact tolerance.
	EXPECT_NEAR(0.0, mlcpSolution.dofCorrection(0), epsilon);
	EXPECT_NEAR(0.0, mlcpSolution.dofCorrection(1), epsilon);
	EXPECT_NEAR(0.0, mlcpSolution.dofCorrection(2), epsilon);
	EXPECT_NEAR(0.0, mlcpSolution.dofCorrection(3), epsilon);
	EXPECT_NEAR(0.0, mlcpSolution.dofCorrection(4), epsilon);
	EXPECT_NEAR(0.0, mlcpSolution.dofCorrection(5), epsilon);

	std::shared_ptr<RigidRepresentation> rigid;
	rigid = std::static_pointer_cast<RigidRepresentation>(m_usedRepresentations[0]);
	const SurgSim::Math::Vector3d& linVel = rigid->getCurrentState().getLinearVelocity();
	const SurgSim::Math::Vector3d& angVel = rigid->getCurrentState().getAngularVelocity();
	EXPECT_NEAR(0.0, linVel[0], epsilon);
	EXPECT_NEAR(0.0, linVel[1], epsilon);
	EXPECT_NEAR(0.0, linVel[2], epsilon);
	EXPECT_NEAR(0.0, angVel[0], epsilon);
	EXPECT_NEAR(0.0, angVel[1], epsilon);
	EXPECT_NEAR(0.0, angVel[2], epsilon);

	const SurgSim::Math::RigidTransform3d& pose = rigid->getCurrentState().getPose();
	EXPECT_NEAR(0.0 * dt, pose.translation()[0], epsilon);
	EXPECT_NEAR(0.0 * dt, pose.translation()[1], epsilon);
	EXPECT_NEAR(0.0 * dt, pose.translation()[2], epsilon);

	// Now a violation less than the contact tolerance.
	m_pushResultsComputation->setContactTolerance(violation * 100.0);
	ASSERT_NO_THROW(m_pushResultsComputation->update(dt, m_physicsManagerState));

	// Test that the results were not discarded
	EXPECT_EQ(1, mlcpSolution.x.rows());
	EXPECT_NEAR(1.3, mlcpSolution.x(0), epsilon);
	EXPECT_EQ(6, mlcpSolution.dofCorrection.rows());
	EXPECT_NEAR(1.3 * 0.0, mlcpSolution.dofCorrection(0), epsilon);
	EXPECT_NEAR(1.3 * 1.0, mlcpSolution.dofCorrection(1), epsilon);
	EXPECT_NEAR(1.3 * 2.0, mlcpSolution.dofCorrection(2), epsilon);
	EXPECT_NEAR(1.3 * 3.0, mlcpSolution.dofCorrection(3), epsilon);
	EXPECT_NEAR(1.3 * 4.0, mlcpSolution.dofCorrection(4), epsilon);
	EXPECT_NEAR(1.3 * 5.0, mlcpSolution.dofCorrection(5), epsilon);

	EXPECT_NEAR(1.3 * 0.0, linVel[0], epsilon);
	EXPECT_NEAR(1.3 * 1.0, linVel[1], epsilon);
	EXPECT_NEAR(1.3 * 2.0, linVel[2], epsilon);
	EXPECT_NEAR(1.3 * 3.0, angVel[0], epsilon);
	EXPECT_NEAR(1.3 * 4.0, angVel[1], epsilon);
	EXPECT_NEAR(1.3 * 5.0, angVel[2], epsilon);

	EXPECT_NEAR(1.3 * 0.0 * dt, pose.translation()[0], epsilon);
	EXPECT_NEAR(1.3 * 1.0 * dt, pose.translation()[1], epsilon);
	EXPECT_NEAR(1.3 * 2.0 * dt, pose.translation()[2], epsilon);
}

TEST_F(PushResultsTests, OneRepresentationTwoConstraintsTest)
{
	// Prep the list of representations: use only 1 rigid representation + 1 fixed
	m_usedRepresentations.push_back(m_allRepresentations[0]);
	m_usedRepresentations.push_back(m_fixedWorldRepresentation);
	// Set the representation list in the Physics Manager State
	m_physicsManagerState->setRepresentations(m_usedRepresentations);
	// The type of constraint.
	auto constraintType = SurgSim::Physics::FRICTIONLESS_3DCONTACT;

	// Prep the list of constraints: use 2 constraints
	{
		// Define the constraint specific data
		std::shared_ptr<ContactConstraintData> data = std::make_shared<ContactConstraintData>();
		data->setPlaneEquation(SurgSim::Math::Vector3d(0.0, 1.0, 0.0), 0.0);

		// Set up the constraint
		std::shared_ptr<Constraint> constraint = std::make_shared<Constraint>(constraintType,
			data, m_usedRepresentations[0],
			SurgSim::DataStructures::Location(SurgSim::Math::Vector3d::Zero()),
			m_fixedWorldRepresentation,
			SurgSim::DataStructures::Location(SurgSim::Math::Vector3d::Zero()));

		// Register the constraint in the list of used constraints for this test
		m_usedConstraints.push_back(constraint);
	}
	{
		// Define the constraint specific data
		std::shared_ptr<ContactConstraintData> data = std::make_shared<ContactConstraintData>();
		data->setPlaneEquation(SurgSim::Math::Vector3d(0.0, 1.0, 0.0), 0.0);

		// Set up the constraint
		std::shared_ptr<Constraint> constraint = std::make_shared<Constraint>(constraintType,
			data, m_usedRepresentations[0],
			SurgSim::DataStructures::Location(SurgSim::Math::Vector3d::Ones()),
			m_fixedWorldRepresentation,
			SurgSim::DataStructures::Location(SurgSim::Math::Vector3d::Ones()));

		// Register the constraint in the list of used constraints for this test
		m_usedConstraints.push_back(constraint);
	}

	// Set the constraint list in the Physics Manager State
	m_physicsManagerState->setConstraintGroup(CONSTRAINT_GROUP_TYPE_CONTACT, m_usedConstraints);

	// Update the Representations mapping.
	updateRepresentationsMapping(m_physicsManagerState);

	// Fill up the Mlcp problem and clear up the Mlcp solution
	resetMlcpProblem(6, 2);
	MlcpPhysicsProblem& mlcpProblem = m_physicsManagerState->getMlcpProblem();
	MlcpPhysicsSolution& mlcpSolution = m_physicsManagerState->getMlcpSolution();
	{
		for (int dofId = 0; dofId < 6; dofId++)
		{
			mlcpProblem.CHt(dofId, 0) = static_cast<double>(dofId);
			mlcpProblem.CHt(dofId, 1) = static_cast<double>(dofId + 1);
		}

		mlcpSolution.x(0) = 1.3;
		mlcpSolution.x(1) =-0.9;
	}

	// Run the BuildMlcp computation...
	ASSERT_NO_THROW(m_pushResultsComputation->update(dt, m_physicsManagerState));

	// Test that the Mlcp is as expected
	EXPECT_EQ(2, mlcpSolution.x.rows());
	EXPECT_NEAR( 1.3, mlcpSolution.x(0), epsilon);
	EXPECT_NEAR(-0.9, mlcpSolution.x(1), epsilon);
	// dofCorrection = CHt .x = (0 1) . ( 1.3)
	//                          (1 2)   (-0.9)
	//                          (2 3)
	//                          (3 4)
	//                          (4 5)
	//                          (5 6)
	EXPECT_EQ(6, mlcpSolution.dofCorrection.rows());
	EXPECT_NEAR(1.3 * 0.0 - 0.9 * 1.0, mlcpSolution.dofCorrection(0), epsilon);
	EXPECT_NEAR(1.3 * 1.0 - 0.9 * 2.0, mlcpSolution.dofCorrection(1), epsilon);
	EXPECT_NEAR(1.3 * 2.0 - 0.9 * 3.0, mlcpSolution.dofCorrection(2), epsilon);
	EXPECT_NEAR(1.3 * 3.0 - 0.9 * 4.0, mlcpSolution.dofCorrection(3), epsilon);
	EXPECT_NEAR(1.3 * 4.0 - 0.9 * 5.0, mlcpSolution.dofCorrection(4), epsilon);
	EXPECT_NEAR(1.3 * 5.0 - 0.9 * 6.0, mlcpSolution.dofCorrection(5), epsilon);

	std::shared_ptr<RigidRepresentation> rigid;
	rigid = std::static_pointer_cast<RigidRepresentation>(m_usedRepresentations[0]);
	const SurgSim::Math::Vector3d& linVel = rigid->getCurrentState().getLinearVelocity();
	const SurgSim::Math::Vector3d& angVel = rigid->getCurrentState().getAngularVelocity();
	EXPECT_NEAR(1.3 * 0.0 - 0.9 * 1.0, linVel[0], epsilon);
	EXPECT_NEAR(1.3 * 1.0 - 0.9 * 2.0, linVel[1], epsilon);
	EXPECT_NEAR(1.3 * 2.0 - 0.9 * 3.0, linVel[2], epsilon);
	EXPECT_NEAR(1.3 * 3.0 - 0.9 * 4.0, angVel[0], epsilon);
	EXPECT_NEAR(1.3 * 4.0 - 0.9 * 5.0, angVel[1], epsilon);
	EXPECT_NEAR(1.3 * 5.0 - 0.9 * 6.0, angVel[2], epsilon);

	const SurgSim::Math::RigidTransform3d& pose = rigid->getCurrentState().getPose();
	EXPECT_NEAR((1.3 * 0.0 - 0.9 * 1.0) * dt, pose.translation()[0], epsilon);
	EXPECT_NEAR((1.3 * 1.0 - 0.9 * 2.0) * dt, pose.translation()[1], epsilon);
	EXPECT_NEAR((1.3 * 2.0 - 0.9 * 3.0) * dt, pose.translation()[2], epsilon);
}

TEST_F(PushResultsTests, TwoRepresentationsTwoConstraintsTest)
{
	SurgSim::Math::Vector3d pointOrigin = SurgSim::Math::Vector3d::Zero();
	SurgSim::Math::Vector3d planeDirection(0.0, 1.0, 0.0);
	double planeDistance = 0.0;
	SurgSim::Math::Vector3d pointOne = planeDirection * 1.0;

	// Prep the list of representations: use only 1 rigid representation + 1 fixed
	m_usedRepresentations.push_back(m_allRepresentations[0]);
	m_usedRepresentations.push_back(m_allRepresentations[1]);
	// Set the representation list in the Physics Manager State
	m_physicsManagerState->setRepresentations(m_usedRepresentations);
	// The type of constraint.
	auto constraintType = SurgSim::Physics::FRICTIONLESS_3DCONTACT;

	// Prep the list of constraints: use 2 constraints
	{
		// Define the constraint specific data
		std::shared_ptr<ContactConstraintData> data = std::make_shared<ContactConstraintData>();
		data->setPlaneEquation(planeDirection, planeDistance);

		// Set up the constraint
		std::shared_ptr<Constraint> constraint = std::make_shared<Constraint>(constraintType,
			data, m_usedRepresentations[0],
			SurgSim::DataStructures::Location(pointOrigin),
			m_usedRepresentations[1],
			SurgSim::DataStructures::Location(pointOrigin));

		// Register the constraint in the list of used constraints for this test
		m_usedConstraints.push_back(constraint);
	}
	{
		// Define the constraint specific data
		std::shared_ptr<ContactConstraintData> data = std::make_shared<ContactConstraintData>();
		data->setPlaneEquation(planeDirection, planeDistance);

		// Set up the constraint
		std::shared_ptr<Constraint> constraint = std::make_shared<Constraint>(constraintType,
			data, m_usedRepresentations[0],
			SurgSim::DataStructures::Location(pointOrigin),
			m_usedRepresentations[1],
			SurgSim::DataStructures::Location(pointOne));

		// Register the constraint in the list of used constraints for this test
		m_usedConstraints.push_back(constraint);
	}

	// Set the constraint list in the Physics Manager State
	m_physicsManagerState->setConstraintGroup(CONSTRAINT_GROUP_TYPE_CONTACT, m_usedConstraints);

	// Update the Representations mapping.
	updateRepresentationsMapping(m_physicsManagerState);

	// Fill up the Mlcp problem and clear up the Mlcp solution
	resetMlcpProblem(12, 2);
	MlcpPhysicsProblem& mlcpProblem = m_physicsManagerState->getMlcpProblem();
	MlcpPhysicsSolution& mlcpSolution = m_physicsManagerState->getMlcpSolution();
	{
		for (int dofId = 0; dofId < 12; dofId++)
		{
			mlcpProblem.CHt(dofId, 0) = static_cast<double>(dofId);
			mlcpProblem.CHt(dofId, 1) = static_cast<double>(dofId + 1);
		}

		mlcpSolution.x(0) = 1.3;
		mlcpSolution.x(1) =-0.9;
	}

	// Run the BuildMlcp computation...
	ASSERT_NO_THROW(m_pushResultsComputation->update(dt, m_physicsManagerState));

	// Test that the Mlcp is as expected
	EXPECT_EQ(2, mlcpSolution.x.rows());
	EXPECT_NEAR( 1.3, mlcpSolution.x(0), epsilon);
	EXPECT_NEAR(-0.9, mlcpSolution.x(1), epsilon);
	// dofCorrection = CHt .x = ( 0  1) . ( 1.3)
	//                          ( 1  2)   (-0.9)
	//                          ( 2  3)
	//                          ( 3  4)
	//                          ( 4  5)
	//                          ( 5  6)
	//                          ( 6  7)
	//                          ( 7  8)
	//                          ( 8  9)
	//                          ( 9 10)
	//                          (10 11)
	//                          (11 12)
	EXPECT_EQ(12, mlcpSolution.dofCorrection.rows());
	EXPECT_NEAR(1.3 * 0.0 - 0.9 * 1.0, mlcpSolution.dofCorrection(0), epsilon);
	EXPECT_NEAR(1.3 * 1.0 - 0.9 * 2.0, mlcpSolution.dofCorrection(1), epsilon);
	EXPECT_NEAR(1.3 * 2.0 - 0.9 * 3.0, mlcpSolution.dofCorrection(2), epsilon);
	EXPECT_NEAR(1.3 * 3.0 - 0.9 * 4.0, mlcpSolution.dofCorrection(3), epsilon);
	EXPECT_NEAR(1.3 * 4.0 - 0.9 * 5.0, mlcpSolution.dofCorrection(4), epsilon);
	EXPECT_NEAR(1.3 * 5.0 - 0.9 * 6.0, mlcpSolution.dofCorrection(5), epsilon);

	EXPECT_NEAR(1.3 * 6.0 - 0.9 * 7.0, mlcpSolution.dofCorrection(6), epsilon);
	EXPECT_NEAR(1.3 * 7.0 - 0.9 * 8.0, mlcpSolution.dofCorrection(7), epsilon);
	EXPECT_NEAR(1.3 * 8.0 - 0.9 * 9.0, mlcpSolution.dofCorrection(8), epsilon);
	EXPECT_NEAR(1.3 * 9.0 - 0.9 * 10.0, mlcpSolution.dofCorrection(9), epsilon);
	EXPECT_NEAR(1.3 * 10.0 - 0.9 * 11.0, mlcpSolution.dofCorrection(10), epsilon);
	EXPECT_NEAR(1.3 * 11.0 - 0.9 * 12.0, mlcpSolution.dofCorrection(11), epsilon);

	{
		std::shared_ptr<RigidRepresentation> rigid;
		rigid = std::static_pointer_cast<RigidRepresentation>(m_usedRepresentations[0]);
		const SurgSim::Math::Vector3d& linVel = rigid->getCurrentState().getLinearVelocity();
		const SurgSim::Math::Vector3d& angVel = rigid->getCurrentState().getAngularVelocity();
		EXPECT_NEAR(1.3 * 0.0 - 0.9 * 1.0, linVel[0], epsilon);
		EXPECT_NEAR(1.3 * 1.0 - 0.9 * 2.0, linVel[1], epsilon);
		EXPECT_NEAR(1.3 * 2.0 - 0.9 * 3.0, linVel[2], epsilon);
		EXPECT_NEAR(1.3 * 3.0 - 0.9 * 4.0, angVel[0], epsilon);
		EXPECT_NEAR(1.3 * 4.0 - 0.9 * 5.0, angVel[1], epsilon);
		EXPECT_NEAR(1.3 * 5.0 - 0.9 * 6.0, angVel[2], epsilon);
		const SurgSim::Math::RigidTransform3d& pose = rigid->getCurrentState().getPose();
		EXPECT_NEAR((1.3 * 0.0 - 0.9 * 1.0) * dt, pose.translation()[0], epsilon);
		EXPECT_NEAR((1.3 * 1.0 - 0.9 * 2.0) * dt, pose.translation()[1], epsilon);
		EXPECT_NEAR((1.3 * 2.0 - 0.9 * 3.0) * dt, pose.translation()[2], epsilon);
	}

	{
		std::shared_ptr<RigidRepresentation> rigid;
		rigid = std::static_pointer_cast<RigidRepresentation>(m_usedRepresentations[1]);
		const SurgSim::Math::Vector3d& linVel = rigid->getCurrentState().getLinearVelocity();
		const SurgSim::Math::Vector3d& angVel = rigid->getCurrentState().getAngularVelocity();
		EXPECT_NEAR(1.3 * 6.0 - 0.9 * 7.0, linVel[0], epsilon);
		EXPECT_NEAR(1.3 * 7.0 - 0.9 * 8.0, linVel[1], epsilon);
		EXPECT_NEAR(1.3 * 8.0 - 0.9 * 9.0, linVel[2], epsilon);
		EXPECT_NEAR(1.3 * 9.0 - 0.9 * 10.0, angVel[0], epsilon);
		EXPECT_NEAR(1.3 * 10.0 - 0.9 * 11.0, angVel[1], epsilon);
		EXPECT_NEAR(1.3 * 11.0 - 0.9 * 12.0, angVel[2], epsilon);
		const SurgSim::Math::RigidTransform3d& pose = rigid->getCurrentState().getPose();
		EXPECT_NEAR((1.3 * 6.0 - 0.9 * 7.0) * dt, pose.translation()[0], epsilon);
		EXPECT_NEAR((1.3 * 7.0 - 0.9 * 8.0) * dt, pose.translation()[1], epsilon);
		EXPECT_NEAR((1.3 * 8.0 - 0.9 * 9.0) * dt, pose.translation()[2], epsilon);
	}
}

}; // namespace Physics
}; // namespace SurgSim
