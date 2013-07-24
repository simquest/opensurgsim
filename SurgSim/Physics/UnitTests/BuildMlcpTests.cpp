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

/// \file Simple Test for BuildMlcp calculation

#include <gtest/gtest.h>

#include <string>
#include <memory>

#include <SurgSim/Physics/UnitTests/CommonTests.h>
#include <SurgSim/Physics/BuildMlcp.h>


namespace SurgSim
{
namespace Physics
{

/// Redefine class
class BuildMlcpTests : public CommonTests
{
public:
	void SetUp()
	{
		CommonTests::SetUp();

		// Create the BuildMlcp computation
		m_buildMlcpComputation = std::make_shared<BuildMlcp>();
	}

protected:

	/// The Build Mlcp computation
	std::shared_ptr<BuildMlcp> m_buildMlcpComputation;
};

TEST_F(BuildMlcpTests, NoRepresentationNoConstraintTest)
{
	// Run the BuildMlcp computation...
	ASSERT_NO_THROW(m_buildMlcpComputation->update(dt, m_physicsManagerState));
	MlcpPhysicsProblem& mlcpProblem = m_physicsManagerState->getMlcpProblem();
	MlcpPhysicsSolution& mlcpSolution = m_physicsManagerState->getMlcpSolution();

	// Test that the Mlcp is as expected
	EXPECT_EQ(0, mlcpProblem.getSize());
	EXPECT_EQ(0, mlcpProblem.A.rows());
	EXPECT_EQ(0, mlcpProblem.A.cols());
	EXPECT_EQ(0, mlcpProblem.b.rows());
	EXPECT_EQ(0, mlcpProblem.CHt.rows());
	EXPECT_EQ(0, mlcpProblem.CHt.cols());
	EXPECT_EQ(0, mlcpProblem.H.rows());
	EXPECT_EQ(0, mlcpProblem.H.cols());
	EXPECT_EQ(0, mlcpProblem.mu.rows());
	EXPECT_EQ(0u, mlcpProblem.constraintTypes.size());
	EXPECT_TRUE(mlcpProblem.isConsistent());

	EXPECT_EQ(0, mlcpSolution.x.rows());
	EXPECT_EQ(0, mlcpSolution.dofCorrection.rows());
}

TEST_F(BuildMlcpTests, OneRepresentationNoConstraintTest)
{
	// Prep the list of representations: use only 1 representation
	m_usedRepresentations.push_back(m_allRepresentations[0]);
	// Set the representation list in the Physics Manager State
	m_physicsManagerState->setRepresentations(m_usedRepresentations);

	// Run the BuildMlcp computation...
	m_buildMlcpComputation->update(dt, m_physicsManagerState);
	MlcpPhysicsProblem& mlcpProblem = m_physicsManagerState->getMlcpProblem();
	MlcpPhysicsSolution& mlcpSolution = m_physicsManagerState->getMlcpSolution();

	// Test that the Mlcp is as expected
	EXPECT_EQ(0, mlcpProblem.getSize());
	EXPECT_EQ(0, mlcpProblem.A.rows());
	EXPECT_EQ(0, mlcpProblem.A.cols());
	EXPECT_EQ(0, mlcpProblem.b.rows());
	EXPECT_EQ(6, mlcpProblem.CHt.rows());
	EXPECT_EQ(0, mlcpProblem.CHt.cols());
	EXPECT_EQ(0, mlcpProblem.H.rows());
	EXPECT_EQ(6, mlcpProblem.H.cols());
	EXPECT_EQ(0, mlcpProblem.mu.rows());
	EXPECT_EQ(0u, mlcpProblem.constraintTypes.size());
	EXPECT_TRUE(mlcpProblem.isConsistent());

	EXPECT_EQ(0, mlcpSolution.x.rows());
	EXPECT_EQ(6, mlcpSolution.dofCorrection.rows());
}

TEST_F(BuildMlcpTests, TwoRepresentationsNoConstraintTest)
{
	// Prep the list of representations: use 2 representations
	m_usedRepresentations.push_back(m_allRepresentations[0]);
	m_usedRepresentations.push_back(m_allRepresentations[1]);
	// Set the representation list in the Physics Manager State
	m_physicsManagerState->setRepresentations(m_usedRepresentations);

	// Run the BuildMlcp computation...
	m_buildMlcpComputation->update(dt, m_physicsManagerState);
	MlcpPhysicsProblem& mlcpProblem = m_physicsManagerState->getMlcpProblem();
	MlcpPhysicsSolution& mlcpSolution = m_physicsManagerState->getMlcpSolution();

	// Test that the Mlcp is as expected
	EXPECT_EQ(0, mlcpProblem.getSize());
	EXPECT_EQ(0, mlcpProblem.A.rows());
	EXPECT_EQ(0, mlcpProblem.A.cols());
	EXPECT_EQ(0, mlcpProblem.b.rows());
	EXPECT_EQ(12, mlcpProblem.CHt.rows());
	EXPECT_EQ(0, mlcpProblem.CHt.cols());
	EXPECT_EQ(0, mlcpProblem.H.rows());
	EXPECT_EQ(12, mlcpProblem.H.cols());
	EXPECT_EQ(0, mlcpProblem.mu.rows());
	EXPECT_EQ(0u, mlcpProblem.constraintTypes.size());
	EXPECT_TRUE(mlcpProblem.isConsistent());

	EXPECT_EQ(0, mlcpSolution.x.rows());
	EXPECT_EQ(12, mlcpSolution.dofCorrection.rows());
}

TEST_F(BuildMlcpTests, OneRepresentationOneConstraintTest)
{
	// Prep the list of representations: use only 1 rigid representation + 1 fixed
	m_usedRepresentations.push_back(m_allRepresentations[0]);
	m_usedRepresentations.push_back(m_fixedWorldRepresentation);
	// Set the representation list in the Physics Manager State
	m_physicsManagerState->setRepresentations(m_usedRepresentations);

	// Prep the list of constraints: use only 1 constraint
	{
		std::shared_ptr<Localization> rigidLocalization;
		{
			std::shared_ptr<RigidRepresentationLocalization> rigidLocalizationTyped;
			rigidLocalizationTyped = std::make_shared<RigidRepresentationLocalization>();
			rigidLocalizationTyped->setRepresentation(m_usedRepresentations[0]);
			rigidLocalizationTyped->setLocalPosition(SurgSim::Math::Vector3d::Zero());
			rigidLocalization = rigidLocalizationTyped;
		}
		std::shared_ptr<RigidRepresentationContact> rigidSideContact;
		rigidSideContact = std::make_shared<RigidRepresentationContact>();

		std::shared_ptr<Localization> fixedLocalization;
		{
			std::shared_ptr<FixedRepresentationLocalization> fixedLocalizationTyped;
			fixedLocalizationTyped = std::make_shared<FixedRepresentationLocalization>();
			fixedLocalizationTyped->setRepresentation(m_fixedWorldRepresentation);
			fixedLocalizationTyped->setLocalPosition(SurgSim::Math::Vector3d::Zero());
			fixedLocalization = fixedLocalizationTyped;
		}
		std::shared_ptr<FixedRepresentationContact> fixedSideContact;
		fixedSideContact = std::make_shared<FixedRepresentationContact>();

		// Define the constraint specific data
		std::shared_ptr<ContactConstraintData> data = std::make_shared<ContactConstraintData>();
		data->setPlaneEquation(SurgSim::Math::Vector3d(0.0, 1.0, 0.0), 0.0);

		// Set up the constraint
		std::shared_ptr<Constraint> constraint = std::make_shared<Constraint>(
			data, rigidSideContact, rigidLocalization, fixedSideContact, fixedLocalization);

		// Register the constraint in the list of used constraints for this test
		m_usedConstraints.push_back(constraint);
	}

	// Set the constraint list in the Physics Manager State
	m_physicsManagerState->setConstraintGroup(CONSTRAINT_GROUP_TYPE_CONTACT, m_usedConstraints);

	// Run the BuildMlcp computation...
	m_buildMlcpComputation->update(dt, m_physicsManagerState);
	MlcpPhysicsProblem& mlcpProblem = m_physicsManagerState->getMlcpProblem();
	MlcpPhysicsSolution& mlcpSolution = m_physicsManagerState->getMlcpSolution();

	// Test that the Mlcp is as expected
	EXPECT_EQ(1, mlcpProblem.getSize());
	EXPECT_EQ(1, mlcpProblem.A.rows());
	EXPECT_EQ(1, mlcpProblem.A.cols());
	EXPECT_EQ(1, mlcpProblem.b.rows());
	EXPECT_EQ(6, mlcpProblem.CHt.rows());
	EXPECT_EQ(1, mlcpProblem.CHt.cols());
	EXPECT_EQ(1, mlcpProblem.H.rows());
	EXPECT_EQ(6, mlcpProblem.H.cols());
	EXPECT_EQ(1, mlcpProblem.mu.rows());
	EXPECT_EQ(1u, mlcpProblem.constraintTypes.size());
	EXPECT_EQ(SurgSim::Math::MLCP_UNILATERAL_3D_FRICTIONLESS_CONSTRAINT, mlcpProblem.constraintTypes[0]);
	EXPECT_TRUE(mlcpProblem.isConsistent());

	EXPECT_EQ(1, mlcpSolution.x.rows());
	EXPECT_EQ(6, mlcpSolution.dofCorrection.rows());
}

TEST_F(BuildMlcpTests, OneRepresentationTwoConstraintsTest)
{
	// Prep the list of representations: use only 1 rigid representation + 1 fixed
	m_usedRepresentations.push_back(m_allRepresentations[0]);
	m_usedRepresentations.push_back(m_fixedWorldRepresentation);
	// Set the representation list in the Physics Manager State
	m_physicsManagerState->setRepresentations(m_usedRepresentations);

	// Prep the list of constraints: use 2 constraints
	{
		std::shared_ptr<Localization> rigidLocalization;
		{
			std::shared_ptr<RigidRepresentationLocalization> rigidLocalizationTyped;
			rigidLocalizationTyped = std::make_shared<RigidRepresentationLocalization>();
			rigidLocalizationTyped->setRepresentation(m_usedRepresentations[0]);
			rigidLocalizationTyped->setLocalPosition(SurgSim::Math::Vector3d::Zero());
			rigidLocalization = rigidLocalizationTyped;
		}
		std::shared_ptr<RigidRepresentationContact> rigidSideContact;
		rigidSideContact = std::make_shared<RigidRepresentationContact>();

		std::shared_ptr<Localization> fixedLocalization;
		{
			std::shared_ptr<FixedRepresentationLocalization> fixedLocalizationTyped;
			fixedLocalizationTyped = std::make_shared<FixedRepresentationLocalization>();
			fixedLocalizationTyped->setRepresentation(m_fixedWorldRepresentation);
			fixedLocalizationTyped->setLocalPosition(SurgSim::Math::Vector3d::Zero());
			fixedLocalization = fixedLocalizationTyped;
		}
		std::shared_ptr<FixedRepresentationContact> fixedSideContact;
		fixedSideContact = std::make_shared<FixedRepresentationContact>();

		// Define the constraint specific data
		std::shared_ptr<ContactConstraintData> data = std::make_shared<ContactConstraintData>();
		data->setPlaneEquation(SurgSim::Math::Vector3d(0.0, 1.0, 0.0), 0.0);

		// Set up the constraint
		std::shared_ptr<Constraint> constraint = std::make_shared<Constraint>(
			data, rigidSideContact, rigidLocalization, fixedSideContact, fixedLocalization);

		// Register the constraint in the list of used constraints for this test
		m_usedConstraints.push_back(constraint);
	}
	{
		std::shared_ptr<Localization> rigidLocalization;
		{
			std::shared_ptr<RigidRepresentationLocalization> rigidLocalizationTyped;
			rigidLocalizationTyped = std::make_shared<RigidRepresentationLocalization>();
			rigidLocalizationTyped->setRepresentation(m_usedRepresentations[0]);
			rigidLocalizationTyped->setLocalPosition(SurgSim::Math::Vector3d::Ones());
			rigidLocalization = rigidLocalizationTyped;
		}
		std::shared_ptr<RigidRepresentationContact> rigidSideContact;
		rigidSideContact = std::make_shared<RigidRepresentationContact>();

		std::shared_ptr<Localization> fixedLocalization;
		{
			std::shared_ptr<FixedRepresentationLocalization> fixedLocalizationTyped;
			fixedLocalizationTyped = std::make_shared<FixedRepresentationLocalization>();
			fixedLocalizationTyped->setRepresentation(m_fixedWorldRepresentation);
			fixedLocalizationTyped->setLocalPosition(SurgSim::Math::Vector3d::Ones());
			fixedLocalization = fixedLocalizationTyped;
		}
		std::shared_ptr<FixedRepresentationContact> fixedSideContact;
		fixedSideContact = std::make_shared<FixedRepresentationContact>();

		// Define the constraint specific data
		std::shared_ptr<ContactConstraintData> data = std::make_shared<ContactConstraintData>();
		data->setPlaneEquation(SurgSim::Math::Vector3d(0.0, 1.0, 0.0), 0.0);

		// Set up the constraint
		std::shared_ptr<Constraint> constraint = std::make_shared<Constraint>(
			data, rigidSideContact, rigidLocalization, fixedSideContact, fixedLocalization);

		// Register the constraint in the list of used constraints for this test
		m_usedConstraints.push_back(constraint);
	}

	// Set the constraint list in the Physics Manager State
	m_physicsManagerState->setConstraintGroup(CONSTRAINT_GROUP_TYPE_CONTACT, m_usedConstraints);

	// Run the BuildMlcp computation...
	m_buildMlcpComputation->update(dt, m_physicsManagerState);
	MlcpPhysicsProblem& mlcpProblem = m_physicsManagerState->getMlcpProblem();
	MlcpPhysicsSolution& mlcpSolution = m_physicsManagerState->getMlcpSolution();

	// Test that the Mlcp is as expected
	EXPECT_EQ(2, mlcpProblem.getSize());
	EXPECT_EQ(2, mlcpProblem.A.rows());
	EXPECT_EQ(2, mlcpProblem.A.cols());
	EXPECT_EQ(2, mlcpProblem.b.rows());
	EXPECT_EQ(6, mlcpProblem.CHt.rows());
	EXPECT_EQ(2, mlcpProblem.CHt.cols());
	EXPECT_EQ(2, mlcpProblem.H.rows());
	EXPECT_EQ(6, mlcpProblem.H.cols());
	EXPECT_EQ(2, mlcpProblem.mu.rows());
	EXPECT_EQ(2u, mlcpProblem.constraintTypes.size());
	EXPECT_EQ(SurgSim::Math::MLCP_UNILATERAL_3D_FRICTIONLESS_CONSTRAINT, mlcpProblem.constraintTypes[0]);
	EXPECT_EQ(SurgSim::Math::MLCP_UNILATERAL_3D_FRICTIONLESS_CONSTRAINT, mlcpProblem.constraintTypes[1]);
	EXPECT_TRUE(mlcpProblem.isConsistent());

	EXPECT_EQ(2, mlcpSolution.x.rows());
	EXPECT_EQ(6, mlcpSolution.dofCorrection.rows());
}

TEST_F(BuildMlcpTests, TwoRepresentationsTwoConstraintsTest)
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

	// Prep the list of constraints: use 2 constraints
	{
		std::shared_ptr<Localization> rigid1Localization;
		{
			std::shared_ptr<RigidRepresentationLocalization> rigidLocalizationTyped;
			rigidLocalizationTyped = std::make_shared<RigidRepresentationLocalization>();
			rigidLocalizationTyped->setRepresentation(m_usedRepresentations[0]);
			rigidLocalizationTyped->setLocalPosition(pointOrigin);
			rigid1Localization = rigidLocalizationTyped;
		}
		std::shared_ptr<RigidRepresentationContact> rigidSide1Contact;
		rigidSide1Contact = std::make_shared<RigidRepresentationContact>();

		std::shared_ptr<Localization> rigid2Localization;
		{
			std::shared_ptr<RigidRepresentationLocalization> rigidLocalizationTyped;
			rigidLocalizationTyped = std::make_shared<RigidRepresentationLocalization>();
			rigidLocalizationTyped->setRepresentation(m_usedRepresentations[1]);
			rigidLocalizationTyped->setLocalPosition(pointOrigin);
			rigid2Localization = rigidLocalizationTyped;
		}
		std::shared_ptr<RigidRepresentationContact> rigidSide2Contact;
		rigidSide2Contact = std::make_shared<RigidRepresentationContact>();

		// Define the constraint specific data
		std::shared_ptr<ContactConstraintData> data = std::make_shared<ContactConstraintData>();
		data->setPlaneEquation(planeDirection, planeDistance);

		// Set up the constraint
		std::shared_ptr<Constraint> constraint = std::make_shared<Constraint>(
			data, rigidSide1Contact, rigid1Localization, rigidSide2Contact, rigid2Localization);

		// Register the constraint in the list of used constraints for this test
		m_usedConstraints.push_back(constraint);
	}
	{
		std::shared_ptr<Localization> rigid1Localization;
		{
			std::shared_ptr<RigidRepresentationLocalization> rigidLocalizationTyped;
			rigidLocalizationTyped = std::make_shared<RigidRepresentationLocalization>();
			rigidLocalizationTyped->setRepresentation(m_usedRepresentations[0]);
			rigidLocalizationTyped->setLocalPosition(pointOrigin);
			rigid1Localization = rigidLocalizationTyped;
		}
		std::shared_ptr<RigidRepresentationContact> rigidSide1Contact;
		rigidSide1Contact = std::make_shared<RigidRepresentationContact>();

		std::shared_ptr<Localization> rigid2Localization;
		{
			std::shared_ptr<RigidRepresentationLocalization> rigidLocalizationTyped;
			rigidLocalizationTyped = std::make_shared<RigidRepresentationLocalization>();
			rigidLocalizationTyped->setRepresentation(m_usedRepresentations[1]);
			rigidLocalizationTyped->setLocalPosition(pointOne);
			rigid2Localization = rigidLocalizationTyped;
		}
		std::shared_ptr<RigidRepresentationContact> rigidSide2Contact;
		rigidSide2Contact = std::make_shared<RigidRepresentationContact>();

		// Define the constraint specific data
		std::shared_ptr<ContactConstraintData> data = std::make_shared<ContactConstraintData>();
		data->setPlaneEquation(planeDirection, planeDistance);

		// Set up the constraint
		std::shared_ptr<Constraint> constraint = std::make_shared<Constraint>(
			data, rigidSide1Contact, rigid1Localization, rigidSide2Contact, rigid2Localization);

		// Register the constraint in the list of used constraints for this test
		m_usedConstraints.push_back(constraint);
	}

	// Set the constraint list in the Physics Manager State
	m_physicsManagerState->setConstraintGroup(CONSTRAINT_GROUP_TYPE_CONTACT, m_usedConstraints);

	// Run the BuildMlcp computation...
	m_buildMlcpComputation->update(dt, m_physicsManagerState);
	MlcpPhysicsProblem& mlcpProblem = m_physicsManagerState->getMlcpProblem();
	MlcpPhysicsSolution& mlcpSolution = m_physicsManagerState->getMlcpSolution();

	// Test that the Mlcp is as expected
	EXPECT_EQ(2, mlcpProblem.getSize());
	EXPECT_EQ(2, mlcpProblem.A.rows());
	EXPECT_EQ(2, mlcpProblem.A.cols());
	EXPECT_EQ(2, mlcpProblem.b.rows());
	// One the 1st constraint, both location are the origin (0 0 0), so there is no resulting violation
	EXPECT_NEAR(0.0, mlcpProblem.b[0], epsilon);
	// One the 2nd constraint, the constraints points are (0 0 0) and (0 1 0), so the resulting
	// violation along the normal direction (0 1 0) should be -1.0
	EXPECT_NEAR(planeDirection.dot(pointOrigin - pointOne), mlcpProblem.b[1], epsilon);
	EXPECT_EQ(12, mlcpProblem.CHt.rows());
	EXPECT_EQ(2, mlcpProblem.CHt.cols());
	EXPECT_EQ(2, mlcpProblem.H.rows());
	EXPECT_EQ(12, mlcpProblem.H.cols());
	EXPECT_EQ(2, mlcpProblem.mu.rows());
	EXPECT_EQ(2u, mlcpProblem.constraintTypes.size());
	EXPECT_EQ(SurgSim::Math::MLCP_UNILATERAL_3D_FRICTIONLESS_CONSTRAINT, mlcpProblem.constraintTypes[0]);
	EXPECT_EQ(SurgSim::Math::MLCP_UNILATERAL_3D_FRICTIONLESS_CONSTRAINT, mlcpProblem.constraintTypes[1]);
	EXPECT_TRUE(mlcpProblem.isConsistent());

	EXPECT_EQ(2, mlcpSolution.x.rows());
	EXPECT_EQ(12, mlcpSolution.dofCorrection.rows());
}

}; // namespace Physics
}; // namespace SurgSim