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

#include <string>

#include "SurgSim/Framework/BasicSceneElement.h"
#include "SurgSim/Math/Vector.h"
#include "SurgSim/Math/Quaternion.h"
#include "SurgSim/Math/RigidTransform.h"
#include "SurgSim/Physics/FixedRepresentation.h"
#include "SurgSim/Physics/RigidRepresentationState.h"

using SurgSim::Framework::BasicSceneElement;
using SurgSim::Math::Vector3d;
using SurgSim::Math::Quaterniond;
using SurgSim::Math::RigidTransform3d;
using SurgSim::Physics::Representation;
using SurgSim::Physics::FixedRepresentation;
using SurgSim::Physics::RigidRepresentationState;

class FixedRepresentationTest : public ::testing::Test
{
public:
	void SetUp()
	{
		Quaterniond q;
		Vector3d t;

		q.coeffs().setRandom();
		q.normalize();
		t.setRandom();
		m_initialTransformation = SurgSim::Math::makeRigidTransform(q, t);

		do
		{
			q.coeffs().setRandom();
			q.normalize();
			t.setRandom();
			m_currentTransformation = SurgSim::Math::makeRigidTransform(q, t);
		} while (m_initialTransformation.isApprox(m_currentTransformation));

		m_identityTransformation.setIdentity();

		m_fixedRepresentation = std::make_shared<FixedRepresentation>("FixedRepresentation");
		m_element = std::make_shared<BasicSceneElement>("element");
		m_element->addComponent(m_fixedRepresentation);
	}

	void TearDown()
	{
	}

	std::shared_ptr<FixedRepresentation> m_fixedRepresentation;
	std::shared_ptr<BasicSceneElement> m_element;

	// Fixed representation initialization pose
	RigidTransform3d m_initialTransformation;

	// Fixed representation current pose
	RigidTransform3d m_currentTransformation;

	// Identity pose (no translation/rotation)
	RigidTransform3d m_identityTransformation;
};

TEST_F(FixedRepresentationTest, ConstructorTest)
{
	ASSERT_NO_THROW( {FixedRepresentation fixedRepresentation("FixedRepresentation");});
}

TEST_F(FixedRepresentationTest, ResetStateTest)
{
	m_fixedRepresentation->setIsActive(false);
	m_fixedRepresentation->setIsGravityEnabled(false);
	m_fixedRepresentation->setLocalPose(m_initialTransformation);

	m_element->initialize();
	m_fixedRepresentation->wakeUp();

	// Initial = Current = Previous = m_initialTransformation
	EXPECT_TRUE(m_fixedRepresentation->getInitialState().getPose().isApprox(m_initialTransformation));
	EXPECT_TRUE(m_fixedRepresentation->getCurrentState() == m_fixedRepresentation->getInitialState());
	EXPECT_TRUE(m_fixedRepresentation->getPreviousState() == m_fixedRepresentation->getInitialState());

	m_fixedRepresentation->setLocalPose(m_currentTransformation);
	m_fixedRepresentation->beforeUpdate(1.0);
	m_fixedRepresentation->update(1.0);
	m_fixedRepresentation->afterUpdate(1.0);
	// update is supposed to backup current in previous and set current
	// Therefore it should not affect initial
	// Initial = Previous = m_initialTransformation
	// Current = m_currentTransformation
	EXPECT_TRUE(m_fixedRepresentation->getCurrentState() != m_fixedRepresentation->getInitialState());
	EXPECT_TRUE(m_fixedRepresentation->getPreviousState() == m_fixedRepresentation->getInitialState());
	EXPECT_TRUE(m_fixedRepresentation->getPreviousState() != m_fixedRepresentation->getCurrentState());
	EXPECT_TRUE(m_fixedRepresentation->getCurrentState().getPose().isApprox(m_currentTransformation));

	m_fixedRepresentation->setLocalPose(m_currentTransformation);
	m_fixedRepresentation->beforeUpdate(1.0);
	m_fixedRepresentation->update(1.0);
	m_fixedRepresentation->afterUpdate(1.0);
	// update is supposed to backup current in previous and set current
	// Therefore it should not affect initial
	// Initial = m_initialTransformation
	// Previous = Current = m_currentTransformation
	EXPECT_TRUE(m_fixedRepresentation->getCurrentState() != m_fixedRepresentation->getInitialState());
	EXPECT_TRUE(m_fixedRepresentation->getPreviousState() != m_fixedRepresentation->getInitialState());
	EXPECT_TRUE(m_fixedRepresentation->getPreviousState() == m_fixedRepresentation->getCurrentState());
	EXPECT_TRUE(m_fixedRepresentation->getCurrentState().getPose().isApprox(m_currentTransformation));

	std::shared_ptr<Representation> representation = m_fixedRepresentation;
	// reset the representation (NOT THE FIXED REPRESENTATION, test polymorphism)
	representation->resetState();

	// isActive flag unchanged
	EXPECT_FALSE(m_fixedRepresentation->isActive());
	// isGravityEnable flag unchanged
	EXPECT_FALSE(m_fixedRepresentation->isGravityEnabled());
	// The current rigid state should be exactly the initial rigid state
	EXPECT_TRUE(m_fixedRepresentation->getCurrentState() == m_fixedRepresentation->getInitialState());
	EXPECT_TRUE(m_fixedRepresentation->getCurrentState().getPose().isApprox(m_initialTransformation));
	EXPECT_TRUE(m_fixedRepresentation->getInitialState().getPose().isApprox(m_initialTransformation));
	// The previous rigid state should be exactly the initial rigid state
	EXPECT_TRUE(m_fixedRepresentation->getPreviousState() == m_fixedRepresentation->getInitialState());
}

TEST_F(FixedRepresentationTest, SetGetAndDefaultValueTest)
{
	// Get/Set active flag [default = true]
	EXPECT_TRUE(m_fixedRepresentation->isActive());
	m_fixedRepresentation->setIsActive(false);
	ASSERT_FALSE(m_fixedRepresentation->isActive());
	m_fixedRepresentation->setIsActive(true);
	ASSERT_TRUE(m_fixedRepresentation->isActive());

	// Get numDof = 0
	ASSERT_EQ(0u, m_fixedRepresentation->getNumDof());

	// Set/Get isGravityEnabled [default = true]
	EXPECT_TRUE(m_fixedRepresentation->isGravityEnabled());
	m_fixedRepresentation->setIsGravityEnabled(false);
	ASSERT_FALSE(m_fixedRepresentation->isGravityEnabled());
	m_fixedRepresentation->setIsGravityEnabled(true);
	ASSERT_TRUE(m_fixedRepresentation->isGravityEnabled());
}

TEST_F(FixedRepresentationTest, UpdateTest)
{
	double dt = 1.0;

	m_fixedRepresentation->setLocalPose(m_initialTransformation);
	m_element->initialize();
	m_fixedRepresentation->wakeUp();

	m_fixedRepresentation->setLocalPose(m_currentTransformation);
	m_fixedRepresentation->beforeUpdate(dt);
	m_fixedRepresentation->update(dt);
	m_fixedRepresentation->afterUpdate(dt);

	EXPECT_TRUE(m_fixedRepresentation->getCurrentState() != m_fixedRepresentation->getPreviousState());
	EXPECT_TRUE(m_fixedRepresentation->getCurrentState() != m_fixedRepresentation->getInitialState());
	EXPECT_TRUE(m_fixedRepresentation->getPreviousState() == m_fixedRepresentation->getInitialState());
}

