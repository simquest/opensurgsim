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

#include "SurgSim/Math/Vector.h"
#include "SurgSim/Math/Quaternion.h"
#include "SurgSim/Math/RigidTransform.h"
#include "SurgSim/Physics/FixedRepresentation.h"
#include "SurgSim/Physics/RigidRepresentationBaseLocalization.h"
#include "SurgSim/Physics/RigidRepresentation.h"
#include "SurgSim/Physics/UnitTests/MockObjects.h"

using SurgSim::Physics::MockFixedRepresentation;
using SurgSim::Physics::MockRigidRepresentation;
using SurgSim::Physics::RigidRepresentation;
using SurgSim::Physics::RigidRepresentationBaseLocalization;

namespace
{
	const double epsilon = 1e-10;
};

class RigidRepresentationBaseLocalizationTest : public ::testing::Test
{
public:
	void SetUp()
	{
		SurgSim::Math::Quaterniond q;
		SurgSim::Math::Vector3d t;

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
	}

	void TearDown()
	{
	}

	// Fixed representation initialization pose
	SurgSim::Math::RigidTransform3d m_initialTransformation;

	// Fixed representation current pose
	SurgSim::Math::RigidTransform3d m_currentTransformation;

	// Identity pose (no translation/rotation)
	SurgSim::Math::RigidTransform3d m_identityTransformation;
};

TEST_F(RigidRepresentationBaseLocalizationTest, ConstructorTest)
{
	ASSERT_NO_THROW( {RigidRepresentationBaseLocalization rigidRepresentationLoc;});

	ASSERT_NO_THROW(
	{
		std::shared_ptr<RigidRepresentation> rigid = std::make_shared<RigidRepresentation>("RigidRepresentation");
		RigidRepresentationBaseLocalization rigidRepresentationLoc(rigid);
	});
}

TEST_F(RigidRepresentationBaseLocalizationTest, SetGetRepresentation)
{
	RigidRepresentationBaseLocalization rigidRepresentationLoc;
	std::shared_ptr<RigidRepresentation> rigid = std::make_shared<RigidRepresentation>("RigidRepresentation");

	EXPECT_EQ(nullptr, rigidRepresentationLoc.getRepresentation());

	rigidRepresentationLoc.setRepresentation(rigid);
	EXPECT_EQ(rigid, rigidRepresentationLoc.getRepresentation());

	rigidRepresentationLoc.setRepresentation(nullptr);
	EXPECT_EQ(nullptr, rigidRepresentationLoc.getRepresentation());
}

TEST_F(RigidRepresentationBaseLocalizationTest, GetPositionTest)
{
	// Create the rigid body
	std::shared_ptr<MockRigidRepresentation> rigidRepresentation =
		std::make_shared<MockRigidRepresentation>();

	// Activate the rigid body and setup its initial pose
	rigidRepresentation->setLocalActive(true);
	rigidRepresentation->getCurrentState().setPose(m_initialTransformation);

	RigidRepresentationBaseLocalization localization = RigidRepresentationBaseLocalization(rigidRepresentation);
	ASSERT_EQ(rigidRepresentation, localization.getRepresentation());

	SurgSim::Math::Vector3d origin = m_initialTransformation.translation();
	SurgSim::Math::Vector3d zero = SurgSim::Math::Vector3d::Zero();
	localization.setLocalPosition(zero);
	EXPECT_TRUE(localization.getLocalPosition().isZero(epsilon));
	EXPECT_TRUE(localization.calculatePosition().isApprox(origin, epsilon));

	SurgSim::Math::Vector3d position = SurgSim::Math::Vector3d::Random();
	localization.setLocalPosition(position);
	EXPECT_TRUE(localization.getLocalPosition().isApprox(position, epsilon));
	EXPECT_FALSE(localization.calculatePosition().isApprox(origin, epsilon));
}


TEST_F(RigidRepresentationBaseLocalizationTest, FixedRepresentation)
{
	// Create the rigid body
	auto fixedRepresentation = std::make_shared<MockFixedRepresentation>();

	// Activate the rigid body and setup its initial pose
	fixedRepresentation->setLocalActive(true);
	fixedRepresentation->getCurrentState().setPose(m_initialTransformation);

	RigidRepresentationBaseLocalization localization = RigidRepresentationBaseLocalization(fixedRepresentation);
	ASSERT_EQ(fixedRepresentation, localization.getRepresentation());

	SurgSim::Math::Vector3d origin = m_initialTransformation.translation();
	SurgSim::Math::Vector3d zero = SurgSim::Math::Vector3d::Zero();
	localization.setLocalPosition(zero);
	EXPECT_TRUE(localization.getLocalPosition().isZero(epsilon));
	EXPECT_TRUE(localization.calculatePosition().isApprox(origin, epsilon));

	SurgSim::Math::Vector3d position = SurgSim::Math::Vector3d::Random();
	localization.setLocalPosition(position);
	EXPECT_TRUE(localization.getLocalPosition().isApprox(position, epsilon));
	EXPECT_FALSE(localization.calculatePosition().isApprox(origin, epsilon));
}
