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

#include <SurgSim/Physics/FixedRepresentation.h>
#include <SurgSim/Physics/FixedRepresentationLocalization.h>
using SurgSim::Physics::FixedRepresentation;
using SurgSim::Physics::FixedRepresentationLocalization;

#include <SurgSim/Math/Vector.h>
#include <SurgSim/Math/Quaternion.h>
#include <SurgSim/Math/RigidTransform.h>

namespace
{
	const double epsilon = 1e-10;
};

class FixedRepresentationLocalizationTest : public ::testing::Test
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

TEST_F(FixedRepresentationLocalizationTest, ConstructorTest)
{
	ASSERT_NO_THROW( {FixedRepresentationLocalization fixedRepresentationLoc;});

	ASSERT_NO_THROW(
	{
		std::shared_ptr<FixedRepresentation> fixed = std::make_shared<FixedRepresentation>("FixedRepresentation");
		FixedRepresentationLocalization fixedRepresentationLoc(fixed);
	});
}

TEST_F(FixedRepresentationLocalizationTest, SetGetRepresentation)
{
	FixedRepresentationLocalization fixedRepresentationLoc;
	std::shared_ptr<FixedRepresentation> fixed = std::make_shared<FixedRepresentation>("FixedRepresentation");
	
	EXPECT_EQ(nullptr, fixedRepresentationLoc.getRepresentation());

	fixedRepresentationLoc.setRepresentation(fixed);
	EXPECT_EQ(fixed, fixedRepresentationLoc.getRepresentation());

	fixedRepresentationLoc.setRepresentation(nullptr);
	EXPECT_EQ(nullptr, fixedRepresentationLoc.getRepresentation());
}

TEST_F(FixedRepresentationLocalizationTest, GetPositionTest)
{
	// Create the rigid body
	std::shared_ptr<FixedRepresentation> fixedRepresentation = std::make_shared<FixedRepresentation>("FixedRepresentation");

	// Activate the rigid body and setup its initial pose
	fixedRepresentation->setIsActive(true);
	fixedRepresentation->setInitialPose(m_initialTransformation);

	FixedRepresentationLocalization localization = FixedRepresentationLocalization(fixedRepresentation);
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

TEST_F(FixedRepresentationLocalizationTest, EqualityTest)
{
	std::shared_ptr<FixedRepresentation> fixedRepresentation1 = std::make_shared<FixedRepresentation>("FixedRepresentation1");
	std::shared_ptr<FixedRepresentation> fixedRepresentation2 = std::make_shared<FixedRepresentation>("FixedRepresentation2");
	FixedRepresentationLocalization localization1 = FixedRepresentationLocalization(fixedRepresentation1);
	FixedRepresentationLocalization localization2 = FixedRepresentationLocalization(fixedRepresentation1);
	SurgSim::Math::Vector3d zeroVector = SurgSim::Math::Vector3d::Zero();
	SurgSim::Math::Vector3d randomVector = SurgSim::Math::Vector3d::Random();
	localization1.setLocalPosition(zeroVector);
	localization2.setLocalPosition(zeroVector);
	EXPECT_EQ(localization1.getRepresentation(), fixedRepresentation1);
	EXPECT_EQ(localization2.getRepresentation(), fixedRepresentation1);
	EXPECT_TRUE(localization1 == localization2);
	EXPECT_FALSE(localization1 != localization2);

	localization2.setLocalPosition(randomVector);
	EXPECT_EQ(localization1.getRepresentation(), fixedRepresentation1);
	EXPECT_EQ(localization2.getRepresentation(), fixedRepresentation1);
	EXPECT_FALSE(localization1 == localization2);
	EXPECT_TRUE(localization1 != localization2);

	localization1.setLocalPosition(randomVector);
	EXPECT_EQ(localization1.getRepresentation(), fixedRepresentation1);
	EXPECT_EQ(localization2.getRepresentation(), fixedRepresentation1);
	EXPECT_TRUE(localization1 == localization2);
	EXPECT_FALSE(localization1 != localization2);

	localization2.setRepresentation(fixedRepresentation2);
	EXPECT_EQ(localization1.getRepresentation(), fixedRepresentation1);
	EXPECT_EQ(localization2.getRepresentation(), fixedRepresentation2);
	EXPECT_FALSE(localization1 == localization2);
	EXPECT_TRUE(localization1 != localization2);

	localization1.setRepresentation(fixedRepresentation2);
	EXPECT_EQ(localization1.getRepresentation(), fixedRepresentation2);
	EXPECT_EQ(localization2.getRepresentation(), fixedRepresentation2);
	EXPECT_TRUE(localization1 == localization2);
	EXPECT_FALSE(localization1 != localization2);
}
