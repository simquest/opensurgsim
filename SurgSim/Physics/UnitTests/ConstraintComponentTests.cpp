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

#include "SurgSim/Physics/Constraint.h"
#include "SurgSim/Physics/ConstraintComponent.h"
#include "SurgSim/Physics/UnitTests/MockObjects.h"

namespace SurgSim
{
namespace Physics
{

TEST(ConstraintComponentTest, Constructor)
{
	ASSERT_NO_THROW(
		{ ConstraintComponent("component"); });
}

TEST(ConstraintComponentTest, GetSetConstraint)
{
	auto component = std::make_shared<ConstraintComponent>("component");

	EXPECT_EQ(nullptr, component->getConstraint());

	auto constraint
		= makeMockConstraint(std::make_shared<MockRepresentation>(), std::make_shared<MockRepresentation>());

	ASSERT_NO_THROW(
		{ component->setConstraint(constraint); });
	EXPECT_EQ(constraint, component->getConstraint());

	ASSERT_NO_THROW(
		{ component->setConstraint(nullptr); });
	EXPECT_EQ(nullptr, component->getConstraint());
}

} // namespace Physics
} // namespace SurgSim
