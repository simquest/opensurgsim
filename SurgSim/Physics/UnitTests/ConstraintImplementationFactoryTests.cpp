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

#include "SurgSim/Physics/ConstraintImplementationFactory.h"

#include "SurgSim/Physics/Fem3DRepresentation.h"
#include "SurgSim/Physics/FixedConstraintFrictionlessContact.h"
#include "SurgSim/Physics/FixedRepresentation.h"
#include "SurgSim/Physics/RigidRepresentation.h"

namespace SurgSim
{
namespace Physics
{

TEST(ConstraintImplementationFactoryTest, GetImplementationTest)
{
	ConstraintImplementationFactory factory;
	EXPECT_TRUE(factory.getImplementation(typeid(FixedRepresentation), FRICTIONLESS_3DCONTACT)
		!= nullptr);
	EXPECT_TRUE(factory.getImplementation(typeid(RigidRepresentation), FRICTIONLESS_3DCONTACT)
		!= nullptr);
	EXPECT_TRUE(factory.getImplementation(typeid(Fem3DRepresentation), FRICTIONLESS_3DCONTACT)
		!= nullptr);
	EXPECT_TRUE(factory.getImplementation(typeid(FixedRepresentation), FIXED_3DPOINT)
		!= nullptr);
	EXPECT_TRUE(factory.getImplementation(typeid(RigidRepresentation), FIXED_3DPOINT)
		!= nullptr);
	EXPECT_TRUE(factory.getImplementation(typeid(Fem3DRepresentation), FIXED_3DPOINT)
		!= nullptr);
}

TEST(ConstraintImplementationFactoryTest, AddImplementationTest)
{

	class TestRepresentation {};
	ConstraintImplementationFactory factory;

	EXPECT_TRUE(factory.getImplementation(typeid(TestRepresentation), FRICTIONLESS_3DCONTACT)
		== nullptr);

	// Add implementation for TestRepresentation.
	EXPECT_NO_THROW(
		factory.addImplementation(typeid(TestRepresentation), std::make_shared<FixedConstraintFrictionlessContact>()));

	EXPECT_TRUE(factory.getImplementation(typeid(TestRepresentation), FRICTIONLESS_3DCONTACT)
		!= nullptr);

	// Adding it again to make sure everything works, even if duplicate implementations are added.
	EXPECT_NO_THROW(
		factory.addImplementation(typeid(TestRepresentation), std::make_shared<FixedConstraintFrictionlessContact>()));

	EXPECT_TRUE(factory.getImplementation(typeid(TestRepresentation), FRICTIONLESS_3DCONTACT)
		!= nullptr);
}

} // namespace Physics
} // namespace SurgSim
