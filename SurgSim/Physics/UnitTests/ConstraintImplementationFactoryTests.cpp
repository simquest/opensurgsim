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

#include "SurgSim/Math/MlcpConstraintType.h"
#include "SurgSim/Physics/Fem3DRepresentation.h"
#include "SurgSim/Physics/FixedRepresentation.h"
#include "SurgSim/Physics/FixedRepresentationContact.h"
#include "SurgSim/Physics/RigidRepresentation.h"

namespace SurgSim
{
namespace Physics
{

TEST(ConstraintImplementationFactoryTest, GetImplementationTest)
{
	using SurgSim::Math::MLCP_UNILATERAL_3D_FRICTIONLESS_CONSTRAINT;
	using SurgSim::Math::MLCP_BILATERAL_3D_CONSTRAINT;

	ConstraintImplementationFactory factory;
	EXPECT_TRUE(factory.getImplementation(typeid(FixedRepresentation), MLCP_UNILATERAL_3D_FRICTIONLESS_CONSTRAINT)
		!= nullptr);
	EXPECT_TRUE(factory.getImplementation(typeid(RigidRepresentation), MLCP_UNILATERAL_3D_FRICTIONLESS_CONSTRAINT)
		!= nullptr);
	EXPECT_TRUE(factory.getImplementation(typeid(Fem3DRepresentation), MLCP_UNILATERAL_3D_FRICTIONLESS_CONSTRAINT)
		!= nullptr);
	EXPECT_TRUE(factory.getImplementation(typeid(FixedRepresentation), MLCP_BILATERAL_3D_CONSTRAINT)
		!= nullptr);
	EXPECT_TRUE(factory.getImplementation(typeid(RigidRepresentation), MLCP_BILATERAL_3D_CONSTRAINT)
		!= nullptr);
	EXPECT_TRUE(factory.getImplementation(typeid(Fem3DRepresentation), MLCP_BILATERAL_3D_CONSTRAINT)
		!= nullptr);
}

TEST(ConstraintImplementationFactoryTest, AddImplementationTest)
{
	using SurgSim::Math::MLCP_UNILATERAL_3D_FRICTIONLESS_CONSTRAINT;

	class TestRepresentation {};
	ConstraintImplementationFactory factory;

	EXPECT_TRUE(factory.getImplementation(typeid(TestRepresentation), MLCP_UNILATERAL_3D_FRICTIONLESS_CONSTRAINT)
		== nullptr);

	// Add implementation for TestRepresentation.
	EXPECT_NO_THROW(
		factory.addImplementation(typeid(TestRepresentation), std::make_shared<FixedRepresentationContact>()));

	EXPECT_TRUE(factory.getImplementation(typeid(TestRepresentation), MLCP_UNILATERAL_3D_FRICTIONLESS_CONSTRAINT)
		!= nullptr);

	// Adding it again to make sure everything works, even if duplicate implementations are added.
	EXPECT_NO_THROW(
		factory.addImplementation(typeid(TestRepresentation), std::make_shared<FixedRepresentationContact>()));

	EXPECT_TRUE(factory.getImplementation(typeid(TestRepresentation), MLCP_UNILATERAL_3D_FRICTIONLESS_CONSTRAINT)
		!= nullptr);
}

} // namespace Physics
} // namespace SurgSim
