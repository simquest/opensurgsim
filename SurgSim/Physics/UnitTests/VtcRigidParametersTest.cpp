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

#include <SurgSim/Physics/VtcRigidParameters.h>

namespace
{
double epsilon = 1e-10;
}

TEST(VtcRigidParametersTest, ConstructorTest)
{
	ASSERT_NO_THROW({SurgSim::Physics::VtcRigidParameters vtcRigidParam;});
}

TEST(VtcRigidParametersTest, DefaultValueTest)
{
	// Create the base rigid representation state
	SurgSim::Physics::VtcRigidParameters vtcRigidParam;

	// Linear damping [default = 0]
	EXPECT_NEAR(0.0, vtcRigidParam.getVtcLinearDamping(), epsilon);
	// Angular damping [default = 0]
	EXPECT_NEAR(0.0, vtcRigidParam.getVtcAngularDamping(), epsilon);
	// Linear stiffness [default = 0]
	EXPECT_NEAR(0.0, vtcRigidParam.getVtcLinearStiffness(), epsilon);
	// Angular stiffness [default = 0]
	EXPECT_NEAR(0.0, vtcRigidParam.getVtcAngularStiffness(), epsilon);
}

TEST(VtcRigidParametersTest, SetGetTest)
{
	// Create the base rigid representation state
	SurgSim::Physics::VtcRigidParameters vtcRigidParam;

	// Linear damping
	vtcRigidParam.setVtcLinearDamping(12.2);
	EXPECT_NEAR(12.2, vtcRigidParam.getVtcLinearDamping(), epsilon);
	vtcRigidParam.setVtcLinearDamping(0.0);
	EXPECT_NEAR(0.0, vtcRigidParam.getVtcLinearDamping(), epsilon);

	// Angular damping
	vtcRigidParam.setVtcAngularDamping(12.1);
	EXPECT_NEAR(12.1, vtcRigidParam.getVtcAngularDamping(), epsilon);
	vtcRigidParam.setVtcAngularDamping(0.0);
	EXPECT_NEAR(0.0, vtcRigidParam.getVtcAngularDamping(), epsilon);

	// Linear stiffness
	vtcRigidParam.setVtcLinearStiffness(12.0);
	EXPECT_NEAR(12.0, vtcRigidParam.getVtcLinearStiffness(), epsilon);
	vtcRigidParam.setVtcLinearStiffness(0.0);
	EXPECT_NEAR(0.0, vtcRigidParam.getVtcLinearStiffness(), epsilon);

	// Angular stiffness
	vtcRigidParam.setVtcAngularStiffness(11.9);
	EXPECT_NEAR(11.9, vtcRigidParam.getVtcAngularStiffness(), epsilon);
	vtcRigidParam.setVtcAngularStiffness(0.0);
	EXPECT_NEAR(0.0, vtcRigidParam.getVtcAngularStiffness(), epsilon);
}
