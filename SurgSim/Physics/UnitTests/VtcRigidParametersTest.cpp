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

TEST(RigidVtcParametersTest, ConstructorTest)
{
	ASSERT_NO_THROW({SurgSim::Physics::RigidVtcParameters rigidVtcParam;});
}

TEST(RigidVtcParametersTest, DefaultValueTest)
{
	// Create the base rigid actor state
	SurgSim::Physics::RigidVtcParameters rigidVtcParam;

	// Linear damping [default = 0]
	EXPECT_EQ(0.0, rigidVtcParam.getVtcLinearDamping());
	// Angular damping [default = 0]
	EXPECT_EQ(0.0, rigidVtcParam.getVtcAngularDamping());
	// Linear stiffness [default = 0]
	EXPECT_EQ(0.0, rigidVtcParam.getVtcLinearStiffness());
	// Angular stiffness [default = 0]
	EXPECT_EQ(0.0, rigidVtcParam.getVtcAngularStiffness());
}

TEST(RigidVtcParametersTest, SetGetTest)
{
	// Create the base rigid actor state
	SurgSim::Physics::RigidVtcParameters rigidVtcParam;

	// Linear damping
	rigidVtcParam.setVtcLinearDamping(12.2);
	EXPECT_EQ(12.2, rigidVtcParam.getVtcLinearDamping());
	rigidVtcParam.setVtcLinearDamping(0.0);
	EXPECT_EQ(0.0, rigidVtcParam.getVtcLinearDamping());

	// Angular damping
	rigidVtcParam.setVtcAngularDamping(12.1);
	EXPECT_EQ(12.1, rigidVtcParam.getVtcAngularDamping());
	rigidVtcParam.setVtcAngularDamping(0.0);
	EXPECT_EQ(0.0, rigidVtcParam.getVtcAngularDamping());

	// Linear stiffness
	rigidVtcParam.setVtcLinearStiffness(12.0);
	EXPECT_EQ(12.0, rigidVtcParam.getVtcLinearStiffness());
	rigidVtcParam.setVtcLinearStiffness(0.0);
	EXPECT_EQ(0.0, rigidVtcParam.getVtcLinearStiffness());
	
	// Angular stiffness
	rigidVtcParam.setVtcAngularStiffness(11.9);
	EXPECT_EQ(11.9, rigidVtcParam.getVtcAngularStiffness());
	rigidVtcParam.setVtcAngularStiffness(0.0);
	EXPECT_EQ(0.0, rigidVtcParam.getVtcAngularStiffness());
}
