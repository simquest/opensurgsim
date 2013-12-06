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

#include "SurgSim/Physics/Mass.h"
#include "SurgSim/Math/Vector.h"

using SurgSim::Physics::Mass;
using SurgSim::Math::Vector3d;

TEST(MassTests, Constructor)
{
	ASSERT_NO_THROW({Mass m(1.3);});

	ASSERT_NO_THROW({Mass* m = new Mass(1.3); delete m;});

	ASSERT_NO_THROW({std::shared_ptr<Mass> m = std::make_shared<Mass>(0.3);});
}

TEST(MassTests, SetGetMethodsTest)
{
	Mass m(1.3);

	EXPECT_DOUBLE_EQ(1.3, m.getMass());
	m.setMass(0.0);
	EXPECT_DOUBLE_EQ(0.0, m.getMass());
	m.setMass(-1.1);
	EXPECT_DOUBLE_EQ(-1.1, m.getMass());
	m.setMass(13.45);
	EXPECT_DOUBLE_EQ(13.45, m.getMass());
}


TEST(MassTests, ComparisonTests)
{
	Mass m1(1.3), m2, m3(1.3);

	EXPECT_FALSE(m1 == m2);
	EXPECT_TRUE(m1 == m3);
	EXPECT_FALSE(m2 == m3);
	m2 = m1;
	EXPECT_TRUE(m1 == m2);
	EXPECT_TRUE(m1 == m3);
	EXPECT_TRUE(m2 == m3);
	m3.setMass(m3.getMass() + 3.4);
	EXPECT_TRUE(m1 == m2);
	EXPECT_FALSE(m1 == m3);
	EXPECT_FALSE(m2 == m3);
}
