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

#include <memory>

#include <gtest/gtest.h>
#include "SurgSim/Physics/Constraint.h"
#include "SurgSim/Physics/ConstraintData.h"
#include "SurgSim/Physics/SlidingConstraintData.h"
using SurgSim::Physics::Constraint;
using SurgSim::Physics::ConstraintData;
using SurgSim::Physics::SlidingConstraintData;

#include "SurgSim/Math/Vector.h"
using SurgSim::Math::Vector3d;

namespace
{
	const double epsilon = 1e-10;
};

TEST(SlidingConstraintDataTests, TestSetGet)
{
	using SurgSim::DataStructures::Location;

	SlidingConstraintData slidingConstraintData;
	Vector3d n1(1.2, 4.5, 6.7), n2(3.1, 6.3, 8.9);
	double d1 = 5.566, d2 = 6.777;
	n1.normalize();
	n2.normalize();

	EXPECT_NEAR(0.0, slidingConstraintData.getD1(), epsilon);
	EXPECT_TRUE(slidingConstraintData.getNormal1().isZero());
	EXPECT_NEAR(0.0, slidingConstraintData.getD2(), epsilon);
	EXPECT_TRUE(slidingConstraintData.getNormal2().isZero());

	slidingConstraintData.setPlane1Equation(n1, d1);

	EXPECT_NEAR(d1, slidingConstraintData.getD1(), epsilon);
	EXPECT_TRUE(slidingConstraintData.getNormal1().isApprox(n1, epsilon));

	slidingConstraintData.setPlane2Equation(n2, d2);

	EXPECT_NEAR(d2, slidingConstraintData.getD2(), epsilon);
	EXPECT_TRUE(slidingConstraintData.getNormal2().isApprox(n2, epsilon));
}
