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

	Vector3d point(1.2, 3.4, 5.6), direction(7.8, 9.8, 7.6);
	direction.normalize();

	SlidingConstraintData slidingConstraintData;
	slidingConstraintData.setSlidingDirection(point, direction);

	const auto normals = slidingConstraintData.getNormals();
	const auto distances = slidingConstraintData.getDistances();

	EXPECT_NEAR(0.0, point.dot(normals[0]) + distances[0], epsilon);
	EXPECT_NEAR(0.0, point.dot(normals[1]) + distances[1], epsilon);

	EXPECT_NEAR(0.0, direction.dot(normals[0]), epsilon);
	EXPECT_NEAR(0.0, direction.dot(normals[1]), epsilon);
}
