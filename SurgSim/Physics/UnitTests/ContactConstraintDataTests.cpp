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
#include "SurgSim/Physics/ContactConstraintData.h"
using SurgSim::Physics::Constraint;
using SurgSim::Physics::ConstraintData;
using SurgSim::Physics::ContactConstraintData;

#include "SurgSim/Math/Vector.h"
using SurgSim::Math::Vector3d;

namespace
{
	const double epsilon = 1e-10;
};

TEST (ContactConstraintDataTests, TestSetGet)
{
	using SurgSim::Collision::Contact;
	using SurgSim::DataStructures::Location;

	ContactConstraintData contactConstraintData;
	Vector3d n(1.2, 4.5, 6.7);
	double d = 5.566;
	n.normalize();

	EXPECT_NEAR(0.0 , contactConstraintData.getDistance(), epsilon);
	EXPECT_TRUE(contactConstraintData.getNormal().isZero());

	contactConstraintData.setPlaneEquation(n, d);

	EXPECT_NEAR(d , contactConstraintData.getDistance(), epsilon);
	EXPECT_TRUE(contactConstraintData.getNormal().isApprox(n, epsilon));

	auto contact0 = std::make_shared<Contact>(SurgSim::Collision::COLLISION_DETECTION_TYPE_DISCRETE, 0.0, 0.0,
		Vector3d::Zero(), Vector3d::UnitX(), std::pair<Location, Location>());
	contactConstraintData.setContact(contact0);

	EXPECT_TRUE(contactConstraintData.getContact() == contact0);
}
