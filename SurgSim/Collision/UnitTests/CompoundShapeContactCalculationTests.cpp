// This file is a part of the OpenSurgSim project.
// Copyright 2013-2015, SimQuest Solutions Inc.
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

#include "SurgSim/Collision/CompoundShapeDcdContact.h"
#include "SurgSim/Collision/BoxSphereDcdContact.h"
#include "SurgSim/Collision/BoxPlaneDcdContact.h"
#include "SurgSim/Collision/ShapeCollisionRepresentation.h"
#include "SurgSim/Math/Shapes.h"
#include "SurgSim/Collision/ContactCalculation.h"

#include "SurgSim/Collision/UnitTests/ContactCalculationTestsCommon.h"

namespace SurgSim
{
namespace Collision
{

class CompoundShapeDcdContactTest : public ::testing::Test
{
	virtual void SetUp() override
	{
		ContactCalculation::registerContactCalculation(std::make_shared<BoxSphereDcdContact>());
		ContactCalculation::registerContactCalculation(std::make_shared<BoxPlaneDcdContact>());
		ContactCalculation::registerContactCalculation(std::make_shared<CompoundShapeDcdContact>(
					std::make_pair(Math::SHAPE_TYPE_COMPOUNDSHAPE, Math::SHAPE_TYPE_SPHERE)));
		ContactCalculation::registerContactCalculation(std::make_shared<CompoundShapeDcdContact>(
					std::make_pair(Math::SHAPE_TYPE_COMPOUNDSHAPE, Math::SHAPE_TYPE_PLANE)));

	}

	virtual void TearDown() override
	{
	}
};

// The basic principle here is that colliding a set of shapes with each other should give the same
// results as colliding a compound representation
TEST_F(CompoundShapeDcdContactTest, SingleCube)
{
	auto box = std::make_shared<Math::BoxShape>();
	auto sphere = std::make_shared<Math::SphereShape>();

	auto compoundShape = std::make_shared<Math::CompoundShape>();

	Math::RigidTransform3d identity = Math::RigidTransform3d::Identity();
	Math::RigidTransform3d transform = Math::makeRigidTranslation(Math::Vector3d(0.25, 0.25, 0.25));

	compoundShape->addShape(box);

	auto calc1 = ContactCalculation::getContactTable()[Math::SHAPE_TYPE_BOX][Math::SHAPE_TYPE_SPHERE];
	auto calc2 = ContactCalculation::getContactTable()[Math::SHAPE_TYPE_COMPOUNDSHAPE][Math::SHAPE_TYPE_SPHERE];


	auto expected = calc1->calculateContact(box, identity, sphere, transform);
	auto result = calc2->calculateContact(compoundShape, identity, sphere, transform);

	contactsInfoEqualityTest(expected, result);
}

TEST_F(CompoundShapeDcdContactTest, MultipleShapes)
{
	auto box = std::make_shared<Math::BoxShape>();
	auto plane = std::make_shared<Math::PlaneShape>();

	Math::RigidTransform3d identity = Math::RigidTransform3d::Identity();
	auto basePose = Math::makeRigidTranslation(Math::Vector3d(0.0, 0.01, 0.0));
	auto box1Pose = Math::makeRigidTransform(
						Math::makeRotationQuaternion(0.01, Vector3d(0.01, 0.01, 0.01)),
						Vector3d(0.1, 0.1, 0.1));

	auto box2Pose = Math::makeRigidTransform(
						Math::makeRotationQuaternion(-0.01, Vector3d(0.01, 0.01, 0.01)),
						Vector3d(0.0, 1.0, 0.0));

	auto compoundShape = std::make_shared<Math::CompoundShape>();

	compoundShape->addShape(box, box1Pose);
	compoundShape->addShape(box, box2Pose);

	auto calc1 = ContactCalculation::getContactTable()[Math::SHAPE_TYPE_BOX][Math::SHAPE_TYPE_PLANE];
	auto calc2 = ContactCalculation::getContactTable()[Math::SHAPE_TYPE_COMPOUNDSHAPE][Math::SHAPE_TYPE_PLANE];

	auto result = calc2->calculateContact(compoundShape, basePose, plane, identity);

	std::list<std::shared_ptr<Contact>> expected;

	expected.splice(expected.end(), calc1->calculateContact(box, basePose * box1Pose, plane, identity));
	expected.splice(expected.end(), calc1->calculateContact(box, basePose * box2Pose, plane, identity));

	contactsInfoEqualityTest(expected, result);
}

}
}