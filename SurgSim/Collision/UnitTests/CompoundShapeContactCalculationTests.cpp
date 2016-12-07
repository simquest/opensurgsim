// This file is a part of the OpenSurgSim project.
// Copyright 2013-2016, SimQuest Solutions Inc.
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

#include "SurgSim/Collision/BoxPlaneContact.h"
#include "SurgSim/Collision/BoxSphereContact.h"
#include "SurgSim/Collision/CompoundShapeContact.h"
#include "SurgSim/Collision/ContactCalculation.h"
#include "SurgSim/Collision/ShapeCollisionRepresentation.h"
#include "SurgSim/Collision/UnitTests/ContactCalculationTestsCommon.h"
#include "SurgSim/Math/Shapes.h"

namespace SurgSim
{

typedef Math::PosedShape<std::shared_ptr<Math::Shape>> PosedShape;

namespace Collision
{

class CompoundShapeDcdContactTest : public ::testing::Test
{
	void SetUp() override
	{
		ContactCalculation::registerDcdContactCalculation(std::make_shared<BoxSphereContact>());
		ContactCalculation::registerDcdContactCalculation(std::make_shared<BoxPlaneContact>());
		ContactCalculation::registerDcdContactCalculation(std::make_shared<CompoundShapeContact>(
					std::make_pair(Math::SHAPE_TYPE_COMPOUNDSHAPE, Math::SHAPE_TYPE_SPHERE)));
		ContactCalculation::registerDcdContactCalculation(std::make_shared<CompoundShapeContact>(
					std::make_pair(Math::SHAPE_TYPE_COMPOUNDSHAPE, Math::SHAPE_TYPE_PLANE)));

	}

	void TearDown() override
	{
	}
};

// The basic principle here is that colliding a set of shapes with each other should give the same
// results as colliding a compound representation
TEST_F(CompoundShapeDcdContactTest, SingleCube)
{
	auto box = std::make_shared<Math::BoxShape>(1.0, 1.0, 1.0);
	auto sphere = std::make_shared<Math::SphereShape>(1.0);

	auto compoundShape = std::make_shared<Math::CompoundShape>();

	Math::RigidTransform3d identity = Math::RigidTransform3d::Identity();
	Math::RigidTransform3d transform = Math::makeRigidTranslation(Math::Vector3d(0.25, 0.25, 0.25));

	compoundShape->addShape(box);

	auto calc1 = ContactCalculation::getDcdContactTable()[Math::SHAPE_TYPE_BOX][Math::SHAPE_TYPE_SPHERE];
	auto calc2 = ContactCalculation::getDcdContactTable()[Math::SHAPE_TYPE_COMPOUNDSHAPE][Math::SHAPE_TYPE_SPHERE];

	auto expected = calc1->calculateDcdContact(PosedShape(box, identity), PosedShape(sphere, transform));
	auto result = calc2->calculateDcdContact(PosedShape(compoundShape, identity), PosedShape(sphere, transform));
	ASSERT_EQ(1u, expected.size());

	contactsInfoEqualityTest(expected, result);
}

TEST_F(CompoundShapeDcdContactTest, MultipleShapes)
{
	auto box = std::make_shared<Math::BoxShape>(1.0, 1.0, 1.0);
	auto plane = std::make_shared<Math::PlaneShape>();

	auto planePose = Math::makeRigidTransform(Math::makeRotationQuaternion(0.01, Vector3d::UnitX().eval()),
		Vector3d::Zero());
	auto basePose = Math::makeRigidTranslation(Math::Vector3d(0.0, 0.01, 0.0));
	auto box1Pose = Math::makeRigidTransform(
						Math::makeRotationQuaternion(0.01, Vector3d(0.01, 0.01, 0.01)),
						Vector3d(0.1, 0.1, 0.1));

	auto box2Pose = Math::makeRigidTransform(
						Math::makeRotationQuaternion(-0.01, Vector3d(0.01, 0.01, 0.01)),
						Vector3d(0.0, -1.0, 0.0));

	auto compoundShape = std::make_shared<Math::CompoundShape>();

	compoundShape->addShape(box, box1Pose);
	compoundShape->addShape(box, box2Pose);

	auto calc1 = ContactCalculation::getDcdContactTable()[Math::SHAPE_TYPE_BOX][Math::SHAPE_TYPE_PLANE];
	auto calc2 = ContactCalculation::getDcdContactTable()[Math::SHAPE_TYPE_COMPOUNDSHAPE][Math::SHAPE_TYPE_PLANE];

	auto result = calc2->calculateDcdContact(PosedShape(compoundShape->getTransformed(basePose), basePose),
		PosedShape(plane, planePose));

	std::list<std::shared_ptr<Contact>> expected =
		calc1->calculateDcdContact(PosedShape(box, basePose * box1Pose), PosedShape(plane, planePose));
	for (auto& contact : expected)
	{
		contact->penetrationPoints.first.rigidLocalPosition.setValue(box1Pose *
			contact->penetrationPoints.first.rigidLocalPosition.getValue());
	}

	auto box2Expected = calc1->calculateDcdContact(PosedShape(box, basePose * box2Pose), PosedShape(plane, planePose));
	for (auto& contact : box2Expected)
	{
		contact->penetrationPoints.first.rigidLocalPosition.setValue(box2Pose *
			contact->penetrationPoints.first.rigidLocalPosition.getValue());
	}
	expected.splice(expected.end(), box2Expected);
	ASSERT_EQ(12u, expected.size());

	contactsInfoEqualityTest(expected, result);
}

}
}
