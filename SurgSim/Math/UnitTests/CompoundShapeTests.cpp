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

#include "SurgSim/Math/CompoundShape.h"
#include "SurgSim/Math/Shapes.h"
#include "SurgSim/Math/RigidTransform.h"
#include "SurgSim/Math/Vector.h"

#include <gtest/gtest.h>

namespace SurgSim
{

namespace Math
{

struct CompoundShapeTest : public ::testing::Test
{
public:

	virtual void SetUp()
	{
		compoundShape = std::make_shared<CompoundShape>();
		shape1 = std::make_shared<BoxShape>(1.0, 1.0, 1.0);
		shape2 = std::make_shared<BoxShape>(1.0, 2.0, 1.0);

		transform1 = makeRigidTranslation(Vector3d(-1.0, 0.0, 0.0));
		transform2 = makeRigidTranslation(Vector3d(1.0, 0.0, 0.0));
	}

	std::shared_ptr<CompoundShape> compoundShape;
	std::shared_ptr<Shape> shape1; 
	std::shared_ptr<Shape> shape2;

	RigidTransform3d transform1;
	RigidTransform3d transform2;
};

TEST_F(CompoundShapeTest, SimpleShapes)
{
	EXPECT_EQ(0u, compoundShape->getNumShapes());
	
	auto index = compoundShape->addShape(shape1);
	EXPECT_EQ(0u, index);
	EXPECT_EQ(1u, compoundShape->getNumShapes());

	index = compoundShape->addShape(shape2, transform2);
	EXPECT_EQ(1u, index);
	EXPECT_EQ(2u, compoundShape->getNumShapes());

	EXPECT_EQ(shape1, compoundShape->getShape(0));
	EXPECT_EQ(shape2, compoundShape->getShape(1));
	EXPECT_ANY_THROW(compoundShape->getShape(3));

	compoundShape->clearShapes();
	EXPECT_EQ(0u, compoundShape->getNumShapes());
}

TEST_F(CompoundShapeTest, Transforms)
{
	compoundShape->addShape(shape1);
	EXPECT_TRUE(RigidTransform3d::Identity().isApprox(compoundShape->getPose(0)));
	
	compoundShape->addShape(shape2, transform2);
	EXPECT_TRUE(transform2.isApprox(compoundShape->getPose(1)));

	compoundShape->setPose(0, transform2);
	EXPECT_TRUE(transform2.isApprox(compoundShape->getPose(0)));

	compoundShape->setPose(0, transform1);

	std::vector<RigidTransform3d> poses; 
	
	EXPECT_ANY_THROW(compoundShape->setPoses(poses));
	poses.push_back(transform1);
	poses.push_back(transform2);
	EXPECT_NO_THROW(compoundShape->setPoses(poses));

	EXPECT_TRUE(transform1.isApprox(compoundShape->getPose(0)));
	EXPECT_TRUE(transform2.isApprox(compoundShape->getPose(1)));
}

TEST_F(CompoundShapeTest, Volume)
{
	EXPECT_DOUBLE_EQ(0.0, compoundShape->getVolume());

	compoundShape->addShape(shape1);
	EXPECT_DOUBLE_EQ(1.0, compoundShape->getVolume());

	compoundShape->addShape(shape2, transform2);
	EXPECT_DOUBLE_EQ(3.0, compoundShape->getVolume());
}

TEST_F(CompoundShapeTest, Center)
{
	Vector3d center = Vector3d::Zero();
	EXPECT_TRUE(center.isApprox(compoundShape->getCenter())) 
		<< "Expected:" << center.transpose() 
		<< " Actual: " << compoundShape->getCenter().transpose();


	center = transform1.translation();
	compoundShape->addShape(shape1, transform1);
	EXPECT_TRUE(center.isApprox(compoundShape->getCenter()))
		<< "Expected:" << center.transpose() 
		<< " Actual: " << compoundShape->getCenter().transpose();
	EXPECT_DOUBLE_EQ(1.0, compoundShape->getVolume());

	center = Vector3d(1.0 - 2.0 / 3.0, 0.0, 0.0);
	compoundShape->addShape(shape2, transform2);
	EXPECT_TRUE(center.isApprox(compoundShape->getCenter()))
		<< "Expected:" << center.transpose()
		<< " Actual: " << compoundShape->getCenter().transpose();
	EXPECT_DOUBLE_EQ(3.0, compoundShape->getVolume());
}

TEST_F(CompoundShapeTest, SecondMomentOfVolumeBasic)
{
	auto zero = Math::Matrix33d::Zero();
	EXPECT_TRUE(zero.isApprox(compoundShape->getSecondMomentOfVolume()));

	auto box1 = std::make_shared<BoxShape>(1.0, 1.0, 1.0);
	auto box2 = std::make_shared<BoxShape>(2.0, 1.0, 1.0);

	auto left = makeRigidTranslation(Vector3d(-0.5, 0.0, 0.0));
	auto right = makeRigidTranslation(Vector3d(0.5, 0.0, 0.0));

	compoundShape->addShape(box1);

	auto shapeInertia = box1->getSecondMomentOfVolume();
	auto compoundInertia = compoundShape->getSecondMomentOfVolume();

	EXPECT_TRUE(shapeInertia.isApprox(compoundInertia));

	compoundShape->clearShapes();
	compoundShape->addShape(box1, left);
	compoundShape->addShape(box1, right);

	shapeInertia = box2->getSecondMomentOfVolume();
	compoundInertia = compoundShape->getSecondMomentOfVolume();

	EXPECT_TRUE(shapeInertia.isApprox(compoundInertia));
}

TEST_F(CompoundShapeTest, SecondMomentOfVolumeComplex)
{
	// Organisation of shape 
	// 
	//  tl Y-Axis
	//  tl ^
	//  bl -> 0, Z-Axis 
	//  bl
	//  bl

	auto base = std::make_shared<BoxShape>(1.0,5.0,2.0);

	auto l = std::make_shared<BoxShape>(1.0,5.0,1.0);
	compoundShape->addShape(l, makeRigidTranslation(Vector3d(0.0, 0.0, 0.5)));
	
	auto t = std::make_shared<BoxShape>(2.0,1.0,1.0);
	Quaterniond quat = makeRotationQuaternion(M_PI_2, Vector3d::UnitZ().eval());
	auto transform = makeRigidTransform(quat, Vector3d(0.0, 1.5, -0.5));
	compoundShape->addShape(t, transform);

	auto b = std::make_shared<BoxShape>(3.0,1.0,1.0);
	quat = makeRotationQuaternion(-M_PI_2, Vector3d::UnitZ().eval());
	transform = makeRigidTransform(quat, Vector3d(0.0, -1.0, -0.5));
	compoundShape->addShape(b, transform);

	auto shapeInertia = base->getSecondMomentOfVolume();
	auto compoundInertia = compoundShape->getSecondMomentOfVolume();
	EXPECT_TRUE(shapeInertia.isApprox(compoundInertia));
}

TEST_F(CompoundShapeTest, Properties)
{
	auto box = std::make_shared<BoxShape>(1.0, 5.0, 2.0);
	std::vector<CompoundShape::SubShape> shapes, result;
	shapes.emplace_back(shape1, transform1);
	shapes.emplace_back(box, transform2);


	ASSERT_NO_THROW(compoundShape->setValue("Shapes", shapes));

	ASSERT_NO_THROW(result = compoundShape->getValue<std::vector<CompoundShape::SubShape>>("Shapes"));

	// Spot check content
	EXPECT_EQ(2u, result.size());
	EXPECT_TRUE(transform2.isApprox(result[1].second));

}

}
}
